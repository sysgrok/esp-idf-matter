//! This module provides the ESP-IDF implementation of the GATT peripheral for the BTP protocol in `rs-matter`.

use core::borrow::Borrow;

use alloc::borrow::ToOwned;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use enumset::enum_set;

use esp_idf_svc::bt::ble::gap::{BleGapEvent, EspBleGap};
use esp_idf_svc::bt::ble::gatt::server::{ConnectionId, EspGatts, GattsEvent, TransferId};
use esp_idf_svc::bt::ble::gatt::{
    AutoResponse, GattCharacteristic, GattDescriptor, GattId, GattInterface, GattResponse,
    GattServiceId, GattStatus, Handle, Permission, Property,
};
use esp_idf_svc::bt::{BdAddr, BleEnabled, BtDriver, BtStatus, BtUuid};
use esp_idf_svc::sys::{EspError, ESP_ERR_INVALID_STATE, ESP_FAIL};

use log::{error, info, trace, warn};

use rs_matter_stack::matter::error::ErrorCode;
use rs_matter_stack::matter::transport::network::btp::{
    AdvData, GattPeripheral, GattPeripheralEvent, C1_CHARACTERISTIC_UUID, C1_MAX_LEN,
    C2_CHARACTERISTIC_UUID, C2_MAX_LEN, MATTER_BLE_SERVICE_UUID16, MAX_BTP_SESSIONS,
};
use rs_matter_stack::matter::transport::network::BtAddr;
use rs_matter_stack::matter::utils::cell::RefCell;
use rs_matter_stack::matter::utils::init::{init, Init};
use rs_matter_stack::matter::utils::sync::blocking::Mutex;
use rs_matter_stack::matter::utils::sync::{IfMutex, Signal};

const MAX_CONNECTIONS: usize = MAX_BTP_SESSIONS;
const MAX_MTU_SIZE: usize = 512;

#[derive(Debug, Clone)]
struct Connection {
    peer: BdAddr,
    conn_id: Handle,
    subscribed: bool,
    mtu: Option<u16>,
}

struct State {
    gatt_if: Option<GattInterface>,
    service_handle: Option<Handle>,
    c1_handle: Option<Handle>,
    c2_handle: Option<Handle>,
    c2_cccd_handle: Option<Handle>,
    connections: rs_matter_stack::matter::utils::storage::Vec<Connection, MAX_CONNECTIONS>,
    response: GattResponse,
}

impl State {
    #[inline(always)]
    const fn new() -> Self {
        Self {
            gatt_if: None,
            service_handle: None,
            c1_handle: None,
            c2_handle: None,
            c2_cccd_handle: None,
            connections: rs_matter_stack::matter::utils::storage::Vec::new(),
            response: GattResponse::new(),
        }
    }

    fn init() -> impl Init<Self> {
        init!(Self {
            gatt_if: None,
            service_handle: None,
            c1_handle: None,
            c2_handle: None,
            c2_cccd_handle: None,
            connections <- rs_matter_stack::matter::utils::storage::Vec::init(),
            response <- gatt_response::init(),
        })
    }
}

#[derive(Debug)]
struct IndBuffer {
    addr: BtAddr,
    data: rs_matter_stack::matter::utils::storage::Vec<u8, MAX_MTU_SIZE>,
}

impl IndBuffer {
    #[inline(always)]
    const fn new() -> Self {
        Self {
            addr: BtAddr([0; 6]),
            data: rs_matter_stack::matter::utils::storage::Vec::new(),
        }
    }

    fn init() -> impl Init<Self> {
        init!(Self {
            addr: BtAddr([0; 6]),
            data <- rs_matter_stack::matter::utils::storage::Vec::init(),
        })
    }
}

/// The `'static` state of the `EspBtpGattPeripheral` struct.
/// Isolated as a separate struct to allow for `const fn` construction
/// and static allocation.
// TODO: Revert to `EspRawMutex` when `esp-idf-svc` is updated to `embassy-sync 0.7`
pub struct EspBtpGattContext {
    state: Mutex<CriticalSectionRawMutex /*EspRawMutex*/, RefCell<State>>,
    ind: IfMutex<CriticalSectionRawMutex /*EspRawMutex*/, IndBuffer>,
    ind_in_flight: Signal<CriticalSectionRawMutex /*EspRawMutex*/, bool>,
}

impl EspBtpGattContext {
    /// Create a new instance.
    #[allow(clippy::large_stack_frames)]
    #[inline(always)]
    pub const fn new() -> Self {
        Self {
            state: Mutex::new(RefCell::new(State::new())),
            ind: IfMutex::new(IndBuffer::new()),
            ind_in_flight: Signal::new(false),
        }
    }

    /// Return an in-place initializer for `EspBtpGattContext`.
    #[allow(clippy::large_stack_frames)]
    pub fn init() -> impl Init<Self> {
        init!(Self {
            state <- Mutex::init(RefCell::init(State::init())),
            ind <- IfMutex::init(IndBuffer::init()),
            ind_in_flight: Signal::new(false),
        })
    }

    pub(crate) fn reset(&self) -> Result<(), EspError> {
        self.state.lock(|state| {
            let mut state = state.borrow_mut();

            state.gatt_if = None;
            state.service_handle = None;
            state.c1_handle = None;
            state.c2_handle = None;
            state.c2_cccd_handle = None;
        });

        self.ind_in_flight.modify(|ind_inf_flight| {
            *ind_inf_flight = false;
            (false, ())
        });

        self.ind
            .try_lock()
            .map(|mut ind| {
                ind.data.clear();
            })
            .map_err(|_| EspError::from_infallible::<{ ESP_ERR_INVALID_STATE }>())?;

        Ok(())
    }
}

impl Default for EspBtpGattContext {
    // TODO
    #[allow(clippy::large_stack_frames)]
    #[inline(always)]
    fn default() -> Self {
        Self::new()
    }
}

/// A GATT peripheral implementation for the BTP protocol in `rs-matter` via ESP-IDF.
/// Implements the `GattPeripheral` trait.
pub struct EspBtpGattPeripheral<'a, 'd, M>
where
    M: BleEnabled,
{
    app_id: u16,
    driver: BtDriver<'d, M>,
    context: &'a EspBtpGattContext,
}

impl<'a, 'd, M> EspBtpGattPeripheral<'a, 'd, M>
where
    M: BleEnabled,
{
    /// Create a new instance.
    ///
    /// Creation might fail if the GATT context cannot be reset, so user should ensure
    /// that there are no other GATT peripherals running before calling this function.
    pub fn new(
        app_id: u16,
        driver: BtDriver<'d, M>,
        context: &'a EspBtpGattContext,
    ) -> Result<Self, EspError> {
        context.reset()?;

        Ok(Self {
            app_id,
            driver,
            context,
        })
    }

    /// Run the GATT peripheral.
    pub async fn run<F>(
        &self,
        service_name: &str,
        service_adv_data: &AdvData,
        mut callback: F,
    ) -> Result<(), EspError>
    where
        F: FnMut(GattPeripheralEvent) + Send + 'd,
    {
        let gap = EspBleGap::new(&self.driver)?;
        let gatts = EspGatts::new(&self.driver)?;

        info!("BLE Gap and Gatts initialized");

        unsafe {
            gap.subscribe_nonstatic(|event| {
                let ctx = GattExecContext::new(self.app_id, &gap, &gatts, self.context);

                ctx.check_esp_status(ctx.on_gap_event(event));
            })?;
        }

        let adv_data = service_adv_data.clone();
        let service_name = service_name.to_owned();

        unsafe {
            gatts.subscribe_nonstatic(|(gatt_if, event)| {
                let ctx = GattExecContext::new(self.app_id, &gap, &gatts, self.context);

                ctx.check_esp_status(ctx.on_gatts_event(
                    &service_name,
                    &adv_data,
                    gatt_if,
                    event,
                    &mut callback,
                ))
            })?;
        }

        info!("BLE Gap and Gatts subscriptions initialized");

        gatts.register_app(self.app_id)?;

        info!("Gatts BTP app registered");

        loop {
            self.context
                .ind_in_flight
                .wait(|in_flight| (!*in_flight).then_some(()))
                .await;

            let mut ind = self.context.ind.lock_if(|ind| !ind.data.is_empty()).await;

            let ctx = GattExecContext::new(self.app_id, &gap, &gatts, self.context);

            self.context.ind_in_flight.modify(|in_flight| {
                if !*in_flight {
                    *in_flight = true;
                } else {
                    // Should not happen as the only code that sets in flight to `true`
                    // is here
                    unreachable!();
                }

                (true, ())
            });

            // TODO: Is this asynchronous?
            let result = ctx.indicate(&ind.data, ind.addr);
            if let Err(e) = result {
                error!("GATT indicate failed: {e}");
            }

            ind.data.clear();
        }
    }

    /// Indicate new data on characteristic `C2` to a remote peer.
    pub async fn indicate(&self, data: &[u8], address: BtAddr) -> Result<(), EspError> {
        self.context
            .ind
            .with(|ind| {
                if ind.data.is_empty() {
                    ind.data.extend_from_slice(data).unwrap();
                    ind.addr = address;

                    Some(())
                } else {
                    None
                }
            })
            .await;

        Ok(())
    }
}

impl<M> GattPeripheral for EspBtpGattPeripheral<'_, '_, M>
where
    M: BleEnabled,
{
    async fn run<F>(
        &self,
        service_name: &str,
        adv_data: &AdvData,
        callback: F,
    ) -> Result<(), rs_matter_stack::matter::error::Error>
    where
        F: FnMut(GattPeripheralEvent) + Send + Clone + 'static,
    {
        EspBtpGattPeripheral::run(self, service_name, adv_data, callback)
            .await
            .map_err(|_| ErrorCode::BtpError)?;

        Ok(())
    }

    async fn indicate(
        &self,
        data: &[u8],
        address: BtAddr,
    ) -> Result<(), rs_matter_stack::matter::error::Error> {
        EspBtpGattPeripheral::indicate(self, data, address)
            .await
            .map_err(|_| ErrorCode::BtpError)?;

        Ok(())
    }
}

struct GattExecContext<'a, 'd, M, T>
where
    T: Borrow<BtDriver<'d, M>>,
    M: BleEnabled,
{
    app_id: u16,
    gap: &'a EspBleGap<'d, M, T>,
    gatts: &'a EspGatts<'d, M, T>,
    ctx: &'a EspBtpGattContext,
}

impl<'a, 'd, M, T> GattExecContext<'a, 'd, M, T>
where
    T: Borrow<BtDriver<'d, M>> + Clone,
    M: BleEnabled,
{
    fn new(
        app_id: u16,
        gap: &'a EspBleGap<'d, M, T>,
        gatts: &'a EspGatts<'d, M, T>,
        ctx: &'a EspBtpGattContext,
    ) -> Self {
        Self {
            app_id,
            gap,
            gatts,
            ctx,
        }
    }

    fn indicate(&self, data: &[u8], address: BtAddr) -> Result<bool, EspError> {
        let conn = self.ctx.state.lock(|state| {
            let state = state.borrow();

            let gatts_if = state.gatt_if?;
            let c2_handle = state.c2_handle?;

            let conn = state
                .connections
                .iter()
                .find(|conn| conn.peer.addr() == address.0 && conn.subscribed)?;

            Some((gatts_if, conn.conn_id, c2_handle))
        });

        if let Some((gatts_if, conn_id, attr_handle)) = conn {
            self.gatts.indicate(gatts_if, conn_id, attr_handle, data)?;

            trace!("Indicated {} bytes to {address}", data.len());

            Ok(true)
        } else {
            warn!("No connection for {address}, cannot indicate");

            Ok(false)
        }
    }

    fn on_gap_event(&self, event: BleGapEvent) -> Result<(), EspError> {
        if let BleGapEvent::RawAdvertisingConfigured(status) = event {
            self.check_bt_status(status)?;
            self.gap.start_advertising()?;
        }

        Ok(())
    }

    fn on_gatts_event<F>(
        &self,
        service_name: &str,
        service_adv_data: &AdvData,
        gatt_if: GattInterface,
        event: GattsEvent,
        mut callback: F,
    ) -> Result<(), EspError>
    where
        F: FnMut(GattPeripheralEvent),
    {
        match event {
            GattsEvent::ServiceRegistered { status, app_id } => {
                self.check_gatt_status(status)?;
                if self.app_id == app_id {
                    self.create_service(gatt_if, service_name, service_adv_data)?;
                }
            }
            GattsEvent::ServiceCreated {
                status,
                service_handle,
                ..
            } => {
                self.check_gatt_status(status)?;
                self.configure_and_start_service(service_handle)?;
            }
            GattsEvent::CharacteristicAdded {
                status,
                attr_handle,
                service_handle,
                char_uuid,
            } => {
                self.check_gatt_status(status)?;
                self.register_characteristic(service_handle, attr_handle, char_uuid)?;
            }
            GattsEvent::DescriptorAdded {
                status,
                attr_handle,
                service_handle,
                descr_uuid,
            } => {
                self.check_gatt_status(status)?;
                self.register_cccd_descriptor(service_handle, attr_handle, descr_uuid)?;
            }
            GattsEvent::ServiceDeleted {
                status,
                service_handle,
            } => {
                self.check_gatt_status(status)?;
                self.delete_service(service_handle)?;
            }
            GattsEvent::ServiceUnregistered {
                status,
                service_handle,
                ..
            } => {
                self.check_gatt_status(status)?;
                self.unregister_service(service_handle)?;
            }
            GattsEvent::Mtu { conn_id, mtu } => {
                self.register_conn_mtu(conn_id, mtu)?;
            }
            GattsEvent::PeerConnected { conn_id, addr, .. } => {
                self.create_conn(conn_id, addr)?;
            }
            GattsEvent::PeerDisconnected { addr, .. } => {
                self.delete_conn(addr, &mut callback)?;
                self.gap.start_advertising()?;
            }
            GattsEvent::Write {
                conn_id,
                trans_id,
                addr,
                handle,
                offset,
                need_rsp,
                is_prep,
                value,
            } => {
                self.write(
                    gatt_if,
                    conn_id,
                    trans_id,
                    addr,
                    handle,
                    offset,
                    need_rsp,
                    is_prep,
                    value,
                    &mut callback,
                )?;
            }
            GattsEvent::Confirm { status, .. } => {
                self.check_gatt_status(status)?;
                self.ctx.ind_in_flight.modify(|in_flight| {
                    if *in_flight {
                        *in_flight = false;
                    } else {
                        // Should not happen: means we have received a confirmation for
                        // an indication we did not send.
                        unreachable!();
                    }

                    (true, ())
                });
            }
            _ => (),
        }

        Ok(())
    }

    fn check_esp_status(&self, status: Result<(), EspError>) {
        if let Err(e) = status {
            warn!("Got status: {e:?}");
        }
    }

    fn check_bt_status(&self, status: BtStatus) -> Result<(), EspError> {
        if !matches!(status, BtStatus::Success) {
            warn!("Got status: {status:?}");
            Err(EspError::from_infallible::<ESP_FAIL>())
        } else {
            Ok(())
        }
    }

    fn check_gatt_status(&self, status: GattStatus) -> Result<(), EspError> {
        if !matches!(status, GattStatus::Ok) {
            warn!("Got status: {status:?}");
            Err(EspError::from_infallible::<ESP_FAIL>())
        } else {
            Ok(())
        }
    }

    fn create_service(
        &self,
        gatt_if: GattInterface,
        service_name: &str,
        service_adv_data: &AdvData,
    ) -> Result<(), EspError> {
        self.ctx.state.lock(|state| {
            state.borrow_mut().gatt_if = Some(gatt_if);
        });

        self.gap.set_device_name(service_name)?;
        self.gap
            .set_raw_adv_conf(&service_adv_data.iter().collect::<heapless::Vec<_, 32>>())?;
        self.gatts.create_service(
            gatt_if,
            &GattServiceId {
                id: GattId {
                    uuid: BtUuid::uuid16(MATTER_BLE_SERVICE_UUID16),
                    inst_id: 0,
                },
                is_primary: true,
            },
            8,
        )?;

        Ok(())
    }

    fn delete_service(&self, service_handle: Handle) -> Result<(), EspError> {
        self.ctx.state.lock(|state| {
            if state.borrow().service_handle == Some(service_handle) {
                state.borrow_mut().c1_handle = None;
                state.borrow_mut().c2_handle = None;
                state.borrow_mut().c2_cccd_handle = None;
            }
        });

        Ok(())
    }

    fn unregister_service(&self, service_handle: Handle) -> Result<(), EspError> {
        self.ctx.state.lock(|state| {
            if state.borrow().service_handle == Some(service_handle) {
                state.borrow_mut().gatt_if = None;
                state.borrow_mut().service_handle = None;
            }
        });

        Ok(())
    }

    fn configure_and_start_service(&self, service_handle: Handle) -> Result<(), EspError> {
        self.ctx.state.lock(|state| {
            state.borrow_mut().service_handle = Some(service_handle);
        });

        self.gatts.start_service(service_handle)?;
        self.add_characteristics(service_handle)?;

        Ok(())
    }

    fn add_characteristics(&self, service_handle: Handle) -> Result<(), EspError> {
        self.gatts.add_characteristic(
            service_handle,
            &GattCharacteristic {
                uuid: BtUuid::uuid128(C1_CHARACTERISTIC_UUID),
                permissions: enum_set!(Permission::Write),
                properties: enum_set!(Property::Write),
                max_len: C1_MAX_LEN,
                auto_rsp: AutoResponse::ByApp,
            },
            &[],
        )?;

        self.gatts.add_characteristic(
            service_handle,
            &GattCharacteristic {
                uuid: BtUuid::uuid128(C2_CHARACTERISTIC_UUID),
                permissions: enum_set!(Permission::Write | Permission::Read),
                properties: enum_set!(Property::Indicate),
                max_len: C2_MAX_LEN,
                auto_rsp: AutoResponse::ByApp,
            },
            &[],
        )?;

        Ok(())
    }

    fn register_characteristic(
        &self,
        service_handle: Handle,
        attr_handle: Handle,
        char_uuid: BtUuid,
    ) -> Result<(), EspError> {
        let c2 = self.ctx.state.lock(|state| {
            if state.borrow().service_handle != Some(service_handle) {
                return false;
            }

            if char_uuid == BtUuid::uuid128(C1_CHARACTERISTIC_UUID) {
                state.borrow_mut().c1_handle = Some(attr_handle);

                false
            } else if char_uuid == BtUuid::uuid128(C2_CHARACTERISTIC_UUID) {
                state.borrow_mut().c2_handle = Some(attr_handle);

                true
            } else {
                false
            }
        });

        if c2 {
            self.gatts.add_descriptor(
                service_handle,
                &GattDescriptor {
                    uuid: BtUuid::uuid16(0x2902), // CCCD
                    permissions: enum_set!(Permission::Read | Permission::Write),
                },
            )?;
        }

        Ok(())
    }

    fn register_cccd_descriptor(
        &self,
        service_handle: Handle,
        attr_handle: Handle,
        descr_uuid: BtUuid,
    ) -> Result<(), EspError> {
        self.ctx.state.lock(|state| {
            if descr_uuid == BtUuid::uuid16(0x2902)
                && state.borrow().service_handle == Some(service_handle)
            {
                state.borrow_mut().c2_cccd_handle = Some(attr_handle);
            }
        });

        Ok(())
    }

    fn register_conn_mtu(&self, conn_id: ConnectionId, mtu: u16) -> Result<(), EspError> {
        self.ctx.state.lock(|state| {
            let mut state = state.borrow_mut();
            if let Some(conn) = state
                .connections
                .iter_mut()
                .find(|conn| conn.conn_id == conn_id)
            {
                conn.mtu = Some(mtu);
            }
        });

        Ok(())
    }

    fn create_conn(&self, conn_id: ConnectionId, addr: BdAddr) -> Result<(), EspError> {
        let added = self.ctx.state.lock(|state| {
            let mut state = state.borrow_mut();
            if state.connections.len() < MAX_CONNECTIONS {
                state
                    .connections
                    .push(Connection {
                        peer: addr,
                        conn_id,
                        subscribed: false,
                        mtu: None,
                    })
                    .map_err(|_| ())
                    .unwrap();

                true
            } else {
                false
            }
        });

        if added {
            self.gap.set_conn_params_conf(addr, 10, 20, 0, 400)?;
        }

        Ok(())
    }

    fn delete_conn<F>(&self, addr: BdAddr, callback: &mut F) -> Result<(), EspError>
    where
        F: FnMut(GattPeripheralEvent),
    {
        self.ctx.state.lock(|state| {
            let mut state = state.borrow_mut();
            if let Some(index) = state
                .connections
                .iter()
                .position(|Connection { peer, .. }| *peer == addr)
            {
                state.connections.swap_remove(index);
            }
        });

        callback(GattPeripheralEvent::NotifyUnsubscribed(BtAddr(addr.into())));

        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn write<F>(
        &self,
        gatt_if: GattInterface,
        conn_id: ConnectionId,
        trans_id: TransferId,
        addr: BdAddr,
        handle: Handle,
        offset: u16,
        need_rsp: bool,
        is_prep: bool,
        value: &[u8],
        callback: &mut F,
    ) -> Result<(), EspError>
    where
        F: FnMut(GattPeripheralEvent),
    {
        let event = self.ctx.state.lock(|state| {
            let mut state = state.borrow_mut();
            let c1_handle = state.c1_handle;
            let c2_cccd_handle = state.c2_cccd_handle;

            let conn = state
                .connections
                .iter_mut()
                .find(|conn| conn.conn_id == conn_id)?;

            if c2_cccd_handle == Some(handle) {
                if offset == 0 && value.len() == 2 {
                    let value = u16::from_le_bytes([value[0], value[1]]);
                    if value == 0x02 {
                        if !conn.subscribed {
                            conn.subscribed = true;
                            return Some(GattPeripheralEvent::NotifySubscribed(BtAddr(
                                addr.into(),
                            )));
                        }
                    } else if conn.subscribed {
                        conn.subscribed = false;
                        return Some(GattPeripheralEvent::NotifyUnsubscribed(BtAddr(addr.into())));
                    }
                }
            } else if c1_handle == Some(handle) && offset == 0 {
                let address = BtAddr(addr.into());

                trace!("Got {} bytes to {address}", value.len());

                return Some(GattPeripheralEvent::Write {
                    address,
                    data: value,
                    gatt_mtu: conn.mtu,
                });
            }

            None
        });

        if let Some(event) = event {
            self.send_write_response(
                gatt_if, conn_id, trans_id, handle, offset, need_rsp, is_prep, value,
            )?;

            callback(event);
        }

        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn send_write_response(
        &self,
        gatt_if: GattInterface,
        conn_id: ConnectionId,
        trans_id: TransferId,
        handle: Handle,
        offset: u16,
        need_rsp: bool,
        is_prep: bool,
        value: &[u8],
    ) -> Result<(), EspError> {
        if !need_rsp {
            return Ok(());
        }

        if is_prep {
            self.ctx.state.lock(|state| {
                let mut state = state.borrow_mut();

                state
                    .response
                    .attr_handle(handle)
                    .auth_req(0)
                    .offset(offset)
                    .value(value)
                    .map_err(|_| EspError::from_infallible::<ESP_FAIL>())?;

                self.gatts.send_response(
                    gatt_if,
                    conn_id,
                    trans_id,
                    GattStatus::Ok,
                    Some(&state.response),
                )
            })?;
        } else {
            self.gatts
                .send_response(gatt_if, conn_id, trans_id, GattStatus::Ok, None)?;
        }

        Ok(())
    }
}

mod gatt_response {
    use esp_idf_svc::bt::ble::gatt::GattResponse;
    use esp_idf_svc::sys::{esp_gatt_rsp_t, esp_gatt_value_t};

    use rs_matter_stack::matter::utils::init::{init, init_from_closure, zeroed, Init};

    /// Return an in-place initializer for `GattResponse`.
    ///
    /// Works by initializing the `GattResponse` struct in-place using the `esp_gatt_rsp_t` type,
    /// which is possible because `GattResponse` is a `#[repr(transparent)]` newtype over `esp_gatt_rsp_t`.
    pub fn init() -> impl Init<GattResponse> {
        unsafe {
            init_from_closure(|slot: *mut GattResponse| {
                let slot = slot as *mut esp_gatt_rsp_t;

                init_esp_gatt_response().__init(slot)
            })
        }
    }

    fn init_esp_gatt_response() -> impl Init<esp_gatt_rsp_t> {
        init!(esp_gatt_rsp_t {
           attr_value <- init!(esp_gatt_value_t {
               len: 0,
               value <- zeroed(),
               handle: 0,
               offset: 0,
               auth_req: 0,
           }),
        })
    }
}
