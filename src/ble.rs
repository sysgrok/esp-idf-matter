//! This module provides the ESP-IDF implementation of the GATT peripheral for the BTP protocol in `rs-matter`.

use core::borrow::Borrow;
use core::fmt::Debug;

use alloc::borrow::ToOwned;

use embassy_futures::select::select;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use enumset::enum_set;

use esp_idf_svc::bt::ble::gap::{BleGapEvent, EspBleGap};
use esp_idf_svc::bt::ble::gatt::server::{ConnectionId, EspGatts, GattsEvent, TransferId};
use esp_idf_svc::bt::ble::gatt::{
    AutoResponse, GattCharacteristic, GattDescriptor, GattId, GattInterface, GattResponse,
    GattServiceId, GattStatus, Handle, Permission, Property,
};
use esp_idf_svc::bt::{BdAddr, BleEnabled, BtDriver, BtStatus, BtUuid};
use esp_idf_svc::sys::{EspError, ESP_FAIL};

use log::{error, info, trace, warn};

use rs_matter_stack::ble::GattPeripheral;
use rs_matter_stack::matter::error::{Error, ErrorCode};
use rs_matter_stack::matter::transport::network::btp::{
    AdvData, Btp, C1_CHARACTERISTIC_UUID, C1_MAX_LEN, C2_CHARACTERISTIC_UUID, C2_MAX_LEN,
    MATTER_BLE_SERVICE_UUID16,
};
use rs_matter_stack::matter::transport::network::BtAddr;
use rs_matter_stack::matter::utils::cell::RefCell;
use rs_matter_stack::matter::utils::init::{init, Init};
use rs_matter_stack::matter::utils::select::Coalesce;
use rs_matter_stack::matter::utils::storage::Vec;
use rs_matter_stack::matter::utils::sync::blocking::Mutex;
use rs_matter_stack::matter::utils::sync::Signal;

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
    connection: Option<Connection>,
    conn_gen: usize,
    in_data: Vec<u8, MAX_MTU_SIZE>,
    /// The transaction ID corresponding to the incoming data (if non-empty)
    in_trans: u32,
    // TODO: Remove this once we can get access to the inner array inside `GattResponse`
    out_data: Vec<u8, MAX_MTU_SIZE>,
    /// Whether the current outgoing indication has been acknowledged by the peer or not
    out_nack: bool,
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
            connection: None,
            conn_gen: 0,
            in_data: Vec::new(),
            in_trans: 0,
            out_data: Vec::new(),
            out_nack: false,
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
            connection: None,
            conn_gen: 0,
            in_data <- Vec::init(),
            in_trans: 0,
            out_data <- Vec::init(),
            out_nack: false,
            response <- gatt_response::init(),
        })
    }
}

/// The `'static` state of the `EspBtpGattPeripheral` struct.
/// Isolated as a separate struct to allow for `const fn` construction
/// and static allocation.
pub struct EspBtpGattContext {
    /// The mutable state of the GATT peripheral, protected by a mutex for safe concurrent access from both the GATT event handlers
    /// and the async tasks processing the events and indications.
    state: Mutex<RefCell<State>, CriticalSectionRawMutex>,
    /// A signal used to awake the `process_incoming()` loop as there might be incoming data (c1 writes) to process.
    notify_process_incoming: Signal<Option<()>, CriticalSectionRawMutex>,
    /// A signal used to awake the `process_outgoing()` loop as there might be outgoing data (c2 indications) to process.
    notify_process_outgoing: Signal<Option<()>, CriticalSectionRawMutex>,
}

impl EspBtpGattContext {
    /// Create a new instance.
    #[allow(clippy::large_stack_frames)]
    #[inline(always)]
    pub const fn new() -> Self {
        Self {
            state: Mutex::new(RefCell::new(State::new())),
            notify_process_incoming: Signal::new(None),
            notify_process_outgoing: Signal::new(None),
        }
    }

    /// Return an in-place initializer for `EspBtpGattContext`.
    #[allow(clippy::large_stack_frames)]
    pub fn init() -> impl Init<Self> {
        init!(Self {
            state <- Mutex::init(RefCell::init(State::init())),
            notify_process_incoming <- Signal::init(None),
            notify_process_outgoing <- Signal::init(None),
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
            state.connection = None;
            state.in_data.clear();
            state.in_trans = 0;
            state.out_nack = false;
        });

        self.notify_process_incoming.modify(|s| {
            *s = None;

            (false, ())
        });

        self.notify_process_outgoing.modify(|s| {
            *s = None;

            (false, ())
        });

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
    pub async fn run(
        &mut self,
        btp: &Btp,
        service_name: &str,
        service_adv_data: &AdvData,
    ) -> Result<(), Error> {
        let gap = EspBleGap::new(&self.driver).map_err(to_matter_err)?;
        let gatts = EspGatts::new(&self.driver).map_err(to_matter_err)?;

        let event_ctx = GattEventContext::new(self.app_id, &gap, &gatts, self.context);

        info!("BLE Gap and Gatts initialized");

        unsafe {
            gap.subscribe_nonstatic(|event| {
                event_ctx.check_esp_status(event_ctx.on_gap_event(event));
            })
            .map_err(to_matter_err)?;
        }

        let adv_data = service_adv_data.clone();
        let service_name = service_name.to_owned();

        unsafe {
            gatts
                .subscribe_nonstatic(|(gatt_if, event)| {
                    event_ctx.check_esp_status(event_ctx.on_gatts_event(
                        &service_name,
                        &adv_data,
                        gatt_if,
                        event,
                    ))
                })
                .map_err(to_matter_err)?;
        }

        info!("BLE Gap and Gatts subscriptions initialized");

        gatts.register_app(self.app_id).map_err(to_matter_err)?;

        info!("Gatts BTP app registered");

        select(
            self.process_incoming(btp, &gatts),
            self.process_outgoing(btp, &gatts),
        )
        .coalesce()
        .await
    }

    /// Process incoming writes on characteristic `C1` from a remote peer.
    ///
    /// While it might seem that `process_incoming` can be called directly from `GattEventContext::on_gatts_event`,
    /// this is not generally possible because `Btp` might not be `Sync`, while `GattEventContext::on_gatts_event` should be.
    async fn process_incoming<T>(&self, btp: &Btp, gatts: &EspGatts<'d, M, T>) -> Result<(), Error>
    where
        T: Borrow<BtDriver<'d, M>>,
    {
        let mut generation = None;

        loop {
            let processed = self.context.state.lock(|state| {
                let mut state = state.borrow_mut();

                if let Some(connection) = state.connection.as_ref() {
                    if generation != Some(state.conn_gen) {
                        btp.reset();
                        generation = Some(state.conn_gen);
                    }

                    if !state.in_data.is_empty() {
                        btp.process_incoming(
                            connection.mtu,
                            BtAddr(connection.peer.addr()),
                            &state.in_data,
                        )?;

                        gatts
                            .send_response(
                                state.gatt_if.unwrap_or(0),
                                connection.conn_id,
                                state.in_trans,
                                GattStatus::Ok,
                                None,
                            )
                            .map_err(to_matter_err)?;

                        state.in_data.clear();

                        return Ok::<_, Error>(true);
                    }
                }

                Ok(false)
            })?;

            if !processed {
                self.context.notify_process_incoming.wait_signalled().await;
            }
        }
    }

    /// Indicate new data on characteristic `C2` to a remote peer.
    async fn process_outgoing<T>(&self, btp: &Btp, gatts: &EspGatts<'d, M, T>) -> Result<(), Error>
    where
        T: Borrow<BtDriver<'d, M>>,
    {
        loop {
            let processed = self.context.state.lock(|state| {
                let mut state = state.borrow_mut();
                let state = &mut *state;

                let Some(gatt_if) = state.gatt_if else {
                    return Ok::<_, Error>(false);
                };

                let Some(c2_handle) = state.c2_handle else {
                    return Ok(false);
                };

                let Some(conn) = state.connection.as_ref() else {
                    return Ok(false);
                };

                if !conn.subscribed {
                    // Peer is not subscribed to indications,
                    // so we shouldn't send anything
                    return Ok(false);
                }

                if state.out_nack {
                    // The previous indication has not been acknowledged
                    // by the peer yet, so we shouldn't send a new one
                    return Ok(false);
                }

                state.out_data.resize_default(MAX_MTU_SIZE).unwrap();

                let len = btp.process_outgoing(conn.mtu, &mut state.out_data)?;
                if len > 0 {
                    let data = &state.out_data[..len];

                    gatts
                        .indicate(gatt_if, conn.conn_id, c2_handle, data)
                        .map_err(to_matter_err)?;

                    // Mark the current outgoing indication as not acknowledged
                    // until we receive the acknowledgment from the peer.
                    state.out_nack = true;

                    trace!("Indicated {} bytes", data.len());

                    Ok(true)
                } else {
                    Ok(false)
                }
            })?;

            if !processed {
                select(
                    btp.wait_outgoing(),
                    self.context.notify_process_outgoing.wait_signalled(),
                )
                .coalesce()
                .await;
            }
        }
    }
}

impl<M> GattPeripheral for EspBtpGattPeripheral<'_, '_, M>
where
    M: BleEnabled,
{
    async fn run(
        &mut self,
        btp: &Btp,
        service_name: &str,
        adv_data: &AdvData,
    ) -> Result<(), Error> {
        EspBtpGattPeripheral::run(self, btp, service_name, adv_data).await
    }
}

struct GattEventContext<'a, 'd, M, T>
where
    T: Borrow<BtDriver<'d, M>>,
    M: BleEnabled,
{
    app_id: u16,
    gap: &'a EspBleGap<'d, M, T>,
    gatts: &'a EspGatts<'d, M, T>,
    ctx: &'a EspBtpGattContext,
}

impl<'a, 'd, M, T> GattEventContext<'a, 'd, M, T>
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

    fn on_gap_event(&self, event: BleGapEvent) -> Result<(), EspError> {
        if let BleGapEvent::RawAdvertisingConfigured(status) = event {
            self.check_bt_status(status)?;
            self.gap.start_advertising()?;
        }

        Ok(())
    }

    fn on_gatts_event(
        &self,
        service_name: &str,
        service_adv_data: &AdvData,
        gatt_if: GattInterface,
        event: GattsEvent,
    ) -> Result<(), EspError> {
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
            GattsEvent::PeerConnected { conn_id, addr, .. }
                if self.create_conn(conn_id, addr)? =>
            {
                self.gap.stop_advertising()?;
            }
            GattsEvent::PeerDisconnected { addr, .. } if self.delete_conn(addr)? => {
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
                    gatt_if, conn_id, trans_id, addr, handle, offset, need_rsp, is_prep, value,
                )?;
            }
            GattsEvent::Confirm { status, .. } => {
                self.check_gatt_status(status)?;
                self.ctx.state.lock(|state| {
                    state.borrow_mut().out_nack = false;
                    // Awake the `process_indicate()` loop now that
                    // the previous indication has been acknowledged by the peer.
                    self.ctx.notify_process_outgoing.signal(());
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
                .connection
                .iter_mut()
                .find(|conn| conn.conn_id == conn_id)
            {
                conn.mtu = Some(mtu);
            }
        });

        Ok(())
    }

    fn create_conn(&self, conn_id: ConnectionId, addr: BdAddr) -> Result<bool, EspError> {
        let done = self.ctx.state.lock(|state| {
            let mut state = state.borrow_mut();
            if state.connection.is_none() {
                state.connection = Some(Connection {
                    peer: addr,
                    conn_id,
                    subscribed: false,
                    mtu: None,
                });

                state.in_data.clear();
                state.out_nack = false;

                state.conn_gen += 1;
                self.ctx.notify_process_incoming.signal(());

                true
            } else {
                false
            }
        });

        if done {
            self.gap.set_conn_params_conf(addr, 10, 20, 0, 400)?;
        }

        Ok(done)
    }

    fn delete_conn(&self, addr: BdAddr) -> Result<bool, EspError> {
        let done = self.ctx.state.lock(|state| {
            let mut state = state.borrow_mut();
            if state
                .connection
                .as_ref()
                .map(|conn| conn.peer == addr)
                .unwrap_or(false)
            {
                state.connection = None;

                state.conn_gen += 1;
                self.ctx.notify_process_incoming.signal(());

                true
            } else {
                false
            }
        });

        Ok(done)
    }

    #[allow(clippy::too_many_arguments)]
    fn write(
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
    ) -> Result<(), EspError> {
        let respond = self.ctx.state.lock(|state| {
            let mut state = state.borrow_mut();
            let c1_handle = state.c1_handle;
            let c2_cccd_handle = state.c2_cccd_handle;

            let Some(conn) = state.connection.as_mut() else {
                return false;
            };

            if conn.conn_id != conn_id {
                return false;
            }

            if c2_cccd_handle == Some(handle) {
                if offset == 0 && value.len() == 2 {
                    let value = u16::from_le_bytes([value[0], value[1]]);
                    if value == 0x02 {
                        if !conn.subscribed {
                            conn.subscribed = true;
                            self.ctx.notify_process_incoming.signal(());
                            self.ctx.notify_process_outgoing.signal(());
                            return true;
                        }
                    } else if conn.subscribed {
                        conn.subscribed = false;
                        self.ctx.notify_process_incoming.signal(());
                        return true;
                    }
                }
            } else if c1_handle == Some(handle) && offset == 0 {
                let address = BtAddr(addr.into());

                state.in_trans = trans_id;
                state.in_data.clear();
                if state.in_data.extend_from_slice(value).is_err() {
                    warn!("Dropping {} bytes on c1: in_data buffer full", value.len());
                }

                trace!("Got {} bytes to {address}", value.len());

                self.ctx.notify_process_incoming.signal(());

                // Do NOT return `true` here even though we have to send a response to the write request for c1,
                // because the incoming data has NOT yet been processed by `process_incoming()`.
            }

            false
        });

        if respond {
            self.send_write_response(
                gatt_if, conn_id, trans_id, handle, offset, need_rsp, is_prep, value,
            )?;
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

fn to_matter_err<E: Debug>(err: E) -> Error {
    error!("BLE error: {:?}", err);

    ErrorCode::BtpError.into()
}
