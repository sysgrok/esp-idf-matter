//! A Gatt peripheral and GATT central implementations for the Matter BTP protocol, on top of the
//! `esp_idf_svc::ble` NimBLE API.

use core::sync::atomic::{AtomicBool, AtomicPtr, AtomicU16, Ordering};

use embassy_futures::select::select;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use esp_idf_svc::ble::gap::{conn_find, BleAdvParams, GapEvent};
use esp_idf_svc::ble::gatt::att_mtu;
use esp_idf_svc::ble::gatt::server::{BleGattRegister, GattsEvent};
use esp_idf_svc::ble::{ensure_addr, BleDriver, BleError, BleUuid, HostEvent};
use esp_idf_svc::gatt_services;
use esp_idf_svc::hal::modem::BluetoothModemPeripheral;
use esp_idf_svc::sys::{
    ble_gatt_svc_def, EspError, BLE_ATT_ERR_INSUFFICIENT_RES, BLE_GAP_CONN_MODE_UND,
    BLE_GAP_DISC_MODE_GEN, BLE_HS_EDONE, BLE_OWN_ADDR_PUBLIC,
};

use ::log::{error, info, trace, warn};

use rs_matter_stack::ble::GattPeripheral;
use rs_matter_stack::matter::error::{Error, ErrorCode};
use rs_matter_stack::matter::transport::network::btp::{
    AdvData, Btp, C1_CHARACTERISTIC_UUID, C2_CHARACTERISTIC_UUID, C3_CHARACTERISTIC_UUID,
    MATTER_BLE_SERVICE_UUID16,
};
use rs_matter_stack::matter::transport::network::BtAddr;
use rs_matter_stack::matter::utils::cell::RefCell;
use rs_matter_stack::matter::utils::init::{init, Init};
use rs_matter_stack::matter::utils::select::Coalesce;
use rs_matter_stack::matter::utils::storage::Vec;
use rs_matter_stack::matter::utils::sync::blocking::Mutex;
use rs_matter_stack::matter::utils::sync::Signal;

/// A size which is enough to accommodate the maximum BTP payload plus the GATT header.
const MAX_MTU_SIZE: usize = 512;

/// The maximum size of the BTP advertising payload (AD1 Flags + AD2 UUID16 & Service Data).
const MAX_ADV_DATA_SIZE: usize = 32;

// The Matter BTP service UUIDs, as `const BleUuid`s (NimBLE stores 128-bit UUIDs LSB-first; the
// `BleUuid` constructors handle that).
const SVC_UUID: BleUuid = BleUuid::uuid16(MATTER_BLE_SERVICE_UUID16);
const C1_UUID: BleUuid = BleUuid::uuid128(C1_CHARACTERISTIC_UUID);
const C2_UUID: BleUuid = BleUuid::uuid128(C2_CHARACTERISTIC_UUID);
const C3_UUID: BleUuid = BleUuid::uuid128(C3_CHARACTERISTIC_UUID);

// The Matter BTP GATT service, defined statically at compile time (in flash). `C1` is written by
// the peer, `C2` is indicated to the peer, `C3` is read by the peer.
gatt_services!(SERVICES {
    primary(SVC_UUID) {
        chr(C1_UUID, Write);
        chr(C2_UUID, Indicate);
        chr(C3_UUID, Read);
    }
});

/// The value attribute handle of `C2`, learned from the registration event; indications are
/// addressed to it.
static C2_VAL_HANDLE: AtomicU16 = AtomicU16::new(0);

/// The context of the currently running peripheral.
///
/// The safe event hooks are `'static` closures, so they reach the context through this global
/// pointer rather than by borrowing it. Only one BTP peripheral can run at a time, which
/// `BleDriver`'s singleton (and `EspBtpGattContext::reset`) enforces.
static CONTEXT: AtomicPtr<EspBtpGattContext> = AtomicPtr::new(core::ptr::null_mut());

fn context() -> Option<&'static EspBtpGattContext> {
    // SAFETY: the pointer is published by `run` and unpublished when the peripheral is dropped,
    // and the context outlives the peripheral.
    unsafe { CONTEXT.load(Ordering::SeqCst).as_ref() }
}

/// The state of the connection to the (single) peer we are talking to.
#[derive(Debug, Clone)]
struct Connection {
    peer: BtAddr,
    conn_handle: u16,
    subscribed: bool,
    mtu: Option<u16>,
}

struct State {
    connection: Option<Connection>,
    conn_gen: usize,
    in_data: Vec<u8, MAX_MTU_SIZE>,
    out_data: Vec<u8, MAX_MTU_SIZE>,
    /// Set by the host-sync / disconnect hooks to request (re)advertising; consumed by the outgoing
    /// pump (advertising is synchronous, so it needs no future of its own).
    need_advertise: bool,
    /// The raw advertising payload, kept so that advertising can be restarted on disconnect
    adv_data: Vec<u8, MAX_ADV_DATA_SIZE>,
}

impl State {
    #[inline(always)]
    const fn new() -> Self {
        Self {
            connection: None,
            conn_gen: 0,
            in_data: Vec::new(),
            out_data: Vec::new(),
            need_advertise: false,
            adv_data: Vec::new(),
        }
    }

    fn init() -> impl Init<Self> {
        init!(Self {
            connection: None,
            conn_gen: 0,
            in_data <- Vec::init(),
            out_data <- Vec::init(),
            need_advertise: false,
            adv_data <- Vec::init(),
        })
    }
}

/// The `'static` state of the `EspBtpGattPeripheral` struct.
/// Isolated as a separate struct to allow for `const fn` construction
/// and static allocation.
pub struct EspBtpGattContext {
    state: Mutex<RefCell<State>, CriticalSectionRawMutex>,
    /// Whether the current outgoing indication is still awaiting the peer's acknowledgment. Kept
    /// *outside* `state` (and its `RefCell`) on purpose: NimBLE can deliver `NotifyComplete`
    /// synchronously from within `driver.indicate()`, which runs while `process_outgoing` holds
    /// `state.borrow_mut()`. An atomic flag lets that re-entrant hook clear it without re-borrowing
    /// `state` (which would panic with `BorrowMutError`).
    out_nack: AtomicBool,
    /// A signal used to awake the `process_incoming()` loop as there might be incoming data (c1 writes) to process.
    notify_process_incoming: Signal<Option<()>, CriticalSectionRawMutex>,
    /// A signal used to awake the `process_outgoing()` loop as there might be outgoing data (c2
    /// indications) to process, or (re)advertising to (re)start.
    notify_process_outgoing: Signal<Option<()>, CriticalSectionRawMutex>,
}

impl EspBtpGattContext {
    /// Create a new instance.
    #[allow(clippy::large_stack_frames)]
    #[inline(always)]
    pub const fn new() -> Self {
        Self {
            state: Mutex::new(RefCell::new(State::new())),
            out_nack: AtomicBool::new(false),
            notify_process_incoming: Signal::new(None),
            notify_process_outgoing: Signal::new(None),
        }
    }

    /// Return an in-place initializer for `EspBtpGattContext`.
    #[allow(clippy::large_stack_frames)]
    pub fn init() -> impl Init<Self> {
        init!(Self {
            state <- Mutex::init(RefCell::init(State::init())),
            out_nack: AtomicBool::new(false),
            notify_process_incoming <- Signal::init(None),
            notify_process_outgoing <- Signal::init(None),
        })
    }

    pub(crate) fn reset(&self) -> Result<(), EspError> {
        self.state.lock(|state| {
            let mut state = state.borrow_mut();

            state.connection = None;
            state.in_data.clear();
            state.out_data.clear();
            state.adv_data.clear();
        });

        self.out_nack.store(false, Ordering::SeqCst);

        self.notify_process_incoming.modify(|state| {
            *state = None;
            (false, ())
        });

        self.notify_process_outgoing.modify(|state| {
            *state = None;
            (false, ())
        });

        Ok(())
    }
}

impl Default for EspBtpGattContext {
    // TODO
    #[allow(clippy::large_stack_frames)]
    fn default() -> Self {
        Self::new()
    }
}

/// The GATT service table `S` type. It is erased behind a `dyn` reference so that
/// `EspBtpGattPeripheral` need not carry the static table's const-generic length in its type.
type ServiceTable = &'static (dyn AsRef<[ble_gatt_svc_def]> + Sync);

/// Implements the `GattPeripheral` trait on top of the NimBLE host.
pub struct EspBtpGattPeripheral<'a, 'd> {
    driver: BleDriver<'d, ServiceTable>,
    context: &'a EspBtpGattContext,
}

impl<'a, 'd> EspBtpGattPeripheral<'a, 'd> {
    /// Create a new instance.
    ///
    /// Creation might fail if the GATT context cannot be reset, so user should ensure
    /// that there are no other GATT peripherals running before calling this function.
    pub fn new<B: BluetoothModemPeripheral + 'd>(
        modem: B,
        context: &'a EspBtpGattContext,
    ) -> Result<Self, EspError> {
        context.reset()?;

        // Initialize the host and register the static Matter BTP service table (in flash) in one
        // step. The host task is not started yet - that happens in `run`, after the hooks are set.
        let driver = BleDriver::new_with_services(modem, &SERVICES as ServiceTable)?;

        Ok(Self { driver, context })
    }

    /// Run the GATT peripheral.
    pub async fn run(
        &mut self,
        btp: &Btp,
        service_name: &str,
        service_adv_data: &AdvData,
    ) -> Result<(), Error> {
        self.context.state.lock(|state| {
            let mut state = state.borrow_mut();

            state.adv_data.clear();
            for byte in service_adv_data.iter() {
                state
                    .adv_data
                    .push(byte)
                    .map_err(|_| Error::new(ErrorCode::NoSpace))?;
            }

            Ok::<_, Error>(())
        })?;

        // Publish the context so that the `'static` event hooks can reach it.
        CONTEXT.store(
            self.context as *const _ as *mut EspBtpGattContext,
            Ordering::SeqCst,
        );

        self.subscribe_hooks();

        self.driver
            .set_device_name(service_name)
            .map_err(to_matter_err_ble)?;

        info!("BTP service registered, device name set to `{service_name}`");

        self.driver.start().map_err(to_matter_err)?;

        info!("NimBLE host task started");

        // Two concurrent pumps: incoming C1 writes, and outgoing C2 indications - which also drives
        // (re)advertising synchronously, so it needs no future of its own. This keeps the `run`
        // future (which rs-matter-stack bump-allocates) small.
        select(self.process_incoming(btp), self.process_outgoing(btp))
            .coalesce()
            .await
    }

    /// Wire the host / GAP / GATT-server event hooks. They are `'static` (they capture nothing and
    /// reach the peripheral state through the global `CONTEXT`), so no `unsafe` borrowing is needed.
    fn subscribe_hooks(&self) {
        // Host lifecycle: advertise once in sync, and again after every stack reset.
        self.driver.host_subscribe(|event| {
            if matches!(event, HostEvent::Sync) {
                if let Some(context) = context() {
                    context.state.lock(|s| s.borrow_mut().need_advertise = true);
                    context.notify_process_outgoing.signal(());
                }
            }
        });

        // GAP: connection lifecycle. (`Subscribe` / `NotifyComplete` are demuxed to the GATTS hook.)
        self.driver.gap_subscribe(|event| {
            let Some(context) = context() else {
                return 0;
            };

            match event {
                GapEvent::Connect {
                    conn_handle,
                    status,
                } => {
                    if status.is_err() {
                        warn!("BLE connection failed: {status:?}");
                        // The attempt failed, so we have to advertise again.
                        context.state.lock(|s| s.borrow_mut().need_advertise = true);
                        context.notify_process_outgoing.signal(());
                        return 0;
                    }

                    let peer = conn_find(conn_handle)
                        .map(|desc| BtAddr(desc.peer_addr().val()))
                        .unwrap_or(BtAddr([0; 6]));
                    let mtu = att_mtu(conn_handle).ok();

                    context.state.lock(|state| {
                        let mut state = state.borrow_mut();

                        state.conn_gen = state.conn_gen.wrapping_add(1);
                        state.in_data.clear();
                        state.out_data.clear();
                        state.connection = Some(Connection {
                            peer,
                            conn_handle,
                            subscribed: false,
                            mtu,
                        });
                    });

                    context.out_nack.store(false, Ordering::SeqCst);

                    info!("BLE connected, handle: {conn_handle}");

                    context.notify_process_incoming.signal(());
                }
                GapEvent::Disconnect { reason, .. } => {
                    context.state.lock(|state| {
                        let mut state = state.borrow_mut();

                        state.connection = None;
                        state.in_data.clear();
                        state.out_data.clear();
                        // Nobody is connected anymore, so become discoverable again.
                        state.need_advertise = true;
                    });

                    context.out_nack.store(false, Ordering::SeqCst);

                    info!("BLE disconnected, reason: {reason}");

                    context.notify_process_incoming.signal(());
                    context.notify_process_outgoing.signal(());
                }
                GapEvent::Mtu { conn_handle, value } => {
                    context.state.lock(|state| {
                        let mut state = state.borrow_mut();

                        if let Some(conn) = state.connection.as_mut() {
                            if conn.conn_handle == conn_handle {
                                conn.mtu = Some(value);
                            }
                        }
                    });

                    trace!("MTU negotiated: {value}");
                }
                _ => {}
            }

            0
        });

        // GATT server: learn the `C2` handle, ingest `C1` writes, answer `C3` reads (empty), and
        // track `C2` subscriptions and indication completions.
        self.driver.gatts_subscribe(|event| {
            let Some(context) = context() else {
                return 0;
            };

            match event {
                GattsEvent::Register(reg) => {
                    if let BleGattRegister::Characteristic {
                        uuid, val_handle, ..
                    } = reg
                    {
                        if uuid == C2_UUID {
                            C2_VAL_HANDLE.store(val_handle, Ordering::SeqCst);
                        }
                    }
                }
                // A `C1` write carries an incoming BTP packet.
                GattsEvent::Write { data, .. } => {
                    let result = context.state.lock(|state| {
                        let mut state = state.borrow_mut();

                        if !state.in_data.is_empty() {
                            // The previous write has not been processed yet.
                            return Err(());
                        }

                        state.in_data.resize_default(MAX_MTU_SIZE).map_err(|_| ())?;

                        let len = data.read(&mut state.in_data).map_err(|_| ())?;
                        state.in_data.truncate(len);

                        Ok(())
                    });

                    if result.is_err() {
                        return BLE_ATT_ERR_INSUFFICIENT_RES as u8;
                    }

                    context.notify_process_incoming.signal(());
                }
                // A `C3` read: answered with an empty payload (no additional commissioning data).
                GattsEvent::Read { .. } => {}
                // A `C2` subscription change: gate outgoing indications on it. Teardown and bond
                // restore report the resulting state too, so `cur_indicate` is all we need.
                GattsEvent::SubscriptionChanged {
                    conn_handle,
                    attr_handle,
                    cur_indicate,
                    ..
                } => {
                    if attr_handle == C2_VAL_HANDLE.load(Ordering::SeqCst) {
                        context.state.lock(|state| {
                            let mut state = state.borrow_mut();

                            if let Some(conn) = state.connection.as_mut() {
                                if conn.conn_handle == conn_handle {
                                    conn.subscribed = cur_indicate;
                                }
                            }
                        });

                        info!(
                            "Peer {} to `C2`",
                            if cur_indicate {
                                "subscribed"
                            } else {
                                "unsubscribed"
                            }
                        );

                        context.notify_process_outgoing.signal(());
                    }
                }
                // An indication completed (peer confirmed, or it failed); the next one may go out.
                GattsEvent::NotifyComplete {
                    indication, status, ..
                } => {
                    if indication && (status == BLE_HS_EDONE as i32 || status != 0) {
                        // `out_nack` is an atomic *outside* `state`, so this can run re-entrantly from
                        // inside `driver.indicate()` (which is called while `process_outgoing` holds
                        // `state.borrow_mut()`) without re-borrowing `state`.
                        context.out_nack.store(false, Ordering::SeqCst);

                        context.notify_process_outgoing.signal(());
                    }
                }
            }

            0
        });
    }

    /// Process incoming writes on characteristic `C1` from a remote peer.
    ///
    /// While it might seem that this can be done directly from the GATTS hook, this is not
    /// generally possible because `Btp` might not be `Sync`, while the hook has to be.
    async fn process_incoming(&self, btp: &Btp) -> Result<(), Error> {
        let mut generation = None;

        loop {
            let processed = self.context.state.lock(|state| {
                let mut state = state.borrow_mut();

                // Copy the connection out so the `in_data` borrows below are unambiguous.
                let conn = state.connection.as_ref().map(|c| (c.mtu, c.peer));

                if let Some((mtu, peer)) = conn {
                    if generation != Some(state.conn_gen) {
                        btp.reset();
                        generation = Some(state.conn_gen);
                    }

                    if !state.in_data.is_empty() {
                        btp.process_incoming(mtu, peer, &state.in_data)?;

                        // Unlike Bluedroid, NimBLE has already acknowledged the write.
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

    /// Indicate new data on characteristic `C2` to a remote peer, and drive (re)advertising.
    async fn process_outgoing(&self, btp: &Btp) -> Result<(), Error> {
        loop {
            // (Re)advertise if the host-sync / disconnect hooks requested it. Synchronous, so it
            // adds no state to this future.
            if self
                .context
                .state
                .lock(|s| core::mem::take(&mut s.borrow_mut().need_advertise))
            {
                if let Err(e) = self.advertise() {
                    error!("Cannot start advertising: {e:?}");
                }
            }

            let processed = self.context.state.lock(|state| {
                let mut state = state.borrow_mut();
                let state = &mut *state;

                let (mtu, conn_handle) = match state.connection.as_ref() {
                    // Peer is connected and subscribed to indications.
                    Some(conn) if conn.subscribed => (conn.mtu, conn.conn_handle),
                    _ => return Ok::<_, Error>(false),
                };

                if self.context.out_nack.load(Ordering::SeqCst) {
                    // The previous indication has not been acknowledged by the peer yet.
                    return Ok(false);
                }

                let c2_handle = C2_VAL_HANDLE.load(Ordering::SeqCst);
                if c2_handle == 0 {
                    return Ok(false);
                }

                state.out_data.resize_default(MAX_MTU_SIZE).unwrap();

                let len = btp.process_outgoing(mtu, &mut state.out_data)?;
                if len == 0 {
                    return Ok(false);
                }

                // Mark unacked *before* the send: NimBLE may deliver `NotifyComplete` synchronously
                // from within `indicate`, and that hook clears `out_nack` - which must win, so it has
                // to be set first. Because `out_nack` is atomic (not in `state`), that re-entrant hook
                // does not touch the `state` borrow we are still holding here, so `indicate` can be
                // called in place - no copy of `out_data` out of the borrow, no bigger future.
                self.context.out_nack.store(true, Ordering::SeqCst);

                match self
                    .driver
                    .indicate(conn_handle, c2_handle, &state.out_data[..len])
                {
                    Ok(()) => {
                        trace!("Indicated {len} bytes");
                        Ok(true)
                    }
                    Err(e) => {
                        // The send failed: clear the flag so a later attempt is not blocked forever
                        // by a phantom in-flight indication.
                        self.context.out_nack.store(false, Ordering::SeqCst);
                        Err(to_matter_err_ble(e))
                    }
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

    /// Configure and (re)start connectable, undirected advertising. Synchronous - deliberately not a
    /// separate async task, so the bump-allocated `run` future stays small.
    fn advertise(&self) -> Result<(), Error> {
        ensure_addr(false).map_err(to_matter_err_ble)?;

        let adv_data = self
            .context
            .state
            .lock(|state| state.borrow().adv_data.clone());

        self.driver
            .adv_set_data(&adv_data)
            .map_err(to_matter_err_ble)?;

        let params = BleAdvParams {
            conn_mode: BLE_GAP_CONN_MODE_UND as u8,
            disc_mode: BLE_GAP_DISC_MODE_GEN as u8,
            ..Default::default()
        };

        self.driver
            .adv_start(BLE_OWN_ADDR_PUBLIC as u8, &params)
            .map_err(to_matter_err_ble)?;

        info!("Advertising started");

        Ok(())
    }
}

impl Drop for EspBtpGattPeripheral<'_, '_> {
    fn drop(&mut self) {
        // Unpublish first; any hook that races in afterwards sees `None` and no-ops. The `driver`
        // field is dropped right after, stopping the host and unsubscribing the hooks.
        CONTEXT.store(core::ptr::null_mut(), Ordering::SeqCst);
    }
}

impl GattPeripheral for EspBtpGattPeripheral<'_, '_> {
    async fn run(
        &mut self,
        btp: &Btp,
        service_name: &str,
        adv_data: &AdvData,
    ) -> Result<(), Error> {
        EspBtpGattPeripheral::run(self, btp, service_name, adv_data).await
    }
}

fn to_matter_err(e: EspError) -> Error {
    error!("BLE error: {e:?}");
    Error::new(ErrorCode::NoNetworkInterface)
}

fn to_matter_err_ble(e: BleError) -> Error {
    error!("BLE error: {e:?}");
    Error::new(ErrorCode::NoNetworkInterface)
}

#[cfg(esp_idf_bt_nimble_gatt_client)]
pub use central::{EspBtpGattClient, EspBtpGattClientContext};

/// A GATT **client** (central / Commissioner-side) BTP transport on top of the type-safe NimBLE
/// GATT-client API.
///
/// This is the analogue of the peripheral above for the Matter *commissioner* role: given the BLE
/// address of a commissionable device (as an mDNS/BLE browse would yield), it connects as a GATT
/// central, discovers the Matter BTP service and its `C1`/`C2` characteristics, subscribes to `C2`
/// indications, and drives the `Btp` engine as the **initiator** — writing outgoing BTP segments to
/// `C1` as acknowledged Write Requests and feeding `C2` indications back in. It models rs-matter's
/// `bluer::run_central`; scanning/browsing for the address is a separate concern not handled here.
///
/// Requires a NimBLE build with the GATT client enabled
/// (`CONFIG_BT_NIMBLE_ROLE_CENTRAL=y` + `CONFIG_BT_NIMBLE_GATT_CLIENT=y`), so the whole module is
/// `#[cfg]`-gated on it.
#[cfg(esp_idf_bt_nimble_gatt_client)]
mod central {
    use core::sync::atomic::{AtomicPtr, Ordering};

    use embassy_futures::select::select;
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

    use esp_idf_svc::ble::gap::GapEvent;
    use esp_idf_svc::ble::gatt::client::GattcEvent;
    use esp_idf_svc::ble::{ensure_addr, BleAddr, BleDriver, HostEvent};
    use esp_idf_svc::hal::modem::BluetoothModemPeripheral;
    use esp_idf_svc::sys::{EspError, BLE_OWN_ADDR_PUBLIC};

    use ::log::{error, info, trace};

    use rs_matter_stack::matter::error::{Error, ErrorCode};
    use rs_matter_stack::matter::transport::network::btp::Btp;
    use rs_matter_stack::matter::transport::network::BtAddr;
    use rs_matter_stack::matter::utils::cell::RefCell;
    use rs_matter_stack::matter::utils::init::{init, Init};
    use rs_matter_stack::matter::utils::select::Coalesce;
    use rs_matter_stack::matter::utils::storage::Vec;
    use rs_matter_stack::matter::utils::sync::blocking::Mutex;
    use rs_matter_stack::matter::utils::sync::Signal;

    use super::{to_matter_err, to_matter_err_ble, C1_UUID, C2_UUID, MAX_MTU_SIZE, SVC_UUID};

    /// The peer address type to connect with. Commissionable ESP devices advertise with a public
    /// address; a full commissioner would take this from the browse/scan result.
    const PEER_ADDR_TYPE: u8 = 0; // BLE_ADDR_PUBLIC

    /// The context reached from the `'static` event hooks (the central counterpart of the
    /// peripheral's `CONTEXT`).
    static CLIENT_CONTEXT: AtomicPtr<EspBtpGattClientContext> =
        AtomicPtr::new(core::ptr::null_mut());

    fn client_context() -> Option<&'static EspBtpGattClientContext> {
        // SAFETY: published by `run`, unpublished on drop; the context outlives the client.
        unsafe { CLIENT_CONTEXT.load(Ordering::SeqCst).as_ref() }
    }

    /// The central-side session state, driven by the GAP / GATT-client event hooks.
    struct State {
        /// Set once the host is in sync with the controller.
        synced: bool,
        /// The connection handle, once connected.
        conn_handle: Option<u16>,
        /// Set if the connection attempt failed (or the link dropped).
        failed: bool,
        mtu: Option<u16>,
        peer: BtAddr,
        /// The Matter service's attribute-handle range, learned from service discovery.
        matter_range: Option<(u16, u16)>,
        services_done: bool,
        /// The value handles of `C1` (write) and `C2` (indicate), learned from char discovery.
        c1_val: Option<u16>,
        c2_val: Option<u16>,
        chars_done: bool,
        /// Set once the `C2` CCCD write (indication subscribe) completes.
        subscribed: bool,
        /// An incoming `C2` indication, awaiting delivery to the BTP engine.
        in_data: Vec<u8, MAX_MTU_SIZE>,
        out_data: Vec<u8, MAX_MTU_SIZE>,
        /// Whether a `C1` Write Request is in flight (awaiting its response = BTP flow control).
        out_inflight: bool,
    }

    impl State {
        const fn new() -> Self {
            Self {
                synced: false,
                conn_handle: None,
                failed: false,
                mtu: None,
                peer: BtAddr([0; 6]),
                matter_range: None,
                services_done: false,
                c1_val: None,
                c2_val: None,
                chars_done: false,
                subscribed: false,
                in_data: Vec::new(),
                out_data: Vec::new(),
                out_inflight: false,
            }
        }

        fn init() -> impl Init<Self> {
            init!(Self {
                synced: false,
                conn_handle: None,
                failed: false,
                mtu: None,
                peer: BtAddr([0; 6]),
                matter_range: None,
                services_done: false,
                c1_val: None,
                c2_val: None,
                chars_done: false,
                subscribed: false,
                in_data <- Vec::init(),
                out_data <- Vec::init(),
                out_inflight: false,
            })
        }
    }

    /// The `'static` backing state of [`EspBtpGattClient`], split out for `const`/static allocation.
    pub struct EspBtpGattClientContext {
        state: Mutex<RefCell<State>, CriticalSectionRawMutex>,
        /// Fired on any discovery/connection progress; `run` waits on it between the setup steps.
        notify_progress: Signal<Option<()>, CriticalSectionRawMutex>,
        /// Fired when a `C2` indication arrives (there may be incoming data to pump).
        notify_in: Signal<Option<()>, CriticalSectionRawMutex>,
        /// Fired when a `C1` write completes (there may be outgoing data to pump).
        notify_out: Signal<Option<()>, CriticalSectionRawMutex>,
    }

    impl EspBtpGattClientContext {
        /// Create a new instance.
        #[inline(always)]
        pub const fn new() -> Self {
            Self {
                state: Mutex::new(RefCell::new(State::new())),
                notify_progress: Signal::new(None),
                notify_in: Signal::new(None),
                notify_out: Signal::new(None),
            }
        }

        /// Return an in-place initializer.
        pub fn init() -> impl Init<Self> {
            init!(Self {
                state <- Mutex::init(RefCell::init(State::init())),
                notify_progress <- Signal::init(None),
                notify_in <- Signal::init(None),
                notify_out <- Signal::init(None),
            })
        }

        pub(crate) fn reset(&self) -> Result<(), EspError> {
            self.state.lock(|state| *state.borrow_mut() = State::new());
            for signal in [&self.notify_progress, &self.notify_in, &self.notify_out] {
                signal.modify(|state| {
                    *state = None;
                    (false, ())
                });
            }
            Ok(())
        }
    }

    impl Default for EspBtpGattClientContext {
        fn default() -> Self {
            Self::new()
        }
    }

    /// A GATT client (Matter Commissioner-side BTP transport) on top of the NimBLE host.
    pub struct EspBtpGattClient<'a, 'd> {
        driver: BleDriver<'d, ()>,
        context: &'a EspBtpGattClientContext,
    }

    impl<'a, 'd> EspBtpGattClient<'a, 'd> {
        /// Create a new instance. A central has no GATT server, so the driver carries no service
        /// table.
        pub fn new<B: BluetoothModemPeripheral + 'd>(
            modem: B,
            context: &'a EspBtpGattClientContext,
        ) -> Result<Self, EspError> {
            context.reset()?;

            let driver = BleDriver::new(modem)?;

            Ok(Self { driver, context })
        }

        /// Connect to the commissionable device at `addr` and run the BTP transport against it as
        /// the GATT central / BTP initiator until the session ends.
        ///
        /// As with rs-matter's `run_central`, this deliberately does **not** `btp.reset()` — the
        /// caller resets and queues the first PASE SDU before driving Matter traffic.
        pub async fn run(&mut self, btp: &Btp, addr: BtAddr) -> Result<(), Error> {
            self.context
                .state
                .lock(|state| state.borrow_mut().peer = addr);

            CLIENT_CONTEXT.store(
                self.context as *const _ as *mut EspBtpGattClientContext,
                Ordering::SeqCst,
            );

            self.subscribe_hooks();

            self.driver.start().map_err(to_matter_err)?;

            // 1) Wait for the host to synchronize, then connect.
            self.wait_state(|s| s.synced).await;
            ensure_addr(false).map_err(to_matter_err_ble)?;

            let peer = BleAddr::new(PEER_ADDR_TYPE, addr.0);
            self.driver
                .connect(BLE_OWN_ADDR_PUBLIC as u8, &peer)
                .map_err(to_matter_err_ble)?;
            info!("Connecting to commissionable device {addr}");

            self.wait_state(|s| s.conn_handle.is_some() || s.failed)
                .await;
            let conn = self
                .connected_handle()
                .ok_or_else(|| Error::new(ErrorCode::NoNetworkInterface))?;

            // 2) Discover the Matter service and its C1/C2 characteristics.
            self.driver
                .discover_services(conn)
                .map_err(to_matter_err_ble)?;
            self.wait_state(|s| s.services_done || s.failed).await;

            let (start, end) = self
                .context
                .state
                .lock(|s| s.borrow().matter_range)
                .ok_or_else(|| {
                    error!("Matter BTP service not found on peer");
                    Error::new(ErrorCode::NoNetworkInterface)
                })?;

            self.driver
                .discover_characteristics(conn, start, end)
                .map_err(to_matter_err_ble)?;
            self.wait_state(|s| s.chars_done || s.failed).await;

            let (c1_val, c2_val) = self.context.state.lock(|s| {
                let s = s.borrow();
                (s.c1_val, s.c2_val)
            });
            let (c1_val, c2_val) = match (c1_val, c2_val) {
                (Some(c1), Some(c2)) => (c1, c2),
                _ => {
                    error!("Matter C1/C2 characteristics not found on peer");
                    return Err(Error::new(ErrorCode::NoNetworkInterface));
                }
            };

            info!("Discovered Matter C1 (handle {c1_val}) / C2 (handle {c2_val})");

            // 3) Subscribe to C2 indications by writing its CCCD (value-handle + 1). NimBLE lays the
            //    CCCD out directly after the value attribute.
            self.driver
                .write(conn, c2_val + 1, &[0x02, 0x00])
                .map_err(to_matter_err_ble)?;
            self.wait_state(|s| s.subscribed || s.failed).await;

            if self.context.state.lock(|s| s.borrow().failed) {
                return Err(Error::new(ErrorCode::NoNetworkInterface));
            }

            info!("Subscribed to C2; driving BTP as initiator");

            // 4) We are the initiator: pump C2 indications -> BTP and BTP -> C1 writes.
            btp.set_initiator(true);

            select(
                self.process_incoming(btp),
                self.process_outgoing(btp, conn, c1_val),
            )
            .coalesce()
            .await
        }

        /// Wire the host / GAP / GATT-client hooks. `'static`; they reach the state through
        /// `CLIENT_CONTEXT` and only record results + signal progress.
        fn subscribe_hooks(&self) {
            self.driver.host_subscribe(|event| {
                if matches!(event, HostEvent::Sync) {
                    if let Some(context) = client_context() {
                        context.state.lock(|s| s.borrow_mut().synced = true);
                        context.notify_progress.signal(());
                    }
                }
            });

            self.driver.gap_subscribe(|event| {
                let Some(context) = client_context() else {
                    return 0;
                };

                match event {
                    GapEvent::Connect {
                        conn_handle,
                        status,
                    } => {
                        context.state.lock(|s| {
                            let mut s = s.borrow_mut();
                            if status.is_ok() {
                                s.conn_handle = Some(conn_handle);
                            } else {
                                s.failed = true;
                            }
                        });
                        context.notify_progress.signal(());
                    }
                    GapEvent::Disconnect { .. } => {
                        context.state.lock(|s| s.borrow_mut().failed = true);
                        context.notify_progress.signal(());
                        context.notify_in.signal(());
                        context.notify_out.signal(());
                    }
                    GapEvent::Mtu { conn_handle, value } => {
                        context.state.lock(|s| {
                            let mut s = s.borrow_mut();
                            if s.conn_handle == Some(conn_handle) {
                                s.mtu = Some(value);
                            }
                        });
                    }
                    _ => {}
                }

                0
            });

            self.driver.gattc_subscribe(|event| {
                let Some(context) = client_context() else {
                    return;
                };

                match event {
                    // Service discovery: collect the Matter service range, signal on completion.
                    GattcEvent::Service { service, .. } => match service {
                        Some(service) => {
                            if service.uuid == SVC_UUID {
                                context.state.lock(|s| {
                                    s.borrow_mut().matter_range =
                                        Some((service.start_handle, service.end_handle));
                                });
                            }
                        }
                        None => {
                            context.state.lock(|s| s.borrow_mut().services_done = true);
                            context.notify_progress.signal(());
                        }
                    },
                    // Characteristic discovery: capture C1/C2 value handles, signal on completion.
                    GattcEvent::Characteristic { chr, .. } => match chr {
                        Some(chr) => context.state.lock(|s| {
                            let mut s = s.borrow_mut();
                            if chr.uuid == C1_UUID {
                                s.c1_val = Some(chr.val_handle);
                            } else if chr.uuid == C2_UUID {
                                s.c2_val = Some(chr.val_handle);
                            }
                        }),
                        None => {
                            context.state.lock(|s| s.borrow_mut().chars_done = true);
                            context.notify_progress.signal(());
                        }
                    },
                    // A write completed: either the CCCD subscribe, or a C1 segment (flow control).
                    GattcEvent::WriteComplete {
                        attr_handle,
                        status,
                        ..
                    } => {
                        let (c2_val, c1_val) = context
                            .state
                            .lock(|s| (s.borrow().c2_val, s.borrow().c1_val));
                        if Some(attr_handle) == c2_val.map(|h| h + 1) {
                            context
                                .state
                                .lock(|s| s.borrow_mut().subscribed = status == 0);
                            context.notify_progress.signal(());
                        } else if Some(attr_handle) == c1_val {
                            context.state.lock(|s| s.borrow_mut().out_inflight = false);
                            context.notify_out.signal(());
                        }
                    }
                    // A C2 indication: stash it for the incoming pump (empty payloads are not BTP).
                    GattcEvent::Notify {
                        attr_handle, data, ..
                    } => {
                        let c2_val = context.state.lock(|s| s.borrow().c2_val);
                        if Some(attr_handle) == c2_val {
                            context.state.lock(|s| {
                                let mut s = s.borrow_mut();
                                if s.in_data.is_empty()
                                    && s.in_data.resize_default(MAX_MTU_SIZE).is_ok()
                                {
                                    if let Ok(len) = data.read(&mut s.in_data) {
                                        s.in_data.truncate(len);
                                    } else {
                                        s.in_data.clear();
                                    }
                                }
                            });
                            context.notify_in.signal(());
                        }
                    }
                    GattcEvent::ReadComplete { .. } => {}
                }
            });
        }

        fn connected_handle(&self) -> Option<u16> {
            self.context.state.lock(|s| s.borrow().conn_handle)
        }

        /// Await until `pred` holds over the session state, waking on any progress signal.
        async fn wait_state(&self, pred: impl Fn(&State) -> bool) {
            loop {
                if self.context.state.lock(|s| pred(&s.borrow())) {
                    return;
                }
                self.context.notify_progress.wait_signalled().await;
            }
        }

        /// Feed received `C2` indications into the BTP engine.
        async fn process_incoming(&self, btp: &Btp) -> Result<(), Error> {
            loop {
                let processed = self.context.state.lock(|state| {
                    let mut state = state.borrow_mut();

                    if state.failed {
                        return Err(Error::new(ErrorCode::NoNetworkInterface));
                    }

                    if state.in_data.is_empty() {
                        return Ok(false);
                    }

                    let mtu = state.mtu;
                    let peer = state.peer;

                    // An empty payload is never a valid BTP frame; non-empty only reaches here.
                    btp.process_incoming(mtu, peer, &state.in_data)?;
                    state.in_data.clear();

                    Ok::<_, Error>(true)
                })?;

                if !processed {
                    self.context.notify_in.wait_signalled().await;
                }
            }
        }

        /// Drive BTP output as acknowledged `C1` Write Requests (one segment in flight at a time,
        /// which is the client-to-server half of BTP flow control).
        async fn process_outgoing(&self, btp: &Btp, conn: u16, c1_val: u16) -> Result<(), Error> {
            loop {
                let processed = self.context.state.lock(|state| {
                    let mut state = state.borrow_mut();

                    if state.failed {
                        return Err(Error::new(ErrorCode::NoNetworkInterface));
                    }

                    if state.out_inflight {
                        // Awaiting the previous Write Response.
                        return Ok(false);
                    }

                    let mtu = state.mtu;
                    state.out_data.resize_default(MAX_MTU_SIZE).unwrap();

                    let len = btp.process_outgoing(mtu, &mut state.out_data)?;
                    if len > 0 {
                        self.driver
                            .write(conn, c1_val, &state.out_data[..len])
                            .map_err(to_matter_err_ble)?;
                        state.out_inflight = true;

                        trace!("Wrote {len} bytes to C1");

                        Ok(true)
                    } else {
                        Ok::<_, Error>(false)
                    }
                })?;

                if !processed {
                    select(
                        btp.wait_outgoing(),
                        self.context.notify_out.wait_signalled(),
                    )
                    .coalesce()
                    .await;
                }
            }
        }
    }

    impl Drop for EspBtpGattClient<'_, '_> {
        fn drop(&mut self) {
            CLIENT_CONTEXT.store(core::ptr::null_mut(), Ordering::SeqCst);
        }
    }
}
