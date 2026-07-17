//! A `GattPeripheral` implementation for the Matter BTP protocol, on top of the NimBLE host.
//!
//! Compared to the Bluedroid backend, NimBLE initializes the controller and the host with a
//! single `nimble_port_init` call, so there is no `BtDriver` equivalent here. Writes are also
//! acknowledged by NimBLE itself as soon as the access callback returns, so - unlike Bluedroid -
//! no GATT response has to be sent by hand.

use core::cell::UnsafeCell;
use core::ffi::{c_int, c_void};
use core::ptr;
use core::sync::atomic::{AtomicPtr, Ordering};

use alloc::ffi::CString;

use embassy_futures::select::select;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use esp_idf_svc::hal::modem::BluetoothModemPeripheral;
use esp_idf_svc::sys::*;

// `esp_idf_svc::sys` is glob-imported below and also exports a `log` item, so the crate has
// to be named unambiguously here.
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

/// A wrapper granting `Sync` to a `T` which is only ever touched from the NimBLE host task,
/// or from the `run` method before that task is started.
#[repr(transparent)]
struct SyncCell<T>(UnsafeCell<T>);

// SAFETY: the GATT tables below are written once in `run`, before `nimble_port_freertos_init`
// spawns the host task, and are only read by NimBLE afterwards.
unsafe impl<T> Sync for SyncCell<T> {}

impl<T> SyncCell<T> {
    const fn new(value: T) -> Self {
        Self(UnsafeCell::new(value))
    }

    #[allow(clippy::mut_from_ref)]
    unsafe fn get(&self) -> *mut T {
        self.0.get()
    }
}

/// The Matter BTP service UUID, as a NimBLE 16-bit UUID.
static SVC_UUID: ble_uuid16_t = ble_uuid16_t {
    u: ble_uuid_t {
        type_: BLE_UUID_TYPE_16 as _,
    },
    value: MATTER_BLE_SERVICE_UUID16,
};

/// `C1`, `C2` and `C3`, as NimBLE 128-bit UUIDs. NimBLE stores 128-bit UUIDs
/// least-significant byte first.
static C1_UUID: ble_uuid128_t = ble_uuid128_t {
    u: ble_uuid_t {
        type_: BLE_UUID_TYPE_128 as _,
    },
    value: C1_CHARACTERISTIC_UUID.to_le_bytes(),
};

static C2_UUID: ble_uuid128_t = ble_uuid128_t {
    u: ble_uuid_t {
        type_: BLE_UUID_TYPE_128 as _,
    },
    value: C2_CHARACTERISTIC_UUID.to_le_bytes(),
};

static C3_UUID: ble_uuid128_t = ble_uuid128_t {
    u: ble_uuid_t {
        type_: BLE_UUID_TYPE_128 as _,
    },
    value: C3_CHARACTERISTIC_UUID.to_le_bytes(),
};

/// Filled in by NimBLE at service registration time with the value attribute handle of `C2`,
/// which is what indications are addressed to.
static C2_VAL_HANDLE: SyncCell<u16> = SyncCell::new(0);

/// The characteristics of the Matter BTP service, terminated by an all-zero entry.
static CHRS: SyncCell<[ble_gatt_chr_def; 4]> = SyncCell::new([
    // `C1`: the peer writes BTP packets here.
    ble_gatt_chr_def {
        uuid: ptr::null(),
        access_cb: Some(access_cb),
        arg: ptr::null_mut(),
        descriptors: ptr::null_mut(),
        flags: BLE_GATT_CHR_F_WRITE as _,
        min_key_size: 0,
        val_handle: ptr::null_mut(),
        cpfd: ptr::null_mut(),
    },
    // `C2`: BTP packets are indicated to the peer from here. NimBLE adds the CCCD itself.
    ble_gatt_chr_def {
        uuid: ptr::null(),
        access_cb: Some(access_cb),
        arg: ptr::null_mut(),
        descriptors: ptr::null_mut(),
        flags: BLE_GATT_CHR_F_INDICATE as _,
        min_key_size: 0,
        val_handle: ptr::null_mut(),
        cpfd: ptr::null_mut(),
    },
    // `C3`: additional commissioning-related data, read by the peer.
    ble_gatt_chr_def {
        uuid: ptr::null(),
        access_cb: Some(access_cb),
        arg: ptr::null_mut(),
        descriptors: ptr::null_mut(),
        flags: BLE_GATT_CHR_F_READ as _,
        min_key_size: 0,
        val_handle: ptr::null_mut(),
        cpfd: ptr::null_mut(),
    },
    NULL_CHR,
]);

/// The service table, terminated by an all-zero entry.
static SVCS: SyncCell<[ble_gatt_svc_def; 2]> = SyncCell::new([
    ble_gatt_svc_def {
        type_: BLE_GATT_SVC_TYPE_PRIMARY as _,
        uuid: ptr::null(),
        includes: ptr::null_mut(),
        characteristics: ptr::null(),
    },
    NULL_SVC,
]);

const NULL_CHR: ble_gatt_chr_def = ble_gatt_chr_def {
    uuid: ptr::null(),
    access_cb: None,
    arg: ptr::null_mut(),
    descriptors: ptr::null_mut(),
    flags: 0,
    min_key_size: 0,
    val_handle: ptr::null_mut(),
    cpfd: ptr::null_mut(),
};

const NULL_SVC: ble_gatt_svc_def = ble_gatt_svc_def {
    type_: 0,
    uuid: ptr::null(),
    includes: ptr::null_mut(),
    characteristics: ptr::null(),
};

/// The context of the currently running peripheral.
///
/// NimBLE's GATT tables have to be `'static`, and its callbacks are plain C functions, so the
/// context is reached through this pointer rather than through the `arg` fields. Only one BTP
/// peripheral can run at a time, which `EspBtpGattContext::reset` enforces.
static CONTEXT: AtomicPtr<EspBtpGattContext> = AtomicPtr::new(ptr::null_mut());

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
    /// Whether the current outgoing indication has been acknowledged by the peer or not
    out_nack: bool,
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
            out_nack: false,
            adv_data: Vec::new(),
        }
    }

    fn init() -> impl Init<Self> {
        init!(Self {
            connection: None,
            conn_gen: 0,
            in_data <- Vec::init(),
            out_data <- Vec::init(),
            out_nack: false,
            adv_data <- Vec::init(),
        })
    }
}

/// The `'static` state of the `EspBtpGattPeripheral` struct.
/// Isolated as a separate struct to allow for `const fn` construction
/// and static allocation.
pub struct EspBtpGattContext {
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

            state.connection = None;
            state.in_data.clear();
            state.out_data.clear();
            state.out_nack = false;
            state.adv_data.clear();
        });

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

/// Implements the `GattPeripheral` trait on top of the NimBLE host.
pub struct EspBtpGattPeripheral<'a, 'd> {
    _modem: core::marker::PhantomData<&'d mut ()>,
    context: &'a EspBtpGattContext,
}

impl<'a, 'd> EspBtpGattPeripheral<'a, 'd> {
    /// Create a new instance.
    ///
    /// Creation might fail if the GATT context cannot be reset, so user should ensure
    /// that there are no other GATT peripherals running before calling this function.
    pub fn new<B: BluetoothModemPeripheral + 'd>(
        _modem: B,
        context: &'a EspBtpGattContext,
    ) -> Result<Self, EspError> {
        context.reset()?;

        Ok(Self {
            _modem: core::marker::PhantomData,
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

        // Publish the context so that the NimBLE callbacks can reach it.
        CONTEXT.store(
            self.context as *const _ as *mut EspBtpGattContext,
            Ordering::SeqCst,
        );

        esp!(unsafe { nimble_port_init() }).map_err(to_matter_err)?;

        info!("NimBLE port initialized");

        unsafe {
            // `sm_*` are bitfields, so their setters take `&mut self`; going through a raw
            // pointer avoids materializing a reference to the `ble_hs_cfg` static itself.
            let cfg = &raw mut ble_hs_cfg;

            // Matter carries its own security on top of BTP, so BLE-level pairing is not used.
            (*cfg).set_sm_bonding(0);
            (*cfg).set_sm_mitm(0);
            (*cfg).set_sm_sc(0);
            (*cfg).sync_cb = Some(on_sync);
            (*cfg).reset_cb = Some(on_reset);

            ble_svc_gap_init();
            ble_svc_gatt_init();
        }

        self.register_service()?;

        let name = CString::new(service_name).map_err(|_| Error::new(ErrorCode::InvalidData))?;
        check(unsafe { ble_svc_gap_device_name_set(name.as_ptr()) })?;

        info!("BTP service registered, device name set to `{service_name}`");

        unsafe {
            nimble_port_freertos_init(Some(host_task));
        }

        info!("NimBLE host task started");

        select(self.process_incoming(btp), self.process_outgoing(btp))
            .coalesce()
            .await
    }

    /// Wire up and register the Matter BTP service with the NimBLE GATT server.
    fn register_service(&self) -> Result<(), Error> {
        // SAFETY: the host task is not started yet, so nothing else is touching these tables.
        unsafe {
            let chrs = &mut *CHRS.get();

            chrs[0].uuid = &C1_UUID.u as *const _;
            chrs[1].uuid = &C2_UUID.u as *const _;
            chrs[1].val_handle = C2_VAL_HANDLE.get();
            chrs[2].uuid = &C3_UUID.u as *const _;

            let svcs = &mut *SVCS.get();

            svcs[0].uuid = &SVC_UUID.u as *const _;
            svcs[0].characteristics = chrs.as_ptr();

            check(ble_gatts_count_cfg(svcs.as_ptr()))?;
            check(ble_gatts_add_svcs(svcs.as_ptr()))?;
        }

        Ok(())
    }

    /// Process incoming writes on characteristic `C1` from a remote peer.
    ///
    /// While it might seem that this can be done directly from `access_cb`, this is not
    /// generally possible because `Btp` might not be `Sync`, while `access_cb` has to be.
    async fn process_incoming(&self, btp: &Btp) -> Result<(), Error> {
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
                        let mtu = connection.mtu;
                        let peer = connection.peer;

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

    /// Indicate new data on characteristic `C2` to a remote peer.
    async fn process_outgoing(&self, btp: &Btp) -> Result<(), Error> {
        loop {
            let processed = self.context.state.lock(|state| {
                let mut state = state.borrow_mut();
                let state = &mut *state;

                let Some(conn) = state.connection.as_ref() else {
                    return Ok::<_, Error>(false);
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

                // SAFETY: written by NimBLE at registration time, before the host task starts.
                let c2_handle = unsafe { *C2_VAL_HANDLE.get() };
                if c2_handle == 0 {
                    return Ok(false);
                }

                state.out_data.resize_default(MAX_MTU_SIZE).unwrap();

                let len = btp.process_outgoing(conn.mtu, &mut state.out_data)?;
                if len > 0 {
                    let data = &state.out_data[..len];

                    // SAFETY: `ble_gatts_indicate_custom` consumes the mbuf, including on error.
                    let om = unsafe { ble_hs_mbuf_from_flat(data.as_ptr() as *const _, len as _) };
                    if om.is_null() {
                        error!("Out of NimBLE mbufs; cannot indicate {len} bytes");
                        return Err(Error::new(ErrorCode::NoSpace));
                    }

                    check(unsafe { ble_gatts_indicate_custom(conn.conn_handle, c2_handle, om) })?;

                    // Mark the current outgoing indication as not acknowledged
                    // until we receive the acknowledgment from the peer.
                    state.out_nack = true;

                    trace!("Indicated {len} bytes");

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

impl Drop for EspBtpGattPeripheral<'_, '_> {
    fn drop(&mut self) {
        CONTEXT.store(ptr::null_mut(), Ordering::SeqCst);
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

/// The NimBLE host task, which runs the host event loop until `nimble_port_stop` is called.
unsafe extern "C" fn host_task(_arg: *mut c_void) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/// Called by NimBLE once the host and the controller are in sync, and after every stack reset.
unsafe extern "C" fn on_sync() {
    if let Err(e) = ble_hs_util_ensure_addr(0).pipe(check) {
        error!("Cannot ensure a BLE address: {e:?}");
        return;
    }

    if let Err(e) = start_advertising() {
        error!("Cannot start advertising: {e:?}");
    }
}

/// Called by NimBLE when the stack resets due to a fatal error.
unsafe extern "C" fn on_reset(reason: c_int) {
    warn!("NimBLE stack reset, reason: {reason}");
}

/// Push the BTP advertising payload to the controller and start undirected, connectable
/// advertising.
fn start_advertising() -> Result<(), Error> {
    let Some(context) = context() else {
        return Ok(());
    };

    context.state.lock(|state| {
        let state = state.borrow();

        check(unsafe { ble_gap_adv_set_data(state.adv_data.as_ptr(), state.adv_data.len() as _) })
    })?;

    let adv_params = ble_gap_adv_params {
        conn_mode: BLE_GAP_CONN_MODE_UND as _,
        disc_mode: BLE_GAP_DISC_MODE_GEN as _,
        ..Default::default()
    };

    check(unsafe {
        ble_gap_adv_start(
            BLE_OWN_ADDR_PUBLIC as _,
            ptr::null(),
            // `BLE_HS_FOREVER`
            i32::MAX,
            &adv_params,
            Some(on_gap_event),
            ptr::null_mut(),
        )
    })?;

    info!("Advertising started");

    Ok(())
}

/// The GAP event handler, driving the connection state machine.
unsafe extern "C" fn on_gap_event(event: *mut ble_gap_event, _arg: *mut c_void) -> c_int {
    let Some(context) = context() else {
        return 0;
    };

    let event = &*event;

    match event.type_ as u32 {
        BLE_GAP_EVENT_CONNECT => {
            let conn = event.__bindgen_anon_1.connect;

            if conn.status != 0 {
                // The connection attempt failed, so we have to advertise again
                warn!("BLE connection failed, status: {}", conn.status);
                let _ = start_advertising();
                return 0;
            }

            let peer = peer_addr(conn.conn_handle);

            context.state.lock(|state| {
                let mut state = state.borrow_mut();

                state.conn_gen = state.conn_gen.wrapping_add(1);
                state.in_data.clear();
                state.out_data.clear();
                state.out_nack = false;
                state.connection = Some(Connection {
                    peer,
                    conn_handle: conn.conn_handle,
                    subscribed: false,
                    mtu: Some(ble_att_mtu(conn.conn_handle)),
                });
            });

            info!("BLE connected, handle: {}", conn.conn_handle);

            context.notify_process_incoming.signal(());
        }
        BLE_GAP_EVENT_DISCONNECT => {
            let conn = event.__bindgen_anon_1.disconnect;

            context.state.lock(|state| {
                let mut state = state.borrow_mut();

                state.connection = None;
                state.in_data.clear();
                state.out_data.clear();
                state.out_nack = false;
            });

            info!("BLE disconnected, reason: {}", conn.reason);

            context.notify_process_incoming.signal(());
            context.notify_process_outgoing.signal(());

            // Nobody is connected anymore, so become discoverable again
            if let Err(e) = start_advertising() {
                error!("Cannot re-start advertising: {e:?}");
            }
        }
        BLE_GAP_EVENT_SUBSCRIBE => {
            let subscribe = event.__bindgen_anon_1.subscribe;

            // SAFETY: written by NimBLE at registration time, before the host task starts.
            let c2_handle = *C2_VAL_HANDLE.get();

            if subscribe.attr_handle == c2_handle {
                let subscribed = subscribe.cur_indicate() != 0;

                context.state.lock(|state| {
                    let mut state = state.borrow_mut();

                    if let Some(conn) = state.connection.as_mut() {
                        if conn.conn_handle == subscribe.conn_handle {
                            conn.subscribed = subscribed;
                        }
                    }
                });

                info!(
                    "Peer {} to `C2`",
                    if subscribed {
                        "subscribed"
                    } else {
                        "unsubscribed"
                    }
                );

                context.notify_process_outgoing.signal(());
            }
        }
        BLE_GAP_EVENT_MTU => {
            let mtu = event.__bindgen_anon_1.mtu;

            context.state.lock(|state| {
                let mut state = state.borrow_mut();

                if let Some(conn) = state.connection.as_mut() {
                    if conn.conn_handle == mtu.conn_handle {
                        conn.mtu = Some(mtu.value);
                    }
                }
            });

            trace!("MTU negotiated: {}", mtu.value);
        }
        BLE_GAP_EVENT_NOTIFY_TX => {
            let notify_tx = event.__bindgen_anon_1.notify_tx;

            // An indication is complete once the peer confirms it (`BLE_HS_EDONE`), or once it
            // fails; either way the next one may go out.
            if notify_tx.indication() != 0
                && (notify_tx.status == BLE_HS_EDONE as _ || notify_tx.status != 0)
            {
                context.state.lock(|state| {
                    let mut state = state.borrow_mut();
                    state.out_nack = false;
                });

                context.notify_process_outgoing.signal(());
            }
        }
        _ => (),
    }

    0
}

/// The GATT access handler: `C1` writes carry incoming BTP packets, `C3` reads are answered
/// with an empty payload, as no additional commissioning data is provided.
unsafe extern "C" fn access_cb(
    _conn_handle: u16,
    _attr_handle: u16,
    ctxt: *mut ble_gatt_access_ctxt,
    _arg: *mut c_void,
) -> c_int {
    let Some(context) = context() else {
        return BLE_ATT_ERR_INSUFFICIENT_RES as _;
    };

    let ctxt = &*ctxt;

    match ctxt.op as u32 {
        BLE_GATT_ACCESS_OP_WRITE_CHR => {
            let result = context.state.lock(|state| {
                let mut state = state.borrow_mut();

                if !state.in_data.is_empty() {
                    // The previous write has not been processed yet
                    return Err(());
                }

                state.in_data.resize_default(MAX_MTU_SIZE).map_err(|_| ())?;

                let mut len = 0u16;
                let rc = ble_hs_mbuf_to_flat(
                    ctxt.om,
                    state.in_data.as_mut_ptr() as *mut _,
                    MAX_MTU_SIZE as _,
                    &mut len,
                );

                if rc != 0 {
                    state.in_data.clear();
                    return Err(());
                }

                state.in_data.truncate(len as _);

                Ok(())
            });

            if result.is_err() {
                return BLE_ATT_ERR_INSUFFICIENT_RES as _;
            }

            context.notify_process_incoming.signal(());

            0
        }
        BLE_GATT_ACCESS_OP_READ_CHR => 0,
        _ => BLE_ATT_ERR_INSUFFICIENT_RES as _,
    }
}

/// Fetch the address of the peer on the other end of `conn_handle`.
unsafe fn peer_addr(conn_handle: u16) -> BtAddr {
    let mut desc = ble_gap_conn_desc::default();

    if ble_gap_conn_find(conn_handle, &mut desc) == 0 {
        BtAddr(desc.peer_id_addr.val)
    } else {
        BtAddr([0; 6])
    }
}

/// Turn a NimBLE return code into a `Result`.
fn check(rc: c_int) -> Result<(), Error> {
    if rc == 0 {
        Ok(())
    } else {
        error!("NimBLE call failed, rc: {rc}");
        Err(Error::new(ErrorCode::NoNetworkInterface))
    }
}

fn to_matter_err(e: EspError) -> Error {
    error!("Error: {e:?}");
    Error::new(ErrorCode::NoNetworkInterface)
}

/// A tiny helper so that `on_sync` reads linearly.
trait Pipe: Sized {
    fn pipe<R>(self, f: impl FnOnce(Self) -> R) -> R {
        f(self)
    }
}

impl<T> Pipe for T {}
