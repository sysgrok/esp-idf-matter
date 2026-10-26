use core::pin::pin;

use alloc::sync::Arc;

use embassy_futures::select::select;

#[cfg(esp_idf_bt_bluedroid_enabled)]
use esp_idf_svc::bt::{self, BtDriver};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::modem::Modem;
use esp_idf_svc::io::vfs::MountedEventfs;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::thread::{EspThread, Node};

use log::info;

use rs_matter_stack::matter::dm::clusters::gen_diag::InterfaceTypeEnum;
use rs_matter_stack::matter::dm::clusters::icd_mgmt::{Icd, IcdNetParams};
use rs_matter_stack::matter::dm::networks::wireless::{Thread, ThreadNetworks};
use rs_matter_stack::matter::error::Error;
use rs_matter_stack::matter::utils::select::Coalesce;

use rs_matter_stack::network::{Embedding, Network};
use rs_matter_stack::wireless::{
    Gatt, GattTask, ThreadCoex, ThreadCoexTask, ThreadTask, MAX_WIRELESS_NETWORKS,
};

use crate::ble::{EspBtpGattContext, EspBtpGattPeripheral};
use crate::error::to_net_error;
use crate::netif::{EspMatterNetStack, EspMatterNetif};
use crate::thread::{EspMatterThreadCtl, EspMatterThreadSrp};

use super::EspWirelessMatterStack;
#[cfg(esp_idf_bt_bluedroid_enabled)]
use super::GATTS_APP_ID;

extern crate alloc;

/// A type alias for an ESP-IDF Matter stack running over Thread (and BLE, during commissioning).
///
/// `S` is the networks store of the Thread interface.
pub type EspThreadMatterStack<'a, const B: usize, E, S = ThreadNetworks<MAX_WIRELESS_NETWORKS>> =
    EspWirelessMatterStack<'a, B, Thread, E, S>;

/// A `Thread` trait implementation via ESP-IDF's Thread/BT modem
pub struct EspMatterThread<'a, 'd> {
    modem: Modem<'d>,
    sysloop: EspSystemEventLoop,
    nvs: EspDefaultNvsPartition,
    mounted_event_fs: Arc<MountedEventfs>,
    ble_context: &'a EspBtpGattContext,
    srp_host_eui64: Option<[u8; 8]>,
    icd: Option<&'a Icd>,
}

impl<'a, 'd> EspMatterThread<'a, 'd> {
    /// Create a new instance of the `EspMatterThread` type.
    pub fn new<const B: usize, E, S>(
        modem: Modem<'d>,
        sysloop: EspSystemEventLoop,
        nvs: EspDefaultNvsPartition,
        mounted_event_fs: Arc<MountedEventfs>,
        stack: &'a EspThreadMatterStack<B, E, S>,
    ) -> Self
    where
        E: Embedding + 'static,
        S: Embedding + 'static,
    {
        Self::wrap(
            modem,
            sysloop,
            nvs,
            mounted_event_fs,
            stack.network().embedding().context(),
        )
    }

    /// Wrap existing parts into a new instance of the `EspMatterThread` type.
    pub fn wrap(
        modem: Modem<'d>,
        sysloop: EspSystemEventLoop,
        nvs: EspDefaultNvsPartition,
        mounted_event_fs: Arc<MountedEventfs>,
        ble_context: &'a EspBtpGattContext,
    ) -> Self {
        Self {
            modem,
            sysloop,
            nvs,
            mounted_event_fs,
            ble_context,
            srp_host_eui64: None,
            icd: None,
        }
    }

    /// Derive the Thread SRP host name from `eui64` rather than from the
    /// factory-programmed IEEE 802.15.4 address.
    ///
    /// A production device should keep the default: the SRP host name is meant to be
    /// stable, and its records on the SRP server are keyed by an ECDSA key that
    /// OpenThread persists alongside its other settings.
    ///
    /// Examples and demos are the exception. Erasing the flash discards that ECDSA key,
    /// but the SRP server keeps the records the device registered under the old key
    /// until their key lease runs out - 14 days, by default. Until then the server
    /// answers every re-registration attempt under the same host name with `YXDOMAIN`
    /// ("name exists"), and the device stays undiscoverable. Passing a freshly randomized
    /// EUI-64 on each boot side-steps that, at the cost of leaving one dead host record
    /// behind on the server per run.
    pub fn with_srp_host_eui64(mut self, eui64: [u8; 8]) -> Self {
        self.srp_host_eui64 = Some(eui64);

        self
    }

    /// Run as a Sleepy End Device that follows the power mode of `icd`: it polls its
    /// parent fast while the ICD is active and slowly while it is idle.
    pub fn with_icd(mut self, icd: &'a Icd) -> Self {
        self.icd = Some(icd);

        self
    }
}

impl Gatt for EspMatterThread<'_, '_> {
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: GattTask,
    {
        #[cfg(esp_idf_bt_bluedroid_enabled)]
        let peripheral = {
            let bt =
                BtDriver::new(unsafe { self.modem.reborrow() }, Some(self.nvs.clone())).unwrap();

            EspBtpGattPeripheral::<bt::Ble>::new(GATTS_APP_ID, bt, self.ble_context).unwrap()
        };

        #[cfg(not(esp_idf_bt_bluedroid_enabled))]
        let peripheral =
            EspBtpGattPeripheral::new(unsafe { self.modem.reborrow() }, self.ble_context).unwrap();

        task.run(peripheral).await
    }
}

impl rs_matter_stack::wireless::Thread for EspMatterThread<'_, '_> {
    // The operational task receives `&net_ctl` (a `&EspMatterThreadCtl`) built from a
    // locally-created `EspThread<'_, Node>`, so the chain net-ctl type — and hence this
    // associated type — is `&'a EspMatterThreadCtl<'a, 'a, Node>`. Naming it here lets
    // the commissioning and operational handler chains share one `WirelessNetCtl` type
    // (single monomorphization).
    type NetCtl<'a>
        = &'a EspMatterThreadCtl<'a, 'a, Node>
    where
        Self: 'a;

    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: ThreadTask,
    {
        let mut thread = EspThread::new(
            unsafe { self.modem.reborrow() },
            self.sysloop.clone(),
            self.nvs.clone(),
            self.mounted_event_fs.clone(),
        )
        .map_err(to_net_error)?;

        // Drop any Active Operational Dataset ESP-IDF persisted in NVS on a previous run.
        //
        // The Matter stack is the only source of the dataset - `NetCtl::connect` always
        // supplies it from `rs-matter`'s own network store - so OpenThread's copy is never
        // read, but it does keep the radio holding credentials for a network the Matter
        // stack may no longer know anything about. Enabling IPv6 below then brings up the
        // Thread netif (and with it the MLE socket) while Thread itself stays disabled
        // until `connect()`, so those stale credentials would let the neighbours' MLE
        // traffic through the MAC filter only for MLE to reject every message.
        thread.set_tod(&[]).map_err(to_net_error)?;

        if let Some(icd) = self.icd {
            configure_sed(&thread, icd)?;
        }

        thread.enable_ipv6(true).map_err(to_net_error)?;
        thread.srp_autostart().map_err(to_net_error)?;

        info!("Thread stack created, about to start it");

        thread.start().map_err(to_net_error)?;

        info!("Thread stack started");

        // Declared before everything that borrows `thread`, so that it is dropped
        // *after* them - and after the task future itself - on either exit path
        let _quiesce = ThreadQuiesce(&thread);

        let net_ctl = EspMatterThreadCtl::new(&thread, self.sysloop.clone());
        let mut mdns = EspMatterThreadSrp::new_with_host_eui64(&thread, self.srp_host_eui64);

        let mut task = pin!(task.run(
            EspMatterNetStack::new(),
            EspMatterNetif::new(&net_ctl, InterfaceTypeEnum::Thread, self.sysloop.clone()),
            &net_ctl,
            &mut mdns,
        ));
        let mut icd = pin!(run_icd(&thread, self.icd));

        select(&mut task, &mut icd).coalesce().await
    }
}

impl ThreadCoex for EspMatterThread<'_, '_> {
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: ThreadCoexTask,
    {
        let modem = unsafe { self.modem.reborrow() };

        #[cfg(not(esp32c6))]
        let (thread_p, bt_p) = modem.split();

        #[cfg(esp32c6)]
        let (_, thread_p, bt_p) = modem.split();

        let mut thread = EspThread::new(
            thread_p,
            self.sysloop.clone(),
            self.nvs.clone(),
            self.mounted_event_fs.clone(),
        )
        .map_err(to_net_error)?;

        // Drop any Active Operational Dataset ESP-IDF persisted in NVS on a previous run.
        //
        // The Matter stack is the only source of the dataset - `NetCtl::connect` always
        // supplies it from `rs-matter`'s own network store - so OpenThread's copy is never
        // read, but it does keep the radio holding credentials for a network the Matter
        // stack may no longer know anything about. Enabling IPv6 below then brings up the
        // Thread netif (and with it the MLE socket) while Thread itself stays disabled
        // until `connect()`, so those stale credentials would let the neighbours' MLE
        // traffic through the MAC filter only for MLE to reject every message.
        thread.set_tod(&[]).map_err(to_net_error)?;

        if let Some(icd) = self.icd {
            configure_sed(&thread, icd)?;
        }

        thread.enable_ipv6(true).map_err(to_net_error)?;
        thread.srp_autostart().map_err(to_net_error)?;

        info!("Thread stack created, about to start it");

        thread.start().map_err(to_net_error)?;

        info!("Thread stack started");

        // Declared before everything that borrows `thread`, so that it is dropped
        // *after* them - and after the task future itself - on either exit path
        let _quiesce = ThreadQuiesce(&thread);

        let net_ctl = EspMatterThreadCtl::new(&thread, self.sysloop.clone());
        let mut mdns = EspMatterThreadSrp::new_with_host_eui64(&thread, self.srp_host_eui64);
        #[cfg(esp_idf_bt_bluedroid_enabled)]
        let mut peripheral = {
            let bt = BtDriver::new(bt_p, Some(self.nvs.clone())).unwrap();

            EspBtpGattPeripheral::<bt::Ble>::new(GATTS_APP_ID, bt, self.ble_context).unwrap()
        };

        #[cfg(not(esp_idf_bt_bluedroid_enabled))]
        let mut peripheral = EspBtpGattPeripheral::new(bt_p, self.ble_context).unwrap();

        let mut task = pin!(task.run(
            EspMatterNetStack::new(),
            EspMatterNetif::new(&net_ctl, InterfaceTypeEnum::Thread, self.sysloop.clone()),
            &net_ctl,
            &mut mdns,
            &mut peripheral,
        ));
        let mut icd = pin!(run_icd(&thread, self.icd));

        select(&mut task, &mut icd).coalesce().await
    }
}

/// Puts the Thread stack back to rest when the driver task ends.
///
/// This has to be a drop guard rather than code following the `await`: the driver task
/// is raced against other futures, so the overwhelmingly common way for it to end is to
/// be dropped (cancelled) rather than to return.
///
/// Bind it *before* anything that borrows the `EspThread` (and hence before the task
/// future is created), so that drop order tears the task down first and only then
/// quiesces the radio.
struct ThreadQuiesce<'a, 'd>(&'a EspThread<'d, Node>);

impl Drop for ThreadQuiesce<'_, '_> {
    fn drop(&mut self) {
        let _ = self.0.enable_thread(false);
        let _ = self.0.srp_stop();
        let _ = self.0.enable_ipv6(false);
    }
}

/// OpenThread's default child timeout, in seconds (`OPENTHREAD_CONFIG_MLE_CHILD_TIMEOUT_DEFAULT`).
const CHILD_TIMEOUT_DEFAULT_S: u32 = 240;

/// The margin added on top of the slowest polling interval when sizing the child timeout: two
/// missed polls plus some slack.
const CHILD_TIMEOUT_MARGIN_S: u32 = 30;

/// OpenThread's default child-supervision interval, in seconds
/// (`OPENTHREAD_CONFIG_CHILD_SUPERVISION_INTERVAL`): the child asks its parent for a supervision
/// message at least this often. Left at its default here.
const CHILD_SUPERVISION_INTERVAL_S: u32 = 129;

/// OpenThread's default child-supervision check timeout, in seconds
/// (`OPENTHREAD_CONFIG_CHILD_SUPERVISION_CHECK_TIMEOUT`).
const CHILD_SUPERVISION_CHECK_TIMEOUT_DEFAULT_S: u32 = 190;

/// Configure the node as a Sleepy End Device (receiver off when idle, MTD, stable network data
/// only) and size its keep-alives from the slowest polling interval the ICD will ever use.
fn configure_sed(thread: &EspThread<'_, Node>, icd: &Icd) -> Result<(), Error> {
    let params = icd.net_params();

    thread
        .set_link_mode(false, false, false)
        .map_err(to_net_error)?;
    thread
        .set_child_timeout(child_timeout_s(params.max_poll_interval_ms))
        .map_err(to_net_error)?;

    apply_icd_params(thread, &params)
}

/// Apply the current ICD polling interval to OpenThread.
fn apply_icd_params(thread: &EspThread<'_, Node>, params: &IcdNetParams) -> Result<(), Error> {
    info!(
        "Thread SED: {:?} / {:?}, polling every {} ms",
        params.power_mode, params.operating_mode, params.poll_interval_ms
    );

    thread
        .set_poll_period(params.poll_interval_ms)
        .map_err(to_net_error)?;
    thread
        .set_child_supervision_check_timeout(child_supervision_check_timeout_s(
            params.poll_interval_ms,
        ))
        .map_err(to_net_error)
}

/// Follow the ICD power mode: re-apply the polling interval whenever it changes.
///
/// Never completes; a no-op (that never completes either) without an ICD.
async fn run_icd(thread: &EspThread<'_, Node>, icd: Option<&Icd>) -> Result<(), Error> {
    let Some(icd) = icd else {
        core::future::pending::<()>().await;
        unreachable!()
    };

    // The state machine signals every change of its own state, including the active deadline
    // moving out on each message received, which leaves the net params as they were. Only a
    // change of those is worth (re-)programming OpenThread for - and logging.
    // `configure_sed` applied the first ones.
    let mut applied = icd.net_params();

    loop {
        icd.wait_net_changed().await;

        let params = icd.net_params();

        if params != applied {
            apply_icd_params(thread, &params)?;
            applied = params;
        }
    }
}

/// The child timeout that fits a polling interval: the parent must not evict the child between
/// two polls, so the timeout exceeds the interval by a margin - but is never shorter than
/// OpenThread's default.
fn child_timeout_s(poll_period_ms: u32) -> u32 {
    poll_period_ms
        .div_ceil(1000)
        .saturating_add(CHILD_TIMEOUT_MARGIN_S)
        .max(CHILD_TIMEOUT_DEFAULT_S)
}

/// The child-supervision check timeout that matches a polling interval, never below
/// OpenThread's default.
///
/// A sleepy child only gets the parent's queued supervision message when it polls, so the
/// longest normal gap between frames from the parent is the supervision interval plus one
/// polling interval. With a check timeout below that gap (e.g. the 190 s default against a
/// 900 s interval), every idle period ends in a false "supervision timeout" and a needless
/// Child Update Request. One more polling interval is added as margin for a missed poll.
fn child_supervision_check_timeout_s(poll_period_ms: u32) -> u16 {
    CHILD_SUPERVISION_INTERVAL_S
        .saturating_add(poll_period_ms.div_ceil(1000).saturating_mul(2))
        .clamp(
            CHILD_SUPERVISION_CHECK_TIMEOUT_DEFAULT_S,
            u32::from(u16::MAX),
        ) as u16
}
