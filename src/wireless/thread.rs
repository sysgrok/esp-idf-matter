use alloc::sync::Arc;

#[cfg(esp_idf_bt_bluedroid_enabled)]
use esp_idf_svc::bt::{self, BtDriver};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::modem::Modem;
use esp_idf_svc::io::vfs::MountedEventfs;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::thread::{EspThread, Node};

use log::info;

use rs_matter_stack::matter::dm::clusters::gen_diag::InterfaceTypeEnum;
use rs_matter_stack::matter::dm::networks::wireless::Thread;
use rs_matter_stack::matter::error::Error;

use rs_matter_stack::network::{Embedding, Network};
use rs_matter_stack::wireless::{Gatt, GattTask, ThreadCoex, ThreadCoexTask, ThreadTask};

use crate::ble::{EspBtpGattContext, EspBtpGattPeripheral};
use crate::error::to_net_error;
use crate::netif::{EspMatterNetStack, EspMatterNetif};
use crate::thread::{EspMatterThreadCtl, EspMatterThreadSrp};

use super::EspWirelessMatterStack;
#[cfg(esp_idf_bt_bluedroid_enabled)]
use super::GATTS_APP_ID;

extern crate alloc;

/// A type alias for an ESP-IDF Matter stack running over Thread (and BLE, during commissioning).
pub type EspThreadMatterStack<'a, const B: usize, E> = EspWirelessMatterStack<'a, B, Thread, E>;

/// A `Thread` trait implementation via ESP-IDF's Thread/BT modem
pub struct EspMatterThread<'a, 'd> {
    modem: Modem<'d>,
    sysloop: EspSystemEventLoop,
    nvs: EspDefaultNvsPartition,
    mounted_event_fs: Arc<MountedEventfs>,
    ble_context: &'a EspBtpGattContext,
    srp_host_eui64: Option<[u8; 8]>,
}

impl<'a, 'd> EspMatterThread<'a, 'd> {
    /// Create a new instance of the `EspMatterThread` type.
    pub fn new<const B: usize, E>(
        modem: Modem<'d>,
        sysloop: EspSystemEventLoop,
        nvs: EspDefaultNvsPartition,
        mounted_event_fs: Arc<MountedEventfs>,
        stack: &'a EspThreadMatterStack<B, E>,
    ) -> Self
    where
        E: Embedding + 'static,
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

        task.run(
            EspMatterNetStack::new(),
            EspMatterNetif::new(&net_ctl, InterfaceTypeEnum::Thread, self.sysloop.clone()),
            &net_ctl,
            &mut mdns,
        )
        .await
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

        task.run(
            EspMatterNetStack::new(),
            EspMatterNetif::new(&net_ctl, InterfaceTypeEnum::Thread, self.sysloop.clone()),
            &net_ctl,
            &mut mdns,
            &mut peripheral,
        )
        .await
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
