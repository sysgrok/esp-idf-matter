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
        }
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

        thread.enable_ipv6(true).map_err(to_net_error)?;
        thread.enable_thread(true).map_err(to_net_error)?;

        info!("Thread stack created, about to start it");

        thread.start().map_err(to_net_error)?;

        info!("Thread stack started");

        let net_ctl = EspMatterThreadCtl::new(&thread, self.sysloop.clone());
        let mut mdns = EspMatterThreadSrp::new(&thread);

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

        thread.enable_ipv6(true).map_err(to_net_error)?;
        thread.enable_thread(true).map_err(to_net_error)?;

        info!("Thread stack created, about to start it");

        thread.start().map_err(to_net_error)?;

        info!("Thread stack started");

        let net_ctl = EspMatterThreadCtl::new(&thread, self.sysloop.clone());
        let mut mdns = EspMatterThreadSrp::new(&thread);
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
