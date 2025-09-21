use esp_idf_svc::bt::{self, BtDriver};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::modem::Modem;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::timer::EspTaskTimerService;
use esp_idf_svc::wifi::{AsyncWifi, EspWifi};

use rs_matter_stack::matter::dm::clusters::gen_diag::InterfaceTypeEnum;
use rs_matter_stack::matter::dm::networks::wireless::Wifi;
use rs_matter_stack::matter::error::Error;

use rs_matter_stack::mdns::{BuiltinMdns, Mdns};
use rs_matter_stack::network::{Embedding, Network};
use rs_matter_stack::wireless::{Gatt, GattTask, WifiCoex, WifiCoexTask, WifiTask};

use crate::ble::{EspBtpGattContext, EspBtpGattPeripheral};
use crate::error::to_net_error;
use crate::netif::{EspMatterNetStack, EspMatterNetif};
use crate::wifi::EspMatterWifiCtl;

use super::{EspWirelessMatterStack, GATTS_APP_ID};

/// A type alias for an ESP-IDF Matter stack running over Wifi (and BLE, during commissioning).
pub type EspWifiMatterStack<'a, const B: usize, E> = EspWirelessMatterStack<'a, B, Wifi, E>;

/// A `Wifi` trait implementation via ESP-IDF's Wifi/BT modem
pub struct EspMatterWifi<'a, 'd, M = BuiltinMdns> {
    modem: Modem<'d>,
    sysloop: EspSystemEventLoop,
    timer: EspTaskTimerService,
    nvs: EspDefaultNvsPartition,
    mdns: M,
    ble_context: &'a EspBtpGattContext,
}

impl<'a, 'd> EspMatterWifi<'a, 'd, BuiltinMdns> {
    /// Create a new instance of the `EspMatterWifi` type .
    pub fn new_with_builtin_mdns<const B: usize, E>(
        modem: Modem<'d>,
        sysloop: EspSystemEventLoop,
        timer: EspTaskTimerService,
        nvs: EspDefaultNvsPartition,
        stack: &'a EspWifiMatterStack<B, E>,
    ) -> Self
    where
        E: Embedding + 'static,
    {
        Self::new(modem, sysloop, timer, nvs, stack, BuiltinMdns)
    }
}

impl<'a, 'd, M> EspMatterWifi<'a, 'd, M>
where
    M: Mdns,
{
    /// Create a new instance of the `EspMatterWifi` type.
    pub fn new<const B: usize, E>(
        modem: Modem<'d>,
        sysloop: EspSystemEventLoop,
        timer: EspTaskTimerService,
        nvs: EspDefaultNvsPartition,
        stack: &'a EspWifiMatterStack<B, E>,
        mdns: M,
    ) -> Self
    where
        E: Embedding + 'static,
    {
        Self::wrap(
            modem,
            sysloop,
            timer,
            nvs,
            mdns,
            stack.network().embedding().context(),
        )
    }

    /// Wrap existing parts into a new instance of the `EspMatterWifi` type.
    pub fn wrap(
        modem: Modem<'d>,
        sysloop: EspSystemEventLoop,
        timer: EspTaskTimerService,
        nvs: EspDefaultNvsPartition,
        mdns: M,
        ble_context: &'a EspBtpGattContext,
    ) -> Self {
        Self {
            modem,
            sysloop,
            timer,
            nvs,
            mdns,
            ble_context,
        }
    }
}

impl Gatt for EspMatterWifi<'_, '_> {
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: GattTask,
    {
        let bt = BtDriver::new(unsafe { self.modem.reborrow() }, Some(self.nvs.clone())).unwrap();

        let peripheral =
            EspBtpGattPeripheral::<bt::Ble>::new(GATTS_APP_ID, bt, self.ble_context).unwrap();

        task.run(peripheral).await
    }
}

impl rs_matter_stack::wireless::Wifi for EspMatterWifi<'_, '_> {
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: WifiTask,
    {
        let wifi = AsyncWifi::wrap(
            EspWifi::new(
                unsafe { self.modem.reborrow() },
                self.sysloop.clone(),
                Some(self.nvs.clone()),
            )
            .map_err(to_net_error)?,
            self.sysloop.clone(),
            self.timer.clone(),
        )
        .map_err(to_net_error)?;

        let wifi = EspMatterWifiCtl::new(wifi, self.sysloop.clone());

        task.run(
            EspMatterNetStack::new(),
            EspMatterNetif::new(&wifi, InterfaceTypeEnum::WiFi, self.sysloop.clone()),
            &wifi,
            &mut self.mdns,
        )
        .await
    }
}

impl WifiCoex for EspMatterWifi<'_, '_> {
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: WifiCoexTask,
    {
        let modem = unsafe { self.modem.reborrow() };

        #[cfg(not(esp32c6))]
        let (wifi_p, bt_p) = modem.split();

        #[cfg(esp32c6)]
        let (wifi_p, _, bt_p) = modem.split();

        let wifi = AsyncWifi::wrap(
            EspWifi::new(wifi_p, self.sysloop.clone(), Some(self.nvs.clone()))
                .map_err(to_net_error)?,
            self.sysloop.clone(),
            self.timer.clone(),
        )
        .map_err(to_net_error)?;

        let net_ctl = EspMatterWifiCtl::new(wifi, self.sysloop.clone());

        let bt = BtDriver::new(bt_p, Some(self.nvs.clone())).unwrap();

        let mut peripheral =
            EspBtpGattPeripheral::<bt::Ble>::new(GATTS_APP_ID, bt, self.ble_context).unwrap();

        task.run(
            EspMatterNetStack::new(),
            EspMatterNetif::new(&net_ctl, InterfaceTypeEnum::WiFi, self.sysloop.clone()),
            &net_ctl,
            &mut self.mdns,
            &mut peripheral,
        )
        .await
    }
}
