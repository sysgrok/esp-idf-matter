use core::marker::PhantomData;

#[cfg(esp_idf_bt_bluedroid_enabled)]
use esp_idf_svc::bt::{self, BtDriver};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::modem::Modem;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::timer::EspTaskTimerService;
use esp_idf_svc::wifi::{AsyncWifi, EspWifi};

use rs_matter::dm::clusters::gen_diag::InterfaceTypeEnum;
use rs_matter::dm::networks::wireless::WifiNetworks;
use rs_matter::error::Error;

use rs_matter::stack::mdns::Mdns;
use rs_matter::stack::network::{Embedding, StackNetwork, WifiStackNetwork};
use rs_matter::stack::wireless::{
    GattDriver, GattTask, WifiCoexDriver, WifiCoexTask, WifiDriver, WifiTask,
};
use rs_matter::stack::MatterStack;
use rs_matter::transport::network::mdns::builtin::BuiltinMdns;

use crate::ble::{EspBtpGattContext, EspBtpGattPeripheral};
use crate::error::to_net_error;
use crate::netif::{EspMatterNetStack, EspMatterNetif};
use crate::wifi::EspMatterWifiCtl;

#[cfg(esp_idf_bt_bluedroid_enabled)]
use super::GATTS_APP_ID;

/// The network of a `MatterStack` that runs Wifi with [`EspMatterWifi`]: the networks
/// store `S` (a Wifi one, `WifiNetworks` by default), and the context of the BLE GATT
/// peripheral it commissions over.
pub struct EspWifiNetwork<S = WifiNetworks>(PhantomData<fn() -> S>);

impl<S> StackNetwork for EspWifiNetwork<S>
where
    S: WifiStackNetwork<Context = ()>,
{
    type Store = S::Store;
    type Context = EspBtpGattContext;
}

impl<S> WifiStackNetwork for EspWifiNetwork<S> where S: WifiStackNetwork<Context = ()> {}

/// The network of a stack that [`EspMatterWifi`] can be built for: `EspWifiNetwork`, or
/// `EspAnyNetwork` on chips with both radios.
pub trait EspWifiCapable: StackNetwork<Context = EspBtpGattContext> {}

impl<S> EspWifiCapable for EspWifiNetwork<S> where S: WifiStackNetwork<Context = ()> {}

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
    pub fn new_with_builtin_mdns<const B: usize, S, E>(
        modem: Modem<'d>,
        sysloop: EspSystemEventLoop,
        timer: EspTaskTimerService,
        nvs: EspDefaultNvsPartition,
        stack: &'a MatterStack<B, S, E>,
    ) -> Self
    where
        S: EspWifiCapable,
        E: Embedding,
    {
        Self::new(modem, sysloop, timer, nvs, stack, BuiltinMdns::new())
    }
}

impl<'a, 'd, M> EspMatterWifi<'a, 'd, M>
where
    M: Mdns,
{
    /// Create a new instance of the `EspMatterWifi` type.
    pub fn new<const B: usize, S, E>(
        modem: Modem<'d>,
        sysloop: EspSystemEventLoop,
        timer: EspTaskTimerService,
        nvs: EspDefaultNvsPartition,
        stack: &'a MatterStack<B, S, E>,
        mdns: M,
    ) -> Self
    where
        S: EspWifiCapable,
        E: Embedding,
    {
        Self::wrap(modem, sysloop, timer, nvs, mdns, stack.context())
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

impl GattDriver for EspMatterWifi<'_, '_> {
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

impl WifiDriver for EspMatterWifi<'_, '_> {
    // The operational task receives `&wifi` (a `&EspMatterWifiCtl`), so the chain
    // net-ctl type — and hence this associated type — is `&'a EspMatterWifiCtl<'a>`.
    // Naming it here lets the commissioning and operational handler chains share one
    // `WirelessNetCtl` type (single monomorphization).
    type NetCtl<'a>
        = &'a EspMatterWifiCtl<'a>
    where
        Self: 'a;

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

impl WifiCoexDriver for EspMatterWifi<'_, '_> {
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: WifiCoexTask,
    {
        let modem = unsafe { self.modem.reborrow() };

        #[cfg(not(any(esp32c5, esp32c6, esp32c61, esp32s31)))]
        let (wifi_p, bt_p) = modem.split();

        #[cfg(any(esp32c5, esp32c6, esp32c61, esp32s31))]
        let (wifi_p, _, bt_p) = modem.split();

        let wifi = AsyncWifi::wrap(
            EspWifi::new(wifi_p, self.sysloop.clone(), Some(self.nvs.clone()))
                .map_err(to_net_error)?,
            self.sysloop.clone(),
            self.timer.clone(),
        )
        .map_err(to_net_error)?;

        let net_ctl = EspMatterWifiCtl::new(wifi, self.sysloop.clone());

        #[cfg(esp_idf_bt_bluedroid_enabled)]
        let mut peripheral = {
            let bt = BtDriver::new(bt_p, Some(self.nvs.clone())).unwrap();

            EspBtpGattPeripheral::<bt::Ble>::new(GATTS_APP_ID, bt, self.ble_context).unwrap()
        };

        #[cfg(not(esp_idf_bt_bluedroid_enabled))]
        let mut peripheral = EspBtpGattPeripheral::new(bt_p, self.ble_context).unwrap();

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
