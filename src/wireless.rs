//! This module provides the ESP-IDF implementation of the Wifi and Thread Matter stacks.

use rs_matter::stack::network::Embedding;
use rs_matter::utils::init::Init;

use crate::ble::EspBtpGattContext;

#[cfg(all(
    esp_idf_comp_openthread_enabled,
    esp_idf_openthread_enabled,
    esp_idf_soc_ieee802154_supported,
    esp_idf_comp_vfs_enabled,
))]
pub use thread::*;

#[cfg(all(esp_idf_comp_esp_wifi_enabled, not(esp32h2)))]
pub use wifi::*;

#[cfg(all(
    esp_idf_comp_openthread_enabled,
    esp_idf_openthread_enabled,
    esp_idf_soc_ieee802154_supported,
    esp_idf_comp_vfs_enabled,
    esp_idf_comp_esp_wifi_enabled,
    not(esp32h2),
))]
pub use any::*;

#[cfg(all(
    esp_idf_comp_openthread_enabled,
    esp_idf_openthread_enabled,
    esp_idf_soc_ieee802154_supported,
    esp_idf_comp_vfs_enabled,
))]
mod thread;

#[cfg(all(esp_idf_comp_esp_wifi_enabled, not(esp32h2)))]
mod wifi;

#[cfg(all(
    esp_idf_comp_openthread_enabled,
    esp_idf_openthread_enabled,
    esp_idf_soc_ieee802154_supported,
    esp_idf_comp_vfs_enabled,
    esp_idf_comp_esp_wifi_enabled,
    not(esp32h2),
))]
mod any;

/// The context of the BLE GATT peripheral, which the stack holds for the Wifi and Thread
/// drivers (see `EspThreadNetwork`, `EspWifiNetwork` and `EspAnyNetwork`).
impl Embedding for EspBtpGattContext {
    const INIT: Self = Self::new();

    fn init() -> impl Init<Self> {
        EspBtpGattContext::init()
    }
}

/// The GATT application ID of the BTP service. A Bluedroid notion; NimBLE has no equivalent.
#[cfg(esp_idf_bt_bluedroid_enabled)]
const GATTS_APP_ID: u16 = 0;
