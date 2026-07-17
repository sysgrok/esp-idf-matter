//! This module provides the ESP-IDF implementation of the Wifi and Thread Matter stacks.

use rs_matter_stack::matter::utils::init::{init, Init};
use rs_matter_stack::network::Embedding;
use rs_matter_stack::wireless::WirelessBle;
use rs_matter_stack::MatterStack;

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
))]
mod thread;

#[cfg(all(esp_idf_comp_esp_wifi_enabled, not(esp32h2)))]
mod wifi;

/// A type alias for an ESP-IDF Matter stack running over a wireless network (Wifi or Thread) and BLE.
pub type EspWirelessMatterStack<'a, const B: usize, T, E> =
    MatterStack<'a, B, EspWirelessBle<T, E>>;

/// A type alias for an ESP-IDF implementation of the `Network` trait for a Matter stack running over
/// BLE during commissioning, and then over either WiFi or Thread when operating.
pub type EspWirelessBle<T, E> = WirelessBle<T, EspGatt<E>>;

/// An embedding of the ESP IDF Bluedroid Gatt peripheral context for the `WirelessBle` network type from `rs-matter-stack`.
///
/// Allows the memory of this context to be statically allocated and cost-initialized.
///
/// Usage:
/// ```no_run
/// MatterStack<WirelessBle<Wifi, KvBlobBuf<EspGatt<E>>>>::new(...);
/// ```
///
/// ... where `E` can be a next-level, user-supplied embedding or just `()` if the user does not need to embed anything.
pub struct EspGatt<E = ()> {
    btp_gatt_context: EspBtpGattContext,
    embedding: E,
}

impl<E> EspGatt<E>
where
    E: Embedding,
{
    /// Creates a new instance of the `EspGatt` embedding.
    #[allow(clippy::large_stack_frames)]
    #[inline(always)]
    const fn new() -> Self {
        Self {
            btp_gatt_context: EspBtpGattContext::new(),
            embedding: E::INIT,
        }
    }

    /// Return an in-place initializer for the `EspGatt` embedding.
    fn init() -> impl Init<Self> {
        init!(Self {
            btp_gatt_context <- EspBtpGattContext::init(),
            embedding <- E::init(),
        })
    }

    /// Return a reference to the Bluedroid Gatt peripheral context.
    pub fn context(&self) -> &EspBtpGattContext {
        &self.btp_gatt_context
    }

    /// Return a reference to the embedding.
    pub fn embedding(&self) -> &E {
        &self.embedding
    }
}

impl<E> Embedding for EspGatt<E>
where
    E: Embedding,
{
    const INIT: Self = Self::new();

    fn init() -> impl Init<Self> {
        EspGatt::init()
    }
}

/// The GATT application ID of the BTP service. A Bluedroid notion; NimBLE has no equivalent.
#[cfg(esp_idf_bt_bluedroid_enabled)]
const GATTS_APP_ID: u16 = 0;
