//! This module provides the ESP-IDF implementation of the Wifi and Thread Matter stacks.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

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

#[cfg(esp_idf_comp_esp_wifi_enabled)]
pub use wifi::*;

#[cfg(all(
    esp_idf_comp_openthread_enabled,
    esp_idf_openthread_enabled,
    esp_idf_soc_ieee802154_supported,
    esp_idf_comp_vfs_enabled,
))]
mod thread;

#[cfg(esp_idf_comp_esp_wifi_enabled)]
mod wifi;

/// A type alias for an ESP-IDF Matter stack running over a wireless network (Wifi or Thread) and BLE.
pub type EspWirelessMatterStack<'a, const B: usize, T, E> =
    MatterStack<'a, B, EspWirelessBle<T, E>>;

/// A type alias for an ESP-IDF implementation of the `Network` trait for a Matter stack running over
/// BLE during commissioning, and then over either WiFi or Thread when operating.
// TODO: Revert to `EspRawMutex` when `esp-idf-svc` is updated to `embassy-sync 0.7`
pub type EspWirelessBle<T, E> =
    WirelessBle<CriticalSectionRawMutex /*EspRawMutex*/, T, EspGatt<E>>;

/// An embedding of the ESP IDF Bluedroid Gatt peripheral context for the `WirelessBle` network type from `rs-matter-stack`.
///
/// Allows the memory of this context to be statically allocated and cost-initialized.
///
/// Usage:
/// ```no_run
/// MatterStack<WirelessBle<EspRawMutex, Wifi, KvBlobBuf<EspGatt<E>>>>::new(...);
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

const GATTS_APP_ID: u16 = 0;
