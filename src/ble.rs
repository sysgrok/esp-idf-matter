//! BTP GATT peripheral support, used by Matter for commissioning over BLE.
//!
//! Which backend is compiled depends on the Bluetooth host selected in `sdkconfig`:
//! - Bluedroid, when `CONFIG_BT_BLUEDROID_ENABLED=y`
//! - NimBLE, when `CONFIG_BT_NIMBLE_ENABLED=y`
//!
//! Both expose the same `EspBtpGattContext` and `EspBtpGattPeripheral` types, so the rest of
//! the crate does not care which one is in use. Their constructors differ, however, as NimBLE
//! initializes the controller and the host in one step and therefore needs no `BtDriver`.
//!
//! NimBLE has a substantially smaller RAM footprint than Bluedroid, which is what makes BLE
//! coexist with Thread on the RAM-constrained `esp32h2` and `esp32c6`.

#[cfg(esp_idf_bt_bluedroid_enabled)]
mod bluedroid;
#[cfg(esp_idf_bt_bluedroid_enabled)]
pub use bluedroid::*;

// The two hosts are mutually exclusive in `Kconfig`; the `not(...)` guard only keeps the
// module tree unambiguous if that ever stops being true.
#[cfg(all(esp_idf_bt_nimble_enabled, not(esp_idf_bt_bluedroid_enabled)))]
mod nimble;
#[cfg(all(esp_idf_bt_nimble_enabled, not(esp_idf_bt_bluedroid_enabled)))]
pub use nimble::*;
