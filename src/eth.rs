//! This module provides the ESP-IDF implementation of the Ethernet Matter stack.

use rs_matter_stack::eth::Eth;
use rs_matter_stack::MatterStack;

#[cfg(all(esp_idf_comp_esp_eth_enabled, esp_idf_comp_esp_event_enabled))]
#[cfg(any(
    esp_idf_eth_use_esp32_emac,
    esp_idf_eth_spi_ethernet_dm9051,
    esp_idf_eth_spi_ethernet_w5500,
    esp_idf_eth_spi_ethernet_ksz8851snl,
    esp_idf_comp_espressif__dm9051_enabled,
    esp_idf_comp_espressif__w5500_enabled,
    esp_idf_comp_espressif__ksz8851snl_enabled,
    esp_idf_eth_use_openeth,
))]
pub use diag::*;

// Same as the cfg of `esp_idf_svc::eth`
#[cfg(all(esp_idf_comp_esp_eth_enabled, esp_idf_comp_esp_event_enabled))]
#[cfg(any(
    esp_idf_eth_use_esp32_emac,
    esp_idf_eth_spi_ethernet_dm9051,
    esp_idf_eth_spi_ethernet_w5500,
    esp_idf_eth_spi_ethernet_ksz8851snl,
    esp_idf_comp_espressif__dm9051_enabled,
    esp_idf_comp_espressif__w5500_enabled,
    esp_idf_comp_espressif__ksz8851snl_enabled,
    esp_idf_eth_use_openeth,
))]
mod diag {
    use esp_idf_svc::eth::{Duplex, EthDriver, Speed};

    use rs_matter_stack::matter::dm::clusters::eth_diag::{EthDiag, PHYRateEnum};
    use rs_matter_stack::matter::error::Error;

    use crate::error::to_net_error;

    /// The ESP-IDF implementation of the Matter `EthDiag` trait, over an Ethernet driver.
    ///
    /// Advertise what it reports with `eth_diag(phy_rate, full_duplex, carrier_detect)`
    /// on the root endpoint.
    pub struct EspMatterEthDiag<'a, 'd, T>(&'a EthDriver<'d, T>);

    impl<'a, 'd, T> EspMatterEthDiag<'a, 'd, T> {
        /// Create a new instance of the `EspMatterEthDiag` type.
        pub const fn new(driver: &'a EthDriver<'d, T>) -> Self {
            Self(driver)
        }
    }

    impl<T> EthDiag for EspMatterEthDiag<'_, '_, T> {
        fn phy_rate(&self) -> Result<Option<PHYRateEnum>, Error> {
            if !self.0.is_connected().map_err(to_net_error)? {
                return Ok(None);
            }

            Ok(match self.0.get_speed() {
                Ok(Speed::Mbps10) => Some(PHYRateEnum::Rate10M),
                Ok(Speed::Mbps100) => Some(PHYRateEnum::Rate100M),
                #[cfg(esp_idf_version_at_least_6_1_0)]
                Ok(Speed::Mbps1000) => Some(PHYRateEnum::Rate1G),
                Err(_) => None,
            })
        }

        fn full_duplex(&self) -> Result<Option<bool>, Error> {
            if !self.0.is_connected().map_err(to_net_error)? {
                return Ok(None);
            }

            Ok(self
                .0
                .get_duplex()
                .ok()
                .map(|duplex| duplex == Duplex::Full))
        }

        fn carrier_detect(&self) -> Result<Option<bool>, Error> {
            // Not configured while the driver is stopped
            if !self.0.is_started().map_err(to_net_error)? {
                return Ok(None);
            }

            Ok(Some(self.0.is_connected().map_err(to_net_error)?))
        }
    }
}

/// A type alias for an ESP-IDF Matter stack running over an Ethernet network (or any other network not managed by Matter).
pub type EspEthMatterStack<'a, const B: usize, E> = MatterStack<'a, B, EspEth<E>>;

/// A type alias for an ESP-IDF implementation of the `Network` trait for a Matter stack running over
/// an Ethernet network (or any other network not managed by Matter).
pub type EspEth<E> = Eth<E>;
