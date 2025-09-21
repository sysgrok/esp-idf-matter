//! This module provides the ESP-IDF Wifi implementation of the Matter `NetCtl`, `NetChangeNotif`, `WirelessDiag`, and `WifiDiag` traits.

use core::cell::Cell;
use core::time::Duration;

extern crate alloc;

use embassy_sync::blocking_mutex;
use embassy_sync::mutex::Mutex;

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::task::embassy_sync::EspRawMutex;
use esp_idf_svc::handle::RawHandle as _;
use esp_idf_svc::netif::EspNetif;
use esp_idf_svc::sys::{esp, EspError};
use esp_idf_svc::wifi::{
    AsyncWifi, AuthMethod, ClientConfiguration, Configuration::Client, EspWifi,
};

use log::{info, warn};
use rs_matter_stack::matter::dm::clusters::decl::general_diagnostics::InterfaceTypeEnum;
use rs_matter_stack::matter::dm::clusters::net_comm::{
    NetCtl, NetCtlError, NetworkScanInfo, NetworkType, WirelessCreds,
};
use rs_matter_stack::matter::dm::clusters::net_comm::{WiFiBandEnum, WiFiSecurityBitmap};
use rs_matter_stack::matter::dm::clusters::wifi_diag::{
    SecurityTypeEnum, WiFiVersionEnum, WifiDiag, WirelessDiag,
};
use rs_matter_stack::matter::dm::networks::NetChangeNotif;
use rs_matter_stack::matter::error::{Error, ErrorCode};
use rs_matter_stack::matter::tlv::Nullable;

use crate::error::to_net_error;
use crate::netif::{self, EspNetifAccess};

/// This type provides the ESP-IDF Wifi implementation of the Matter `NetCtl`, `NetChangeNotif`, `WirelessDiag`, and `WifiDiag` traits
pub struct EspMatterWifiCtl<'a> {
    wifi: Mutex<EspRawMutex, AsyncWifi<EspWifi<'a>>>,
    operational: blocking_mutex::Mutex<EspRawMutex, Cell<bool>>,
    sysloop: EspSystemEventLoop,
}

impl<'a> EspMatterWifiCtl<'a> {
    /// Create a new instance of the `EspMatterWifiCtl` type.
    pub const fn new(wifi: AsyncWifi<EspWifi<'a>>, sysloop: EspSystemEventLoop) -> Self {
        Self {
            wifi: Mutex::new(wifi),
            operational: blocking_mutex::Mutex::new(Cell::new(false)),
            sysloop,
        }
    }

    /// Fetch whether the Wifi interface is operational (i.e. connected and has IPv6 and Ipv4 addresses).
    fn fetch_is_operational(wifi: &EspWifi<'_>) -> Result<bool, EspError> {
        let netif = wifi.sta_netif();
        let l2_connected = wifi.is_connected().unwrap_or(false);

        netif::utils::get_netif_conf(netif, InterfaceTypeEnum::WiFi, |info| {
            Ok(netif::utils::info_is_operational(l2_connected, info))
        })
    }
}

impl NetCtl for EspMatterWifiCtl<'_> {
    fn net_type(&self) -> NetworkType {
        NetworkType::Wifi
    }

    async fn scan<F>(&self, network: Option<&[u8]>, mut f: F) -> Result<(), NetCtlError>
    where
        F: FnMut(&NetworkScanInfo) -> Result<(), Error>,
    {
        let mut wifi = self.wifi.lock().await;

        if !wifi.is_started().map_err(to_net_error)? {
            wifi.start().await.map_err(to_net_error)?;
        }

        for ap in wifi.scan().await.map_err(to_net_error)? {
            if network
                .map(|network| ap.ssid.as_bytes() == network)
                .unwrap_or(true)
            {
                f(&NetworkScanInfo::Wifi {
                    security: if let Some(auth_method) = ap.auth_method {
                        match auth_method {
                            AuthMethod::None => WiFiSecurityBitmap::UNENCRYPTED,
                            AuthMethod::WEP => WiFiSecurityBitmap::WEP,
                            AuthMethod::WPA => WiFiSecurityBitmap::WPA_PERSONAL,
                            AuthMethod::WPA2Personal => WiFiSecurityBitmap::WPA_2_PERSONAL,
                            AuthMethod::WPAWPA2Personal => {
                                WiFiSecurityBitmap::WPA_PERSONAL
                                    | WiFiSecurityBitmap::WPA_2_PERSONAL
                            }
                            AuthMethod::WPA3Personal => WiFiSecurityBitmap::WPA_3_PERSONAL,
                            AuthMethod::WPA2WPA3Personal => {
                                WiFiSecurityBitmap::WPA_2_PERSONAL
                                    | WiFiSecurityBitmap::WPA_3_PERSONAL
                            }
                            _ => WiFiSecurityBitmap::empty(),
                        }
                    } else {
                        WiFiSecurityBitmap::empty()
                    },
                    ssid: ap.ssid.as_bytes(),
                    bssid: &ap.bssid,
                    channel: ap.channel as _,
                    band: WiFiBandEnum::V2G4,
                    rssi: ap.signal_strength,
                })?;
            }
        }

        Ok(())
    }

    async fn connect(&self, creds: &WirelessCreds<'_>) -> Result<(), NetCtlError> {
        let WirelessCreds::Wifi { ssid, pass } = creds else {
            return Err(NetCtlError::Other(ErrorCode::InvalidData.into()));
        };

        let mut wifi = self.wifi.lock().await;

        let mut result = Ok(());

        let mut conf = Client(ClientConfiguration {
            ssid: core::str::from_utf8(ssid)
                .map_err(to_net_constr_error)?
                .try_into()
                .map_err(to_net_constr_error)?,
            password: core::str::from_utf8(pass)
                .map_err(to_net_constr_error)?
                .try_into()
                .map_err(to_net_constr_error)?,
            auth_method: AuthMethod::None,
            ..Default::default()
        });

        for auth_method in [
            AuthMethod::WPA2Personal,
            AuthMethod::WPA,
            AuthMethod::WPA2WPA3Personal,
            AuthMethod::WEP,
        ] {
            let _ = wifi.disconnect().await;

            conf.as_client_conf_mut().auth_method = auth_method;
            wifi.set_configuration(&conf).map_err(to_net_error)?;

            if !wifi.is_started().map_err(to_net_error)? {
                wifi.start().await.map_err(to_net_error)?;
            }

            result = wifi
                .connect()
                .await
                .map_err(|_| NetCtlError::OtherConnectionFailure);

            if result.is_ok() {
                break;
            }
        }

        result?;

        // Matter needs an IPv6 address to work
        esp!(unsafe {
            esp_idf_svc::sys::esp_netif_create_ip6_linklocal(wifi.wifi().sta_netif().handle() as _)
        })
        .map_err(to_net_error)?;

        // Wait not just for the wireless interface to come up, but also for the
        // IP addresses to be assigned.
        let result = wifi
            .ip_wait_while(
                |wifi| Self::fetch_is_operational(wifi.wifi()).map(|op| !op),
                Some(Duration::from_secs(20)),
            )
            .await
            .map_err(|_| NetCtlError::IpBindFailed);

        match result {
            Ok(()) => {
                self.operational.lock(|operational| {
                    info!(
                        "Wifi operational state updated: {} -> {}",
                        operational.get(),
                        true
                    );

                    operational.set(true)
                });

                Ok(())
            }
            Err(e) => {
                let _ = wifi.disconnect().await;

                Err(e)
            }
        }
    }
}

impl NetChangeNotif for EspMatterWifiCtl<'_> {
    async fn wait_changed(&self) {
        let fetch_operational = || async {
            let wifi = self.wifi.lock().await;
            let new_operational = Self::fetch_is_operational(wifi.wifi()).unwrap_or(false);

            self.operational.lock(|operational| {
                if operational.get() != new_operational {
                    warn!(
                        "Wifi operational state changed: {} -> {}",
                        operational.get(),
                        new_operational
                    );

                    operational.set(new_operational);

                    true
                } else {
                    false
                }
            })
        };

        loop {
            if fetch_operational().await {
                break;
            }

            let _ = netif::utils::wait_any_conf_change(&self.sysloop).await;
        }
    }
}

impl WirelessDiag for EspMatterWifiCtl<'_> {
    fn connected(&self) -> Result<bool, Error> {
        Ok(self.operational.lock(|operational| operational.get()))
    }
}

// TODO
impl WifiDiag for EspMatterWifiCtl<'_> {
    fn bssid(&self, f: &mut dyn FnMut(Option<&[u8]>) -> Result<(), Error>) -> Result<(), Error> {
        f(None)
    }

    fn security_type(&self) -> Result<Nullable<SecurityTypeEnum>, Error> {
        Ok(Nullable::none())
    }

    fn wi_fi_version(&self) -> Result<Nullable<WiFiVersionEnum>, Error> {
        Ok(Nullable::none())
    }

    fn channel_number(&self) -> Result<Nullable<u16>, Error> {
        Ok(Nullable::none())
    }

    fn rssi(&self) -> Result<Nullable<i8>, Error> {
        Ok(Nullable::none())
    }
}

impl EspNetifAccess for EspMatterWifiCtl<'_> {
    async fn access<F, R>(&self, f: F) -> Result<R, EspError>
    where
        F: FnOnce(&EspNetif, bool) -> Result<R, EspError>,
    {
        let wifi = self.wifi.lock().await;

        f(
            wifi.wifi().sta_netif(),
            wifi.is_connected().unwrap_or(false),
        )
    }
}

fn to_net_constr_error<E>(_err: E) -> NetCtlError {
    NetCtlError::Other(ErrorCode::ConstraintError.into())
}
