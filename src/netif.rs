//! This module provides the ESP-IDF implementation of the `Netif` trait for the Matter stack, as
//! well as the `EspMatterNetStack` type alias for a STD stack which is based on `async-io` or `async-io-mini`.

use core::borrow::Borrow;
use core::future::Future;
use core::net::{Ipv4Addr, Ipv6Addr};

use embassy_sync::blocking_mutex;

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::task::embassy_sync::EspRawMutex;
use esp_idf_svc::netif::EspNetif;
use esp_idf_svc::sys::EspError;

use rs_matter_stack::matter::dm::clusters::gen_diag::{InterfaceTypeEnum, NetifDiag, NetifInfo};
use rs_matter_stack::matter::dm::networks::NetChangeNotif;
use rs_matter_stack::matter::error::Error;
use rs_matter_stack::matter::utils::cell::RefCell;

/// A network stack for ESP-IDF
pub type EspMatterNetStack = edge_nal_std::Stack;

/// A `Netif` trait implementation for ESP-IDF
pub struct EspMatterNetif<T> {
    netif_access: T,
    sysloop: EspSystemEventLoop,
    netif_type: InterfaceTypeEnum,
    netif_state: blocking_mutex::Mutex<EspRawMutex, RefCell<NetifInfoOwned>>,
}

impl<T> EspMatterNetif<T>
where
    T: EspNetifAccess,
{
    /// Create a new `EspMatterNetif` instance
    pub const fn new(
        netif_access: T,
        netif_type: InterfaceTypeEnum,
        sysloop: EspSystemEventLoop,
    ) -> Self {
        Self {
            netif_access,
            netif_type,
            sysloop,
            netif_state: blocking_mutex::Mutex::new(RefCell::new(NetifInfoOwned::new())),
        }
    }

    fn load_netif_state(&self, l2_connected: bool, netif: &EspNetif) -> Result<bool, EspError> {
        self.netif_state.lock(|state| {
            state
                .borrow_mut()
                .load(l2_connected, netif, self.netif_type)
        })
    }
}

impl<T> NetifDiag for EspMatterNetif<T> {
    fn netifs(&self, f: &mut dyn FnMut(&NetifInfo) -> Result<(), Error>) -> Result<(), Error> {
        self.netif_state.lock(|info| info.borrow().as_ref(f))
    }
}

impl<T> NetChangeNotif for EspMatterNetif<T>
where
    T: EspNetifAccess,
{
    async fn wait_changed(&self) {
        loop {
            let changed = self
                .netif_access
                .access(|netif, l2_connected| self.load_netif_state(l2_connected, netif))
                .await
                .unwrap_or(false);

            if changed {
                break;
            }

            let _ = utils::wait_any_conf_change(&self.sysloop).await;
        }
    }
}

/// A trait to abstract the way how `EspMatterNotif` gets access
/// to the `EspNetif` instance associated with the concrete network protocol (Ethernet, Wifi or Thread)
pub trait EspNetifAccess {
    /// Access the `EspNetif` instance
    ///
    /// # Arguments
    /// - `f`: A closure which is called with the `EspNetif` instance and a boolean indicating whether the underlying L2 protocol
    ///   is connected (e.g. Wifi is connected to an AP, Ethernet cable is plugged in, Thread is attached to a Thread network)
    async fn access<F, R>(&self, f: F) -> Result<R, EspError>
    where
        F: FnOnce(&EspNetif, bool) -> Result<R, EspError>;
}

impl<T> EspNetifAccess for &T
where
    T: EspNetifAccess,
{
    fn access<F, R>(&self, f: F) -> impl Future<Output = Result<R, EspError>>
    where
        F: FnOnce(&EspNetif, bool) -> Result<R, EspError>,
    {
        (*self).access(f)
    }
}

impl EspNetifAccess for &EspNetif {
    async fn access<F, R>(&self, f: F) -> Result<R, EspError>
    where
        F: FnOnce(&EspNetif, bool) -> Result<R, EspError>,
    {
        f(self.borrow(), true)
    }
}

/// A cache type for storing the information for one network interface
///
/// Necessary, because the `NetifDiag` trait is not async
#[allow(dead_code)]
#[derive(Debug)]
pub(crate) struct NetifInfoOwned {
    name: heapless::String<6>,
    operational: bool,
    hw_addr: [u8; 8],
    ipv4_addr: Ipv4Addr,
    ipv6_addr: Ipv6Addr,
    netif_type: InterfaceTypeEnum,
    netif_index: u32,
}

#[allow(dead_code)]
impl NetifInfoOwned {
    pub(crate) const fn new() -> Self {
        Self {
            name: heapless::String::new(),
            operational: false,
            hw_addr: [0; 8],
            ipv4_addr: Ipv4Addr::UNSPECIFIED,
            ipv6_addr: Ipv6Addr::UNSPECIFIED,
            netif_type: InterfaceTypeEnum::WiFi,
            netif_index: 0,
        }
    }

    pub(crate) fn is_operational(&self) -> bool {
        self.is_operational_v6() && !self.ipv4_addr.is_unspecified()
    }

    pub(crate) fn is_operational_v6(&self) -> bool {
        self.operational && !self.ipv6_addr.is_unspecified()
    }

    pub(crate) fn as_ref<F>(&self, f: F) -> Result<(), Error>
    where
        F: FnOnce(&NetifInfo<'_>) -> Result<(), Error>,
    {
        let ipv4_addrs = [self.ipv4_addr];
        let ipv6_addrs = [self.ipv6_addr];

        f(&NetifInfo {
            name: &self.name,
            operational: self.operational,
            hw_addr: &self.hw_addr,
            ipv4_addrs: if self.ipv4_addr.is_unspecified() {
                &[]
            } else {
                &ipv4_addrs
            },
            ipv6_addrs: if self.ipv6_addr.is_unspecified() {
                &[]
            } else {
                &ipv6_addrs
            },
            netif_type: self.netif_type,
            offprem_svc_reachable_ipv4: None,
            offprem_svc_reachable_ipv6: None,
            netif_index: self.netif_index,
        })
    }

    pub(crate) fn load(
        &mut self,
        l2_connected: bool,
        netif: &EspNetif,
        netif_type: InterfaceTypeEnum,
    ) -> Result<bool, EspError> {
        utils::get_netif_conf(netif, netif_type, |info| {
            Ok(self.load_from_info(l2_connected, info))
        })
    }

    fn load_from_info(&mut self, l2_connected: bool, info: &NetifInfo<'_>) -> bool {
        let hw_addr: &[u8] = info.hw_addr;

        let ipv4_addr = utils::info_ipv4_addr(info);
        let ipv6_addr = utils::info_ipv6_addr(info);

        let changed = self.name != info.name
            || self.operational != info.operational && l2_connected
            || self.hw_addr != hw_addr
            || self.ipv4_addr != ipv4_addr
            || self.ipv6_addr != ipv6_addr
            || self.netif_type != info.netif_type
            || self.netif_index != info.netif_index;

        if changed {
            self.name = info.name.try_into().unwrap();
            self.operational = info.operational && l2_connected;
            self.hw_addr = hw_addr.try_into().unwrap();
            self.ipv4_addr = ipv4_addr;
            self.ipv6_addr = ipv6_addr;
            self.netif_type = info.netif_type;
            self.netif_index = info.netif_index;
        }

        changed
    }
}

/// Utility functions for working with the ESP-IDF `EspNetif` type
pub mod utils {
    use core::net::{Ipv4Addr, Ipv6Addr};
    use core::pin::pin;

    use alloc::sync::Arc;

    use embassy_futures::select::select;
    use embassy_time::{Duration, Timer};

    use esp_idf_svc::eventloop::EspSystemEventLoop;
    use esp_idf_svc::hal::task::embassy_sync::EspRawMutex;
    use esp_idf_svc::handle::RawHandle;
    use esp_idf_svc::netif::{EspNetif, IpEvent};
    use esp_idf_svc::sys::{
        esp_ip6_addr_t, esp_netif_get_all_ip6, EspError, LWIP_IPV6_NUM_ADDRESSES,
    };

    use rs_matter_stack::matter::dm::clusters::gen_diag::{InterfaceTypeEnum, NetifInfo};
    use rs_matter_stack::matter::utils::sync::Notification;

    extern crate alloc;

    /// Get the network interface configuration as a `NetifInfo` structure
    pub fn get_netif_conf<F, R>(
        netif: &EspNetif,
        netif_type: InterfaceTypeEnum,
        f: F,
    ) -> Result<R, EspError>
    where
        F: FnOnce(&NetifInfo) -> Result<R, EspError>,
    {
        let ip_info = netif.get_ip_info()?;

        let ipv4: Ipv4Addr = ip_info.ip.octets().into();

        let ipv6 = {
            let mut ipv6: [esp_ip6_addr_t; LWIP_IPV6_NUM_ADDRESSES as usize] = Default::default();
            let count = unsafe { esp_netif_get_all_ip6(netif.handle() as _, ipv6.as_mut_ptr()) };

            if count > 0 {
                let ipv6 = &ipv6[count as usize - 1];

                [
                    ipv6.addr[0].to_le_bytes()[0],
                    ipv6.addr[0].to_le_bytes()[1],
                    ipv6.addr[0].to_le_bytes()[2],
                    ipv6.addr[0].to_le_bytes()[3],
                    ipv6.addr[1].to_le_bytes()[0],
                    ipv6.addr[1].to_le_bytes()[1],
                    ipv6.addr[1].to_le_bytes()[2],
                    ipv6.addr[1].to_le_bytes()[3],
                    ipv6.addr[2].to_le_bytes()[0],
                    ipv6.addr[2].to_le_bytes()[1],
                    ipv6.addr[2].to_le_bytes()[2],
                    ipv6.addr[2].to_le_bytes()[3],
                    ipv6.addr[3].to_le_bytes()[0],
                    ipv6.addr[3].to_le_bytes()[1],
                    ipv6.addr[3].to_le_bytes()[2],
                    ipv6.addr[3].to_le_bytes()[3],
                ]
                .into()
            } else {
                Ipv6Addr::UNSPECIFIED
            }
        };

        let mut mac: [u8; 8] = Default::default();
        mac[..6].copy_from_slice(&netif.get_mac()?);

        f(&NetifInfo {
            name: &netif.get_name(),
            operational: if matches!(netif_type, InterfaceTypeEnum::Thread) {
                netif.is_netif_up()?
            } else {
                netif.is_up()?
            },
            offprem_svc_reachable_ipv4: None,
            offprem_svc_reachable_ipv6: None,
            hw_addr: &mac,
            ipv4_addrs: &[ipv4],
            ipv6_addrs: &[ipv6],
            netif_type,
            netif_index: netif.get_index(),
        })
    }

    pub fn info_is_operational(l2_connected: bool, info: &NetifInfo<'_>) -> bool {
        info_is_operational_v6(l2_connected, info) && !info_ipv4_addr(info).is_unspecified()
    }

    pub fn info_is_operational_v6(l2_connected: bool, info: &NetifInfo<'_>) -> bool {
        l2_connected && info.operational && !info_ipv6_addr(info).is_unspecified()
    }

    /// Wait for any IP configuration change
    pub async fn wait_any_conf_change(sysloop: &EspSystemEventLoop) -> Result<(), EspError> {
        const TIMEOUT_PERIOD_SECS: u8 = 5;

        let notification = Arc::new(Notification::<EspRawMutex>::new());

        let _subscription = {
            let notification = notification.clone();

            sysloop.subscribe::<IpEvent, _>(move |_| {
                notification.notify();
            })
        }?;

        let mut events = pin!(notification.wait());
        let mut timer = pin!(Timer::after(Duration::from_secs(TIMEOUT_PERIOD_SECS as _)));

        select(&mut events, &mut timer).await;

        Ok(())
    }

    pub(crate) fn info_ipv4_addr(info: &NetifInfo<'_>) -> Ipv4Addr {
        info.ipv4_addrs
            .first()
            .copied()
            .unwrap_or(Ipv4Addr::UNSPECIFIED)
    }

    pub(crate) fn info_ipv6_addr(info: &NetifInfo<'_>) -> Ipv6Addr {
        let ipv6_addr = if matches!(info.netif_type, InterfaceTypeEnum::Thread) {
            // For Thread: return the first Ipv6 address
            // Does not really matter what is returned, as the Ipv6 Thread address
            // returned here is FYI only, and is not used for opening the Matter stack
            // or for mDNS-over-Thread (SRP)
            info.ipv6_addrs.first()
        } else {
            // For Wifi: locate the link-local Ipv6 address
            info.ipv6_addrs
                .iter()
                .find(|ipv6| ipv6.is_unicast_link_local())
        };

        ipv6_addr.copied().unwrap_or(Ipv6Addr::UNSPECIFIED)
    }
}
