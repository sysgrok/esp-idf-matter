//! This module provides the ESP-IDF Thread implementation of the Matter `NetCtl`, `NetChangeNotif`, `WirelessDiag`, and `ThreadDiag` traits.

use core::cell::{Cell, RefCell};
use core::fmt::Write;
#[cfg(all(esp_idf_openthread_dns_client, not(esp_idf_version_major = "4")))]
use core::net::IpAddr;
#[cfg(all(esp_idf_openthread_dns_client, not(esp_idf_version_major = "4")))]
use core::pin::pin;

use alloc::sync::Arc;

#[cfg(all(esp_idf_openthread_dns_client, not(esp_idf_version_major = "4")))]
use embassy_futures::select::{select, select3};
use embassy_sync;
use embassy_sync::blocking_mutex::{self, raw::CriticalSectionRawMutex};
use embassy_sync::mutex::Mutex;

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::task::embassy_sync::EspRawMutex;
use esp_idf_svc::netif::EspNetif;
use esp_idf_svc::sys::{esp, esp_mac_type_t_ESP_MAC_IEEE802154, esp_read_mac, EspError};
use esp_idf_svc::thread::{
    ActiveScanResult, EspThread, NetifMode, Role, SrpConf, SrpService, SrpServiceSlot,
};

use log::{debug, error, info, warn};

use rs_matter_stack::matter::crypto::Crypto;
use rs_matter_stack::matter::dm::clusters::gen_diag::InterfaceTypeEnum;
use rs_matter_stack::matter::dm::clusters::net_comm::{
    NetCtl, NetCtlError, NetworkScanInfo, NetworkType, WirelessCreds,
};
use rs_matter_stack::matter::dm::clusters::thread_diag::{
    MacCounters, MleCounters, NeighborTable, NetworkFaultEnum, OperationalDatasetComponents,
    RouteTable, RoutingRoleEnum, SecurityPolicy, ThreadDiag,
};
use rs_matter_stack::matter::dm::clusters::wifi_diag::WirelessDiag;
use rs_matter_stack::matter::dm::networks::NetChangeNotif;
use rs_matter_stack::matter::error::{Error, ErrorCode};
use rs_matter_stack::matter::fabric::MAX_FABRICS;
#[cfg(all(esp_idf_openthread_dns_client, not(esp_idf_version_major = "4")))]
use rs_matter_stack::matter::transport::network::mdns::{DottedName, MdnsRemoteService, MdnsTxt};
use rs_matter_stack::matter::transport::network::MatterLocalService;
#[cfg(all(esp_idf_openthread_dns_client, not(esp_idf_version_major = "4")))]
use rs_matter_stack::matter::transport::network::MatterRemoteService;
#[cfg(all(esp_idf_openthread_dns_client, not(esp_idf_version_major = "4")))]
use rs_matter_stack::matter::transport::MdnsResponder;
use rs_matter_stack::matter::utils::init::{init, Init};
#[cfg(all(esp_idf_openthread_dns_client, not(esp_idf_version_major = "4")))]
use rs_matter_stack::matter::utils::select::Coalesce;
use rs_matter_stack::matter::utils::storage::Vec;
use rs_matter_stack::matter::Matter;
use rs_matter_stack::mdns::Mdns;

use crate::error::to_net_error;
use crate::netif::{self, EspNetifAccess};

extern crate alloc;

const OT_MDNS_BUF_SZ: usize = 256;

/// This type provides the ESP-IDF Thread implementation of the Matter `NetCtl`, `NetChangeNotif`, `WirelessDiag`, and `ThreadDiag` traits
pub struct EspMatterThreadCtl<'a, 'd, M>
where
    M: NetifMode,
{
    thread: Mutex<EspRawMutex, &'a EspThread<'d, M>>,
    // Unlocked access for the (sync) diagnostics, which only do short OpenThread reads
    diag: &'a EspThread<'d, M>,
    operational: blocking_mutex::Mutex<EspRawMutex, Cell<bool>>,
    sysloop: EspSystemEventLoop,
}

impl<'a, 'd, M> EspMatterThreadCtl<'a, 'd, M>
where
    M: NetifMode,
{
    /// Create a new instance of the `EspMatterThreadCtl` type.
    pub fn new(thread: &'a EspThread<'d, M>, sysloop: EspSystemEventLoop) -> Self {
        Self {
            thread: Mutex::new(thread),
            diag: thread,
            operational: blocking_mutex::Mutex::new(Cell::new(false)),
            sysloop,
        }
    }

    /// Fetch from the underlying Thread interface whether it is operational (i.e. connected and has IPv6 addresses).
    fn fetch_is_operational<MM: NetifMode>(thread: &EspThread<MM>) -> Result<bool, EspError> {
        let netif = thread.netif();
        let l2_connected = Self::fetch_is_thread_connected(thread).unwrap_or(false);

        netif::utils::get_netif_conf(netif, InterfaceTypeEnum::Thread, |info| {
            Ok(netif::utils::info_is_operational_v6(l2_connected, info))
        })
    }

    /// Fetch from the underlying Thread interface whether it is connected at L2 (i.e. not detached or disabled).
    fn fetch_is_thread_connected<MM: NetifMode>(
        thread: &EspThread<'_, MM>,
    ) -> Result<bool, EspError> {
        Ok(!matches!(thread.role()?, Role::Detached | Role::Disabled))
    }

    /// Return `None` without calling `f` if the node has no Active Operational Dataset.
    fn if_commissioned<T>(
        &self,
        f: impl FnOnce(&EspThread<'d, M>) -> Result<T, EspError>,
    ) -> Result<Option<T>, Error> {
        if !self.is_commissioned()? {
            return Ok(None);
        }

        f(self.diag).map(Some).map_err(to_net_error)
    }

    fn is_commissioned(&self) -> Result<bool, Error> {
        self.diag.is_commissioned().map_err(to_net_error)
    }

    /// Map the `Child` device role to the Matter routing role.
    fn child_routing_role(&self) -> Result<RoutingRoleEnum, EspError> {
        if !self.diag.rx_on_when_idle()? {
            return Ok(RoutingRoleEnum::SleepyEndDevice);
        }

        #[cfg(esp_idf_openthread_ftd)]
        if self.diag.router_eligible()? {
            return Ok(RoutingRoleEnum::REED);
        }

        Ok(RoutingRoleEnum::EndDevice)
    }
}

impl<M> NetCtl for EspMatterThreadCtl<'_, '_, M>
where
    M: NetifMode,
{
    fn net_type(&self) -> NetworkType {
        NetworkType::Thread
    }

    async fn scan<F>(&self, network: Option<&[u8]>, mut f: F) -> Result<(), NetCtlError>
    where
        F: FnMut(&NetworkScanInfo) -> Result<(), Error>,
    {
        const POLL_SCAN_WAIT: embassy_time::Duration = embassy_time::Duration::from_millis(500);

        let thread = self.thread.lock().await;

        struct OwnedScanResult {
            pan_id: u16,
            ext_pan_id: u64,
            network_name: heapless::String<16>,
            channel: u16,
            version: u8,
            ext_addr: [u8; 8],
            rssi: i8,
            lqi: u8,
        }

        impl From<ActiveScanResult<'_>> for OwnedScanResult {
            fn from(result: ActiveScanResult<'_>) -> Self {
                Self {
                    pan_id: result.pan_id(),
                    ext_pan_id: u64::from_be_bytes(result.extended_pan_id().try_into().unwrap()),
                    network_name: result
                        .network_name_cstr()
                        .to_str()
                        .unwrap_or("???")
                        .try_into()
                        .unwrap_or("???".try_into().unwrap()),
                    channel: result.channel() as _,
                    version: result.version(),
                    ext_addr: result.extended_address().try_into().unwrap(),
                    rssi: result.max_rssi(),
                    lqi: result.lqi(),
                }
            }
        }

        impl<'a> From<&'a OwnedScanResult> for NetworkScanInfo<'a> {
            fn from(result: &'a OwnedScanResult) -> Self {
                NetworkScanInfo::Thread {
                    pan_id: result.pan_id,
                    ext_pan_id: result.ext_pan_id,
                    network_name: result.network_name.as_str(),
                    channel: result.channel,
                    version: result.version,
                    ext_addr: &result.ext_addr,
                    rssi: result.rssi,
                    lqi: result.lqi,
                }
            }
        }

        let scan_result = Arc::new(blocking_mutex::Mutex::<CriticalSectionRawMutex, _>::new(
            RefCell::new(Some(heapless::Vec::<_, 5>::new())),
        ));

        {
            let scan_result = scan_result.clone();

            thread
                .scan(move |info: Option<ActiveScanResult<'_>>| {
                    if let Some(info) = info {
                        scan_result.lock(|results| {
                            let mut results = results.borrow_mut();

                            if let Some(results) = results.as_mut() {
                                results.push(OwnedScanResult::from(info)).ok();
                            }
                        });
                    }
                })
                .map_err(to_net_error)?;
        }

        loop {
            if !thread.is_scan_in_progress().map_err(to_net_error)? {
                break;
            }

            embassy_time::Timer::after(POLL_SCAN_WAIT).await;
        }

        let results = scan_result
            .lock(|results| results.borrow_mut().take())
            .unwrap();

        for result in results {
            if network
                .map(|network| result.ext_pan_id.to_be_bytes() == network)
                .unwrap_or(true)
            {
                f(&NetworkScanInfo::Thread {
                    pan_id: result.pan_id,
                    ext_pan_id: result.ext_pan_id,
                    network_name: result.network_name.as_str(),
                    channel: result.channel,
                    version: result.version,
                    ext_addr: &result.ext_addr,
                    rssi: result.rssi,
                    lqi: result.lqi,
                })?;
            }
        }

        Ok(())
    }

    async fn connect(&self, creds: &WirelessCreds<'_>) -> Result<(), NetCtlError> {
        const CONNECT_WAIT: embassy_time::Duration = embassy_time::Duration::from_millis(30000);
        const POLL_CONNECT_WAIT: embassy_time::Duration = embassy_time::Duration::from_millis(1000);

        let WirelessCreds::Thread { dataset_tlv } = creds else {
            return Err(NetCtlError::Other(ErrorCode::InvalidData.into()));
        };

        let thread = self.thread.lock().await;

        // Apply the new dataset with Thread stopped and only then bring it back up.
        // Swapping the Active Operational Dataset underneath an attached stack leaves
        // OpenThread re-attaching to the *old* partition for a while, and the SRP
        // client bound to the old network's server.
        let _ = thread.enable_thread(false);

        thread.set_tod(dataset_tlv).map_err(to_net_constr_error)?;

        thread.enable_thread(true).map_err(to_net_error)?;

        let connect_attempt_time = embassy_time::Instant::now();

        let result = loop {
            let operational = Self::fetch_is_operational(&thread).map_err(to_net_error)?;

            if operational {
                break Ok(());
            }

            if connect_attempt_time.elapsed() > CONNECT_WAIT {
                break Err(NetCtlError::AuthFailure);
            }

            embassy_time::Timer::after(POLL_CONNECT_WAIT).await;
        };

        match result {
            Ok(()) => {
                // TODO: Disconnect Thread?
                self.operational.lock(|operational| {
                    info!(
                        "Thread operational state updated: {} -> {}",
                        operational.get(),
                        true
                    );

                    operational.set(true)
                });

                Ok(())
            }
            Err(e) => {
                // Don't leave the stack churning on a network we failed to join
                let _ = thread.enable_thread(false);

                Err(e)
            }
        }
    }
}

impl<M> NetChangeNotif for EspMatterThreadCtl<'_, '_, M>
where
    M: NetifMode,
{
    async fn wait_changed(&self) {
        let fetch_operational = || async {
            let thread = self.thread.lock().await;

            let new_operational = Self::fetch_is_operational(&thread).unwrap_or(false);
            self.operational.lock(|operational| {
                if operational.get() != new_operational {
                    warn!(
                        "Thread operational state changed: {} -> {}",
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

impl<M> WirelessDiag for EspMatterThreadCtl<'_, '_, M>
where
    M: NetifMode,
{
    fn connected(&self) -> Result<bool, Error> {
        Ok(self.operational.lock(|operational| operational.get()))
    }
}

impl<M> ThreadDiag for EspMatterThreadCtl<'_, '_, M>
where
    M: NetifMode,
{
    fn channel(&self) -> Result<Option<u16>, Error> {
        self.if_commissioned(|thread| thread.channel().map(u16::from))
    }

    fn routing_role(&self) -> Result<Option<RoutingRoleEnum>, Error> {
        // Not null even when not commissioned: OpenThread reports a valid role
        let role = match self.diag.role().map_err(to_net_error)? {
            Role::Disabled => RoutingRoleEnum::Unspecified,
            Role::Detached => RoutingRoleEnum::Unassigned,
            Role::Child => self.child_routing_role().map_err(to_net_error)?,
            Role::Router => RoutingRoleEnum::Router,
            Role::Leader => RoutingRoleEnum::Leader,
        };

        Ok(Some(role))
    }

    fn network_name(
        &self,
        f: &mut dyn FnMut(Option<&str>) -> Result<(), Error>,
    ) -> Result<(), Error> {
        if self.is_commissioned()? {
            self.diag
                .network_name(|name| f(Some(name)))
                .map_err(to_net_error)?
        } else {
            f(None)
        }
    }

    fn pan_id(&self) -> Result<Option<u16>, Error> {
        self.if_commissioned(|thread| thread.pan_id())
    }

    fn extended_pan_id(&self) -> Result<Option<u64>, Error> {
        self.if_commissioned(|thread| thread.extended_pan_id())
    }

    fn mesh_local_prefix(
        &self,
        f: &mut dyn FnMut(Option<&[u8]>) -> Result<(), Error>,
    ) -> Result<(), Error> {
        let prefix = self.if_commissioned(|thread| thread.mesh_local_prefix())?;

        if let Some(prefix) = prefix {
            // Prefix length in bits, followed by the prefix itself
            let mut buf = [0; 9];
            buf[0] = 64;
            buf[1..].copy_from_slice(&prefix);

            f(Some(&buf))
        } else {
            f(None)
        }
    }

    fn neighbor_table(
        &self,
        f: &mut dyn FnMut(&NeighborTable) -> Result<(), Error>,
    ) -> Result<(), Error> {
        let mut result = Ok(());

        self.diag
            .neighbor_table(|info| {
                if result.is_ok() {
                    result = f(&NeighborTable {
                        ext_address: info.ext_address,
                        age: info.age,
                        rloc16: info.rloc16,
                        link_frame_counter: info.link_frame_counter,
                        mle_frame_counter: info.mle_frame_counter,
                        lqi: info.link_quality_in,
                        average_rssi: info.average_rssi,
                        last_rssi: info.last_rssi,
                        frame_error_rate: info.frame_error_rate,
                        message_error_rate: info.message_error_rate,
                        rx_on_when_idle: info.rx_on_when_idle,
                        full_thread_device: info.full_thread_device,
                        full_network_data: info.full_network_data,
                        is_child: info.is_child,
                    });
                }
            })
            .map_err(to_net_error)?;

        result
    }

    fn route_table(
        &self,
        f: &mut dyn FnMut(&RouteTable) -> Result<(), Error>,
    ) -> Result<(), Error> {
        let mut result = Ok(());

        self.diag
            .route_table(|info| {
                if result.is_ok() {
                    result = f(&RouteTable {
                        ext_address: info.ext_address,
                        rloc16: info.rloc16,
                        router_id: info.router_id,
                        next_hop: info.next_hop,
                        path_cost: info.path_cost,
                        lqi_in: info.lqi_in,
                        lqi_out: info.lqi_out,
                        age: info.age,
                        allocated: info.allocated,
                        link_established: info.link_established,
                    });
                }
            })
            .map_err(to_net_error)?;

        result
    }

    fn partition_id(&self) -> Result<Option<u32>, Error> {
        self.if_commissioned(|thread| thread.partition_id())
    }

    fn weighting(&self) -> Result<Option<u16>, Error> {
        self.if_commissioned(|thread| thread.leader_weight().map(u16::from))
    }

    fn data_version(&self) -> Result<Option<u16>, Error> {
        self.if_commissioned(|thread| thread.net_data_version().map(u16::from))
    }

    fn stable_data_version(&self) -> Result<Option<u16>, Error> {
        self.if_commissioned(|thread| thread.net_data_stable_version().map(u16::from))
    }

    fn leader_router_id(&self) -> Result<Option<u8>, Error> {
        self.if_commissioned(|thread| thread.leader_router_id())
    }

    fn ext_address(&self) -> Result<Option<u64>, Error> {
        self.if_commissioned(|thread| thread.ext_address())
    }

    fn rloc_16(&self) -> Result<Option<u16>, Error> {
        self.if_commissioned(|thread| thread.rloc16())
    }

    fn security_policy(&self) -> Result<Option<SecurityPolicy>, Error> {
        let policy = self
            .diag
            .active_dataset_diag()
            .map_err(to_net_error)?
            .and_then(|diag| diag.security_policy);

        Ok(policy.map(|policy| SecurityPolicy {
            rotation_time: policy.rotation_time,
            flags: security_policy_flags(&policy),
        }))
    }

    fn channel_page0_mask(
        &self,
        f: &mut dyn FnMut(Option<&[u8]>) -> Result<(), Error>,
    ) -> Result<(), Error> {
        let mask = self
            .diag
            .active_dataset_diag()
            .map_err(to_net_error)?
            .and_then(|diag| diag.channel_mask);

        if let Some(mask) = mask {
            // Matter wants the lowest channel in the MSB of the first byte
            f(Some(&mask.reverse_bits().to_be_bytes()))
        } else {
            f(None)
        }
    }

    fn operational_dataset_components(
        &self,
        f: &mut dyn FnMut(Option<&OperationalDatasetComponents>) -> Result<(), Error>,
    ) -> Result<(), Error> {
        let Some(diag) = self.diag.active_dataset_diag().map_err(to_net_error)? else {
            return f(None);
        };

        let c = diag.components;

        f(Some(&OperationalDatasetComponents {
            active_timestamp_present: c.active_timestamp_present,
            pending_timestamp_present: c.pending_timestamp_present,
            master_key_present: c.network_key_present,
            network_name_present: c.network_name_present,
            extended_pan_id_present: c.extended_pan_id_present,
            mesh_local_prefix_present: c.mesh_local_prefix_present,
            delay_present: c.delay_present,
            pan_id_present: c.pan_id_present,
            channel_present: c.channel_present,
            pskc_present: c.pskc_present,
            security_policy_present: c.security_policy_present,
            channel_mask_present: c.channel_mask_present,
        }))
    }

    fn active_network_faults_list(
        &self,
        _f: &mut dyn FnMut(NetworkFaultEnum) -> Result<(), Error>,
    ) -> Result<(), Error> {
        // OpenThread does not track these
        Ok(())
    }

    fn mac_counters(
        &self,
        f: &mut dyn FnMut(Option<&MacCounters>) -> Result<(), Error>,
    ) -> Result<(), Error> {
        let c = self.diag.mac_counters().map_err(to_net_error)?;

        f(Some(&MacCounters {
            tx_total_count: c.tx_total,
            tx_unicast_count: c.tx_unicast,
            tx_broadcast_count: c.tx_broadcast,
            tx_ack_requested_count: c.tx_ack_requested,
            tx_acked_count: c.tx_acked,
            tx_no_ack_requested_count: c.tx_no_ack_requested,
            tx_data_count: c.tx_data,
            tx_data_poll_count: c.tx_data_poll,
            tx_beacon_count: c.tx_beacon,
            tx_beacon_request_count: c.tx_beacon_request,
            tx_other_count: c.tx_other,
            tx_retry_count: c.tx_retry,
            tx_direct_max_retry_expiry_count: Some(c.tx_direct_max_retry_expiry),
            tx_indirect_max_retry_expiry_count: Some(c.tx_indirect_max_retry_expiry),
            tx_err_cca_count: c.tx_err_cca,
            tx_err_abort_count: c.tx_err_abort,
            tx_err_busy_channel_count: c.tx_err_busy_channel,
            rx_total_count: c.rx_total,
            rx_unicast_count: c.rx_unicast,
            rx_broadcast_count: c.rx_broadcast,
            rx_data_count: c.rx_data,
            rx_data_poll_count: c.rx_data_poll,
            rx_beacon_count: c.rx_beacon,
            rx_beacon_request_count: c.rx_beacon_request,
            rx_other_count: c.rx_other,
            rx_address_filtered_count: c.rx_address_filtered,
            rx_dest_addr_filtered_count: c.rx_dest_addr_filtered,
            rx_duplicated_count: c.rx_duplicated,
            rx_err_no_frame_count: c.rx_err_no_frame,
            rx_err_unknown_neighbor_count: c.rx_err_unknown_neighbor,
            rx_err_invalid_src_addr_count: c.rx_err_invalid_src_addr,
            rx_err_sec_count: c.rx_err_sec,
            rx_err_fcs_count: c.rx_err_fcs,
            rx_err_other_count: c.rx_err_other,
        }))
    }

    fn mle_counters(
        &self,
        f: &mut dyn FnMut(Option<&MleCounters>) -> Result<(), Error>,
    ) -> Result<(), Error> {
        let c = self.diag.mle_counters().map_err(to_net_error)?;

        f(Some(&MleCounters {
            detached_role_count: c.detached_role,
            child_role_count: c.child_role,
            router_role_count: c.router_role,
            leader_role_count: c.leader_role,
            attach_attempt_count: c.attach_attempts,
            partition_id_change_count: c.partition_id_changes,
            better_partition_attach_attempt_count: c.better_partition_attach_attempts,
            parent_change_count: c.parent_changes,
        }))
    }

    fn active_timestamp(&self) -> Result<Option<u64>, Error> {
        Ok(self
            .diag
            .active_dataset_diag()
            .map_err(to_net_error)?
            .and_then(|diag| diag.active_timestamp)
            .map(|timestamp| dataset_timestamp(&timestamp)))
    }

    fn pending_timestamp(&self) -> Result<Option<u64>, Error> {
        Ok(self
            .diag
            .pending_dataset_diag()
            .map_err(to_net_error)?
            .and_then(|diag| diag.pending_timestamp)
            .map(|timestamp| dataset_timestamp(&timestamp)))
    }

    fn delay(&self) -> Result<Option<u32>, Error> {
        Ok(self
            .diag
            .pending_dataset_diag()
            .map_err(to_net_error)?
            .and_then(|diag| diag.delay))
    }
}

impl<T: NetifMode> EspNetifAccess for EspMatterThreadCtl<'_, '_, T> {
    async fn access<F, R>(&self, f: F) -> Result<R, EspError>
    where
        F: FnOnce(&EspNetif, bool) -> Result<R, EspError>,
    {
        let thread = self.thread.lock().await;

        f(
            thread.netif(),
            Self::fetch_is_thread_connected(&thread).unwrap_or(false),
        )
    }

    fn hw_addr(&self) -> Option<[u8; 8]> {
        self.diag.ext_address().ok().map(u64::to_be_bytes)
    }
}

const MAX_MATTER_SERVICES: usize = MAX_FABRICS + 1;

/// An mDNS implementation for Thread: registers the Matter services over SRP,
/// and resolves and browses those of others with the DNS client (when the
/// OpenThread DNS client is enabled).
pub struct EspMatterThreadSrp<'a, 'd, M>
where
    M: NetifMode,
{
    thread: &'a EspThread<'d, M>,
    host_eui64: Option<[u8; 8]>,
    services: Vec<(MatterLocalService, SrpServiceSlot), MAX_MATTER_SERVICES>,
    mdns_buf: Vec<u8, OT_MDNS_BUF_SZ>,
}

impl<'a, 'd, M> EspMatterThreadSrp<'a, 'd, M>
where
    M: NetifMode,
{
    /// Create a new instance of the `EspMatterThreadSrp` type,
    /// deriving the SRP host name from the factory-programmed IEEE 802.15.4 address.
    pub fn new(thread: &'a EspThread<'d, M>) -> Self {
        Self::new_with_host_eui64(thread, None)
    }

    /// Create a new instance of the `EspMatterThreadSrp` type.
    ///
    /// # Arguments
    /// - `host_eui64`: The EUI-64 to derive the SRP host name from, or `None` to use the
    ///   factory-programmed IEEE 802.15.4 address (see [`EspMatterThreadSrp::new`]).
    pub fn new_with_host_eui64(thread: &'a EspThread<'d, M>, host_eui64: Option<[u8; 8]>) -> Self {
        Self {
            thread,
            host_eui64,
            services: Vec::new(),
            mdns_buf: Vec::new(),
        }
    }

    /// Create a new instance of the `EspMatterThreadSrp` type,
    /// deriving the SRP host name from the factory-programmed IEEE 802.15.4 address.
    pub fn init(thread: &'a EspThread<'d, M>) -> impl Init<Self> {
        Self::init_with_host_eui64(thread, None)
    }

    /// Create a new instance of the `EspMatterThreadSrp` type.
    ///
    /// # Arguments
    /// - `host_eui64`: The EUI-64 to derive the SRP host name from, or `None` to use the
    ///   factory-programmed IEEE 802.15.4 address (see [`EspMatterThreadSrp::init`]).
    pub fn init_with_host_eui64(
        thread: &'a EspThread<'d, M>,
        host_eui64: Option<[u8; 8]>,
    ) -> impl Init<Self> {
        init!(Self {
            thread,
            host_eui64,
            services <- Vec::init(),
            mdns_buf <- Vec::init(),
        })
    }

    pub async fn run(
        &mut self,
        matter: &Matter<'_>,
        _ipv6: &[core::net::Ipv6Addr],
    ) -> Result<(), Error> {
        let ieee_eui64 = match self.host_eui64 {
            Some(eui64) => eui64,
            None => {
                let mut eui64 = [0; 8];
                esp!(unsafe {
                    esp_read_mac(eui64.as_mut_ptr(), esp_mac_type_t_ESP_MAC_IEEE802154)
                })
                .map_err(to_net_error)?;

                eui64
            }
        };

        let mut hostname = heapless::String::<16>::new();
        write!(
            hostname,
            "{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}",
            ieee_eui64[0],
            ieee_eui64[1],
            ieee_eui64[2],
            ieee_eui64[3],
            ieee_eui64[4],
            ieee_eui64[5],
            ieee_eui64[6],
            ieee_eui64[7]
        )
        .unwrap();

        // This method is (re)started by the Matter stack every time the Thread netif
        // changes, which includes the device moving to a *different* Thread network
        // (i.e. a new Operational Dataset applied during commissioning). The SRP
        // server of the new network knows nothing about what was registered before,
        // so start from a clean slate and re-register the host and all services.
        //
        // The reset is an *immediate* clear rather than a graceful, server-side
        // removal: a graceful removal only completes once the server acks the host
        // into the `Removed` state, which never happens for records registered with a
        // different server (or with no server at all). Re-registering with the same
        // (persisted) ECDSA key overwrites whatever a previous registration left on
        // the server; records on servers we are no longer talking to expire by lease.
        self.thread.srp_remove_all(true).map_err(to_net_error)?;
        self.services.clear();

        self.thread
            .srp_set_conf(&SrpConf {
                host_name: &hostname,
                host_addrs: &[],
                ..Default::default()
            })
            .map_err(to_net_error)?;

        info!("SRP hostname set to '{hostname}'");

        #[cfg(all(esp_idf_openthread_dns_client, not(esp_idf_version_major = "4")))]
        {
            let thread = self.thread;

            let mut register = pin!(self.run_register(matter));
            let mut serve = pin!(Self::run_serve(thread, matter));

            select(&mut register, &mut serve).coalesce().await
        }

        #[cfg(not(all(esp_idf_openthread_dns_client, not(esp_idf_version_major = "4"))))]
        {
            self.run_register(matter).await
        }
    }

    /// Register the Matter services with the SRP server, again whenever they change.
    async fn run_register(&mut self, matter: &Matter<'_>) -> Result<(), Error> {
        loop {
            let mut services = Vec::<_, MAX_MATTER_SERVICES>::new();
            matter.mdns_services(|service| {
                if services.push(service).is_err() {
                    error!("Too many mDNS services registered, max is {MAX_MATTER_SERVICES}");

                    Err(ErrorCode::ConstraintError)?;
                }

                Ok(())
            })?;

            debug!("mDNS services changed, updating...");

            self.update_services(matter, &services)?;

            debug!("mDNS services updated");

            self.log_srp_state();

            matter.transport().wait_mdns().await;
        }
    }

    /// Serve the resolve and browse requests while the SRP client has a
    /// server: that is the one the DNS client queries.
    #[cfg(all(esp_idf_openthread_dns_client, not(esp_idf_version_major = "4")))]
    async fn run_serve(thread: &EspThread<'_, M>, matter: &Matter<'_>) -> Result<(), Error> {
        loop {
            Self::wait_srp_server(thread, true).await?;

            let responder = matter.transport().mdns_responder();

            let mut resolve = pin!(Self::run_resolve(thread, &responder, matter));
            let mut browse = pin!(Self::run_browse(thread, &responder, matter));
            let mut lost = pin!(Self::wait_srp_server(thread, false));

            select3(&mut resolve, &mut browse, &mut lost)
                .coalesce()
                .await?;
        }
    }

    /// Wait until the SRP client has a server (`present`), or no longer has one.
    #[cfg(all(esp_idf_openthread_dns_client, not(esp_idf_version_major = "4")))]
    async fn wait_srp_server(thread: &EspThread<'_, M>, present: bool) -> Result<(), Error> {
        while thread.srp_server_addr().map_err(to_net_error)?.is_some() != present {
            thread.srp_wait_changed().await;
        }

        Ok(())
    }

    /// Resolve the requested Matter services with the DNS client.
    #[cfg(all(esp_idf_openthread_dns_client, not(esp_idf_version_major = "4")))]
    async fn run_resolve(
        thread: &EspThread<'_, M>,
        responder: &MdnsResponder<'_>,
        matter: &Matter<'_>,
    ) -> Result<(), Error> {
        loop {
            let service = responder.wait_resolve_request().await;

            // The DNS-SD server of the Border Router serves the Thread domain
            // `default.service.arpa`, not mDNS `local`
            let mut label = heapless::String::<33>::new();
            let service_name = match service {
                MatterRemoteService::Operational {
                    compressed_fabric_id,
                    node_id,
                } => {
                    let _ = write!(label, "{compressed_fabric_id:016X}-{node_id:016X}");
                    "_matter._tcp.default.service.arpa"
                }
                MatterRemoteService::Commissionable { id } => {
                    let _ = write!(label, "{id:016X}");
                    "_matterc._udp.default.service.arpa"
                }
            };

            // ... while `rs-matter` matches answers by their mDNS `local` name
            let mut instance_name = heapless::String::<128>::new();
            service.instance_name(&mut instance_name);

            match thread
                .dns_resolve_service_and_host_address(&label, service_name)
                .await
            {
                Ok(info) => matter.transport().try_deposit_mdns_resolve(
                    &MdnsRemoteService {
                        instance_name: DottedName(instance_name.as_str()),
                        port: Some(info.port),
                        addrs: info.host_addresses.iter().copied().map(IpAddr::V6),
                        txt: MdnsTxt::new(&info.txt_data),
                        scope_id: 0,
                    },
                    // Resolved via the SRP server: no on-link vs off-link scoring
                    &[],
                ),
                Err(e) => warn!("mDNS resolve query failed: {e:?}"),
            }
        }
    }

    /// Browse the requested Matter services with the DNS client.
    #[cfg(all(esp_idf_openthread_dns_client, not(esp_idf_version_major = "4")))]
    async fn run_browse(
        thread: &EspThread<'_, M>,
        responder: &MdnsResponder<'_>,
        matter: &Matter<'_>,
    ) -> Result<(), Error> {
        loop {
            let query = responder.wait_browse_request().await;

            let mut service_type = heapless::String::<64>::new();
            query.service_type(&mut service_type, false);

            let mut service_name = heapless::String::<96>::new();
            let _ = write!(service_name, "{service_type}.default.service.arpa");

            let instances = match thread.dns_browse(&service_name).await {
                Ok(instances) => instances,
                Err(e) => {
                    warn!("mDNS browse query failed: {e:?}");
                    continue;
                }
            };

            for instance in &instances {
                let info = instance.info.as_ref();

                if query.needs_resolve() && info.is_none() {
                    continue;
                }

                let mut instance_name = heapless::String::<96>::new();
                let _ = write!(
                    instance_name,
                    "{}.{}.local",
                    instance.label,
                    query.base_service_type()
                );

                matter
                    .transport()
                    .try_deposit_mdns_browse(&MdnsRemoteService {
                        instance_name: DottedName(instance_name.as_str()),
                        port: info.map(|info| info.port),
                        addrs: info
                            .into_iter()
                            .flat_map(|info| info.host_addresses.iter().copied())
                            .map(IpAddr::V6),
                        txt: MdnsTxt::new(info.map_or(&[][..], |info| info.txt_data.as_slice())),
                        scope_id: 0,
                    });
            }
        }
    }

    fn update_services(
        &mut self,
        matter: &Matter,
        services: &[MatterLocalService],
    ) -> Result<(), Error> {
        // Nothing to do unless the set actually changed
        if services.len() == self.services.len()
            && services
                .iter()
                .all(|service| self.services.iter().any(|(s, _)| s == service))
        {
            return Ok(());
        }

        // Re-register *every* service that should stay published, together with the
        // removals, so that all of it goes out as a single, self-contained SRP update.
        //
        // OpenThread's SRP client would otherwise send a partial update - the removals
        // only - because it relies on the server merging back the services the update did
        // not mention. Its own SRP server does exactly that, but a border router is free
        // to read an SRP update as the complete description of that host's services and
        // republish accordingly, which withdraws every record the update left out. The
        // client keeps reporting those records as `Registered` while they are no longer
        // discoverable, so the node silently goes unreachable.
        let mut registered = Vec::<_, MAX_MATTER_SERVICES>::new();
        for entry in &self.services {
            // `unwrap` cannot fail: same capacity, and we are draining `self.services`
            registered.push(entry.clone()).unwrap();
        }

        self.services.clear();

        for (service, slot) in &registered {
            if services.contains(service) {
                // Drop the local registration only - re-added below, into the same update
                self.thread
                    .srp_remove_service(*slot, true)
                    .map_err(to_net_error)?;
            } else {
                info!("Deregistering mDNS service: {service:?}");
                self.deregister(*slot)?;
            }
        }

        for service in services {
            info!("Registering mDNS service: {service:?}");

            let slot = self.register(matter, service)?;
            if self.services.push((service.clone(), slot)).is_err() {
                error!("Too many mDNS services registered, max is {MAX_MATTER_SERVICES}");

                Err(ErrorCode::ConstraintError)?;
            }
        }

        Ok(())
    }

    fn register(
        &mut self,
        matter: &Matter,
        service: &MatterLocalService,
    ) -> Result<SrpServiceSlot, Error> {
        self.mdns_buf.resize_default(OT_MDNS_BUF_SZ).unwrap();

        let (service, _) = service.service(matter, &mut self.mdns_buf)?;
        let service = core::mem::ManuallyDrop::new(service);

        // TODO:
        // Remove `ManuallyDrop` by removing the `'a` lifetime from the signature of the function below:
        // pub fn srp_add_service<'a, SI, TI>(&self, service: &'a SrpService<'a, SI, TI>)
        //                                                      ^- remove this lifetime

        let srp_service = core::mem::ManuallyDrop::new(SrpService {
            name: service.service_protocol,
            instance_name: service.name,
            port: service.port,
            subtype_labels: service.service_subtypes.clone(),
            txt_entries: service.txt_kvs.clone().map(|(k, v)| (k, v.as_bytes())),
            priority: 0,
            weight: 0,
            lease_secs: 0,
            key_lease_secs: 0,
        });

        self.thread
            .srp_add_service(&srp_service)
            .map_err(to_net_error)
    }

    fn deregister(&mut self, slot: SrpServiceSlot) -> Result<(), Error> {
        self.thread
            .srp_remove_service(slot, false)
            .map_err(to_net_error)?;

        Ok(())
    }

    /// Log what the SRP client itself believes about the host and each registered service.
    fn log_srp_state(&self) {
        let running = self.thread.srp_running().unwrap_or(false);

        let _ = self.thread.srp_conf(|conf, state, _| {
            debug!(
                "SRP state: client running={running}, host '{}' is {state}",
                conf.host_name
            );

            Ok(())
        });

        let _ = self.thread.srp_services(|service| {
            if let Some((service, state, slot)) = service {
                debug!(
                    "SRP state: slot {slot}: '{}' / '{}' is {state}",
                    service.instance_name, service.name
                );
            }
        });
    }
}

impl<M> Mdns for EspMatterThreadSrp<'_, '_, M>
where
    M: NetifMode,
{
    async fn run<C, U>(
        &mut self,
        matter: &Matter<'_>,
        _crypto: C,
        _udp: U,
        _mac: &[u8],
        _ipv4: core::net::Ipv4Addr,
        ipv6: &[core::net::Ipv6Addr],
        _interface: u32,
    ) -> Result<(), Error>
    where
        C: Crypto,
        U: edge_nal::UdpBind,
    {
        Self::run(self, matter, ipv6).await
    }
}

/// A Thread dataset timestamp as the 64-bit value of its TLV: seconds, ticks and the authoritative bit
fn dataset_timestamp(timestamp: &esp_idf_svc::thread::Timestamp) -> u64 {
    (timestamp.seconds << 16) | ((timestamp.ticks as u64) << 1) | timestamp.authoritative as u64
}

/// The Matter `SecurityPolicy` flags bitmap
fn security_policy_flags(policy: &esp_idf_svc::thread::SecurityPolicy) -> u16 {
    [
        policy.obtain_network_key_enabled,
        policy.native_commissioning_enabled,
        policy.routers_enabled,
        policy.external_commissioning_enabled,
        policy.commercial_commissioning_enabled,
        policy.autonomous_enrollment_enabled,
        policy.network_key_provisioning_enabled,
        policy.toble_link_enabled,
        policy.non_ccm_routers_enabled,
    ]
    .into_iter()
    .enumerate()
    .fold(0, |flags, (bit, set)| flags | ((set as u16) << bit))
        | ((policy.version_threshold_for_routing as u16 & 0x7) << 9)
}

fn to_net_constr_error<E>(_err: E) -> NetCtlError {
    NetCtlError::Other(ErrorCode::ConstraintError.into())
}
