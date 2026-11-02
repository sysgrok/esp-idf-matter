//! A `NetStack` over the standard UDP sockets, with either `async-io` or `async-io-mini`.

use core::future::{ready, Future};
use core::net::{IpAddr, Ipv4Addr, SocketAddr};

use std::io;
use std::net::UdpSocket;
use std::os::fd::AsRawFd;

#[cfg(not(feature = "async-io-mini"))]
use async_io::Async;
#[cfg(feature = "async-io-mini")]
use async_io_mini::Async;

use esp_idf_svc::sys;

use rs_matter::error::{Error, ErrorCode};
use rs_matter::stack::net::{NetSocket, NetStack};
use rs_matter::transport::network::{Address, NetworkMulticast, NetworkReceive, NetworkSend};
use rs_matter::utils::sync::MaybeSend;

/// A network stack for ESP-IDF
#[derive(Debug, Default, Clone, Copy)]
pub struct EspMatterNetStack(());

impl EspMatterNetStack {
    /// Create a new instance of the `EspMatterNetStack` type.
    pub const fn new() -> Self {
        Self(())
    }
}

impl NetStack for EspMatterNetStack {
    type Socket<'a>
        = EspMatterNetSocket
    where
        Self: 'a;

    fn bind(
        &self,
        addr: SocketAddr,
        ipv4_interface: Ipv4Addr,
        ipv6_interface: u32,
    ) -> impl Future<Output = Result<Self::Socket<'_>, Error>> + MaybeSend {
        ready(EspMatterNetSocket::bind(
            &addr,
            ipv4_interface,
            ipv6_interface,
        ))
    }
}

/// The UDP socket of `EspMatterNetStack`.
pub struct EspMatterNetSocket {
    socket: Async<UdpSocket>,
    ipv4_interface: Ipv4Addr,
    ipv6_interface: u32,
}

impl EspMatterNetSocket {
    fn bind(
        addr: &SocketAddr,
        ipv4_interface: Ipv4Addr,
        ipv6_interface: u32,
    ) -> Result<Self, Error> {
        let socket = Async::<UdpSocket>::bind(*addr)?;

        socket.as_ref().set_broadcast(true)?;

        Ok(Self {
            socket,
            ipv4_interface,
            ipv6_interface,
        })
    }

    fn set_ipv4_membership(&self, option: u32, addr: Ipv4Addr) -> Result<(), io::Error> {
        // `UdpSocket::join_multicast_v4` and `leave_multicast_v4` do not work on ESP-IDF, as
        // the `libc` crate has wrong values for `IP_ADD_MEMBERSHIP` and `IP_DROP_MEMBERSHIP`
        let mreq = sys::ip_mreq {
            imr_multiaddr: sys::in_addr {
                s_addr: u32::from_ne_bytes(addr.octets()),
            },
            imr_interface: sys::in_addr {
                s_addr: u32::from_ne_bytes(self.ipv4_interface.octets()),
            },
        };

        let result = unsafe {
            sys::lwip_setsockopt(
                self.socket.as_raw_fd(),
                sys::IPPROTO_IP as _,
                option as _,
                &mreq as *const _ as *const _,
                core::mem::size_of::<sys::ip_mreq>() as _,
            )
        };

        if result == -1 {
            Err(io::Error::last_os_error())
        } else {
            Ok(())
        }
    }
}

impl NetSocket for EspMatterNetSocket {
    type Send<'a>
        = EspMatterNetSocketRef<'a>
    where
        Self: 'a;

    type Receive<'a>
        = EspMatterNetSocketRef<'a>
    where
        Self: 'a;

    type Multicast<'a>
        = EspMatterNetSocketRef<'a>
    where
        Self: 'a;

    fn split(&mut self) -> (Self::Send<'_>, Self::Receive<'_>, Self::Multicast<'_>) {
        (
            EspMatterNetSocketRef(self),
            EspMatterNetSocketRef(self),
            EspMatterNetSocketRef(self),
        )
    }
}

/// One of the halves of a split `EspMatterNetSocket`.
pub struct EspMatterNetSocketRef<'a>(&'a EspMatterNetSocket);

impl NetworkSend for EspMatterNetSocketRef<'_> {
    async fn send_to(&mut self, data: &[u8], addr: Address) -> Result<(), Error> {
        let addr = addr.udp().ok_or(ErrorCode::NoNetworkInterface)?;

        self.0.socket.send_to(data, addr).await?;

        Ok(())
    }
}

impl NetworkReceive for EspMatterNetSocketRef<'_> {
    async fn wait_available(&mut self) -> Result<(), Error> {
        self.0.socket.readable().await?;

        Ok(())
    }

    async fn recv_from(&mut self, buffer: &mut [u8]) -> Result<(usize, Address), Error> {
        let (len, addr) = self.0.socket.recv_from(buffer).await?;

        Ok((len, Address::Udp(addr)))
    }
}

impl NetworkMulticast for EspMatterNetSocketRef<'_> {
    async fn join(&mut self, addr: IpAddr) -> Result<(), Error> {
        match addr {
            IpAddr::V6(addr) => self
                .0
                .socket
                .as_ref()
                .join_multicast_v6(&addr, self.0.ipv6_interface)?,
            IpAddr::V4(addr) => self.0.set_ipv4_membership(sys::IP_ADD_MEMBERSHIP, addr)?,
        }

        Ok(())
    }

    async fn leave(&mut self, addr: IpAddr) -> Result<(), Error> {
        match addr {
            IpAddr::V6(addr) => self
                .0
                .socket
                .as_ref()
                .leave_multicast_v6(&addr, self.0.ipv6_interface)?,
            IpAddr::V4(addr) => self.0.set_ipv4_membership(sys::IP_DROP_MEMBERSHIP, addr)?,
        }

        Ok(())
    }
}
