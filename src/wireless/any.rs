//! A driver for Ethernet, Wifi or Thread, whichever the node runs.

use core::marker::PhantomData;

use rs_matter::dm::networks::any::AnyNetworks;
use rs_matter::stack::any::AnyNetDriver;
use rs_matter::stack::eth::NoEthernet;
use rs_matter::stack::network::{AnyStackNetwork, StackNetwork};

use crate::ble::EspBtpGattContext;

use super::{EspMatterThread, EspMatterWifi, EspThreadCapable, EspWifiCapable};

/// The network of a `MatterStack` that runs Ethernet, Wifi or Thread with [`EspMatterAny`]:
/// the networks store `S` (`AnyNetworks` by default), and the context of the BLE GATT
/// peripheral that Wifi and Thread commission over.
pub struct EspAnyNetwork<S = AnyNetworks>(PhantomData<fn() -> S>);

impl<S> StackNetwork for EspAnyNetwork<S>
where
    S: AnyStackNetwork<Context = ()>,
{
    type Store = S::Store;
    type Context = EspBtpGattContext;
}

impl<S> AnyStackNetwork for EspAnyNetwork<S> where S: AnyStackNetwork<Context = ()> {}

impl<S> EspThreadCapable for EspAnyNetwork<S> where S: AnyStackNetwork<Context = ()> {}

impl<S> EspWifiCapable for EspAnyNetwork<S> where S: AnyStackNetwork<Context = ()> {}

/// A driver for Ethernet (`E`), Wifi or Thread, whichever the node runs; with BLE
/// alongside Wifi and Thread.
///
/// For the stack's `run`, `run_no_coex` and `run_oper`, whose data model handler is
/// compiled once for all three networks. `E` is `NoEthernet` for a node without
/// Ethernet.
pub type EspMatterAny<'a, 'd, E = NoEthernet> =
    AnyNetDriver<E, EspMatterWifi<'a, 'd>, EspMatterThread<'a, 'd>>;
