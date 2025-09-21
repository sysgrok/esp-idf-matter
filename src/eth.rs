//! This module provides the ESP-IDF implementation of the Ethernet Matter stack.

use rs_matter_stack::eth::Eth;
use rs_matter_stack::MatterStack;

/// A type alias for an ESP-IDF Matter stack running over an Ethernet network (or any other network not managed by Matter).
pub type EspEthMatterStack<'a, const B: usize, E> = MatterStack<'a, B, EspEth<E>>;

/// A type alias for an ESP-IDF implementation of the `Network` trait for a Matter stack running over
/// an Ethernet network (or any other network not managed by Matter).
pub type EspEth<E> = Eth<E>;
