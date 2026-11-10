# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

- Breaking: no stack aliases; a stack is a plain `MatterStack` over `EspThreadNetwork`, `EspWifiNetwork` or `EspAnyNetwork` (a networks store plus the BLE GATT context), as in `MatterStack<'a, B, EspThreadNetwork, E>`, or over `EthNetwork` for Ethernet. Each takes a store of its network type. `EspMatterThread` and `EspMatterWifi` take a stack whose network is `EspThreadCapable` or `EspWifiCapable`: their own network type, or `EspAnyNetwork`. `EspWirelessMatterStack`, `EspWirelessBle`, `EspThreadMatterStack`, `EspWifiMatterStack`, `EspEthMatterStack`, `EspEth` and `EspGatt` are gone
- New: `EspMatterAny`, a driver for Ethernet, Wifi or Thread, whichever the node runs, for the stack's `run`, `run_no_coex` and `run_oper`: an `AnyNetDriver` of the ESP-IDF drivers
- The stacks that commission over BLE need `BTP_BUMP_SIZE` more in their bump arena, as the examples show, since `rs_matter::stack` keeps the BTP transport there
- New: ESP32-S31 support; the Wifi and Thread drivers split the modem as `esp-idf-hal` does on the S31, C5, C6 and C61
- Fix: the Thread SRP host name comes from the Thread extended address, not the factory EUI-64, so that the SRP server does not refuse a node whose flash was erased for up to 14 days
- Breaking: move to `rs_matter::stack`, in place of the `rs-matter-stack` crate; `rs-matter-stack`, `edge-nal` and `edge-nal-std` are gone, and `EspMatterNetStack` is a type of its own
- Breaking: update to the `rs-matter` split into crates; `root_handler` takes the `Matter` instance, as per `rs-matter-stack`
- Raise MSRV to 1.92, as per `rs-matter`
- Use `rs-matter`'s `utils::futures`, `utils::timers`, `utils::storage` and `utils::sync` instead of `embassy-futures`, `embassy-time`, `heapless` and the `embassy-sync` mutexes
- Fix: a Thread ICD's child timeout follows its slowest polling interval as the ICD sets it, not as it was when Thread came up, so that the parent keeps a LIT child between its polls
- New: `EspMatterThread::with_icd`, running as a Sleepy End Device that follows the ICD power mode
- New: examples `trv_battery_thread` (SIT ICD, light sleep) and `temp_sensor_battery_thread` (LIT ICD, deep sleep)
- `EspMatterThreadSrp` resolves and browses Matter services with the OpenThread DNS client, while the SRP client has a server; before, nothing over Thread could be resolved (e.g. ICD Check-In clients, subscribers to reconnect to)
- Thread and Wi-Fi Network Diagnostics report the actual network state, incl. the Thread MLE and MAC counters and dataset timestamps
- New: `EspMatterEthDiag`, Ethernet Network Diagnostics (PHY rate, duplex, carrier detect) over an `EthDriver`
- New: `EspSwDiag`, Software Diagnostics with heap usage, watermarks and the FreeRTOS tasks
- The Wi-Fi and Thread stacks take the networks store type as a parameter, `S`, after `E` (`EspWifiMatterStack<'a, B, E, S>`, `EspThreadMatterStack<'a, B, E, S>`), as per `rs-matter-stack`; by default a `WirelessNetworks` with `MAX_WIRELESS_NETWORKS` networks

## [0.1.0] - 2026-09-15

- Initial release
