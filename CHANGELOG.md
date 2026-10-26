# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

- New: `EspMatterThread::with_icd`, running as a Sleepy End Device that follows the ICD power mode
- New: examples `trv_battery_thread` (SIT ICD, light sleep) and `temp_sensor_battery_thread` (LIT ICD, deep sleep)
- `EspMatterThreadSrp` resolves and browses Matter services with the OpenThread DNS client, while the SRP client has a server; before, nothing over Thread could be resolved (e.g. ICD Check-In clients, subscribers to reconnect to)
- Thread and Wi-Fi Network Diagnostics report the actual network state, incl. the Thread MLE and MAC counters and dataset timestamps
- New: `EspMatterEthDiag`, Ethernet Network Diagnostics (PHY rate, duplex, carrier detect) over an `EthDriver`
- New: `EspSwDiag`, Software Diagnostics with heap usage, watermarks and the FreeRTOS tasks
- The Wi-Fi and Thread stacks take the networks store type as a parameter, `S`, after `E` (`EspWifiMatterStack<'a, B, E, S>`, `EspThreadMatterStack<'a, B, E, S>`), as per `rs-matter-stack`; by default a `WirelessNetworks` with `MAX_WIRELESS_NETWORKS` networks

## [0.1.0] - 2026-09-15

- Initial release
