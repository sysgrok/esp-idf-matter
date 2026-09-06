# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

# Added
- Support for Matter-over-Thread

# Changed
- (Breaking) Update to the latest `rs-matter` (`rand_core` 0.10 / latest RustCrypto) and `rs-matter-stack`; the examples use `rand` 0.10 (`rand::rng()`)
- (Breaking) Endpoint 0 is no longer fully owned by the stack - the examples now chain `<Stack>::root_handler(&(), &mut rand)` on `ROOT_ENDPOINT_ID` themselves; handler chain matchers are closures (`|e, c| e == LIGHT_ENDPOINT_ID && c == ...`)
- `EspMatterThreadSrp` follows the `Mdns::run` signature change (all IPv6 addresses of the interface)
