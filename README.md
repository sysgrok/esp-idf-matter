# ![alt text](https://avatars.githubusercontent.com/u/61027988?s=48&v=4 "esp-idf-matter") Run [rs-matter](https://github.com/project-chip/rs-matter) on Espressif chips with [ESP-IDF](https://github.com/esp-rs/esp-idf-svc)

[![CI](https://github.com/ivmarkov/esp-idf-matter/actions/workflows/ci.yml/badge.svg)](https://github.com/ivmarkov/esp-idf-matter/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/esp-idf-matter.svg)](https://crates.io/crates/esp-idf-matter)
[![Documentation](https://img.shields.io/badge/docs-esp--rs-brightgreen)](https://ivmarkov.github.io/esp-idf-matter/esp_idf_matter/index.html)
[![Matrix](https://img.shields.io/matrix/matter-rs:matrix.org?label=join%20matrix&color=BEC5C9&logo=matrix)](https://matrix.to/#/#matter-rs:matrix.org)

## Overview

Everything necessary to run [`rs-matter`](https://github.com/project-chip/rs-matter) on the ESP-IDF:
* Bluedroid implementation of `rs-matter`'s `GattPeripheral` for BLE comissioning support.
* [`rs-matter-stack`](https://github.com/ivmarkov/rs-matter-stack) support with `Netif`, `Ble`, `Wireless` (all of Wifi / Thread / Ethernet) and `KvBlobStore` implementations.

Since ESP-IDF does support the Rust Standard Library, UDP networking just works.

## Example

(See also [All examples](#all-examples))

```rust
//! An example utilizing the `EspWifiMatterStack` struct.
//!
//! As the name suggests, this Matter stack assembly uses Wifi as the main transport,
//! and thus BLE for commissioning.
//!
//! If you want to use Ethernet, utilize `EspEthMatterStack` instead.
//! If you want to use non-concurrent commissioning, call `run` instead of `run_coex`
//! and provision a higher `BUMP_SIZE` because the non-concurrent commissioning currently has a much-higher
//! memory requirements on the futures' sizes (but lower memory requirements inside ESP-IDF).
//! (Note: Alexa does not work (yet) with non-concurrent commissioning.)
//!
//! The example implements a fictitious Light device (an On-Off Matter cluster).
#![allow(unexpected_cfgs)]
#![recursion_limit = "256"]

fn main() -> Result<(), anyhow::Error> {
    #[cfg(any(esp32, esp32s3, esp32c3, esp32c6))]
    {
        example::main()
    }

    #[cfg(not(any(esp32, esp32s3, esp32c3, esp32c6)))]
    panic!("This example is only supported on ESP32, ESP32-S2, ESP32-S3, ESP32-C3 and ESP32-C6 chips. Please select a different example or target.");
}

#[cfg(any(esp32, esp32s3, esp32c3, esp32c6))]
mod example {
    use core::pin::pin;

    use alloc::sync::Arc;

    use embassy_futures::select::select;
    use embassy_time::{Duration, Timer};

    use esp_idf_matter::init_async_io;
    use esp_idf_matter::matter::dm::clusters::desc::{self, ClusterHandler as _, DescHandler};
    use esp_idf_matter::matter::dm::clusters::on_off::{ClusterHandler as _, OnOffHandler};
    use esp_idf_matter::matter::dm::devices::test::{TEST_DEV_ATT, TEST_DEV_COMM, TEST_DEV_DET};
    use esp_idf_matter::matter::dm::devices::DEV_TYPE_ON_OFF_LIGHT;
    use esp_idf_matter::matter::dm::{Async, Dataver, EmptyHandler, Endpoint, EpClMatcher, Node};
    use esp_idf_matter::matter::utils::init::InitMaybeUninit;
    use esp_idf_matter::matter::utils::select::Coalesce;
    use esp_idf_matter::matter::{clusters, devices};
    use esp_idf_matter::wireless::{EspMatterWifi, EspWifiMatterStack};

    use esp_idf_svc::bt::reduce_bt_memory;
    use esp_idf_svc::eventloop::EspSystemEventLoop;
    use esp_idf_svc::hal::peripherals::Peripherals;
    use esp_idf_svc::hal::task::block_on;
    use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
    use esp_idf_svc::io::vfs::MountedEventfs;
    use esp_idf_svc::nvs::EspDefaultNvsPartition;
    use esp_idf_svc::timer::EspTaskTimerService;

    use log::{error, info};

    use static_cell::StaticCell;

    extern crate alloc;

    const STACK_SIZE: usize = 20 * 1024; // Can go down to 15K for esp32c6
    const BUMP_SIZE: usize = 13500;

    pub fn main() -> Result<(), anyhow::Error> {
        esp_idf_svc::log::init_from_env();

        info!("Starting...");

        ThreadSpawnConfiguration::set(&ThreadSpawnConfiguration {
            name: Some(c"matter"),
            ..Default::default()
        })?;

        // Run in a higher-prio thread to avoid issues with `async-io` getting
        // confused by the low priority of the ESP IDF main task
        // Also allocate a very large stack (for now) as `rs-matter` futures do occupy quite some space
        let thread = std::thread::Builder::new()
            .stack_size(STACK_SIZE)
            .spawn(run)
            .unwrap();

        thread.join().unwrap()
    }

    #[inline(never)]
    #[cold]
    fn run() -> Result<(), anyhow::Error> {
        let result = block_on(matter());

        if let Err(e) = &result {
            error!("Matter aborted execution with error: {e:?}");
        }
        {
            info!("Matter finished execution successfully");
        }

        result
    }

    async fn matter() -> Result<(), anyhow::Error> {
        // Initialize the Matter stack (can be done only once),
        // as we'll run it in this thread
        let stack = MATTER_STACK
            .uninit()
            .init_with(EspWifiMatterStack::init_default(
                &TEST_DEV_DET,
                TEST_DEV_COMM,
                &TEST_DEV_ATT,
            ));

        // Take some generic ESP-IDF stuff we'll need later
        let sysloop = EspSystemEventLoop::take()?;
        let timers = EspTaskTimerService::new()?;
        let nvs = EspDefaultNvsPartition::take()?;
        let mut peripherals = Peripherals::take()?;

        let mounted_event_fs = Arc::new(MountedEventfs::mount(3)?);
        init_async_io(mounted_event_fs.clone())?;

        reduce_bt_memory(unsafe { peripherals.modem.reborrow() })?;

        // Our "light" on-off handler.
        // Can be anything implementing `Handler` or `AsyncHandler`
        let on_off = OnOffHandler::new(Dataver::new_rand(stack.matter().rand())).adapt();

        // Chain our endpoint clusters with the
        // (root) Endpoint 0 system clusters in the final handler
        let handler = EmptyHandler
            // Our on-off cluster, on Endpoint 1
            .chain(
                EpClMatcher::new(Some(LIGHT_ENDPOINT_ID), Some(OnOffHandler::CLUSTER.id)),
                Async(&on_off),
            )
            // Each Endpoint needs a Descriptor cluster too
            // Just use the one that `rs-matter` provides out of the box
            .chain(
                EpClMatcher::new(Some(LIGHT_ENDPOINT_ID), Some(DescHandler::CLUSTER.id)),
                Async(desc::DescHandler::new(Dataver::new_rand(stack.matter().rand())).adapt()),
            );

        // Run the Matter stack with our handler
        // Using `pin!` is completely optional, but saves some memory due to `rustc`
        // not being very intelligent w.r.t. stack usage in async functions
        //
        // NOTE: When testing initially, use the `DummyKVBlobStore` to make sure device
        // commissioning works fine with your controller. Once you confirm, you can enable
        // the `EspKvBlobStore` to persist the Matter state in NVS.
        // let store = stack.create_shared_store(esp_idf_matter::persist::EspKvBlobStore::new_default(nvs.clone())?);
        let store = stack.create_shared_store(rs_matter_stack::persist::DummyKvBlobStore);
        let mut matter = pin!(stack.run_coex(
            // The Matter stack needs the Wifi/BLE modem peripheral
            EspMatterWifi::new_with_builtin_mdns(peripherals.modem, sysloop, timers, nvs, stack),
            // The Matter stack needs a persister to store its state
            &store,
            // Our `AsyncHandler` + `AsyncMetadata` impl
            (NODE, handler),
            // No user future to run
            (),
        ));

        // Just for demoing purposes:
        //
        // Run a sample loop that simulates state changes triggered by the HAL
        // Changes will be properly communicated to the Matter controllers
        // (i.e. Google Home, Alexa) and other Matter devices thanks to subscriptions
        let mut device = pin!(async {
            loop {
                // Simulate user toggling the light with a physical switch every 5 seconds
                Timer::after(Duration::from_secs(5)).await;

                // Toggle
                on_off.0.set(!on_off.0.get());

                // Let the Matter stack know that we have changed
                // the state of our Light device
                stack.notify_changed();

                info!("Light toggled");
            }
        });

        // Schedule the Matter run & the device loop together
        select(&mut matter, &mut device).coalesce().await?;

        Ok(())
    }

    /// The Matter stack is allocated statically to avoid
    /// program stack blowups.
    /// It is also a mandatory requirement when the `WifiBle` stack variation is used.
    static MATTER_STACK: StaticCell<EspWifiMatterStack<BUMP_SIZE, ()>> = StaticCell::new();

    /// Endpoint 0 (the root endpoint) always runs
    /// the hidden Matter system clusters, so we pick ID=1
    const LIGHT_ENDPOINT_ID: u16 = 1;

    /// The Matter Light device Node
    const NODE: Node = Node {
        id: 0,
        endpoints: &[
            EspWifiMatterStack::<0, ()>::root_endpoint(),
            Endpoint {
                id: LIGHT_ENDPOINT_ID,
                device_types: devices!(DEV_TYPE_ON_OFF_LIGHT),
                clusters: clusters!(DescHandler::CLUSTER, OnOffHandler::CLUSTER),
            },
        ],
    };
}
```

## Future

* Device Attestation data support using secure flash storage
* Setting system time via Matter
* Matter OTA support based on the ESP-IDF OTA API

## Build Prerequisites

Follow the [Prerequisites](https://github.com/esp-rs/esp-idf-template#prerequisites) section in the `esp-idf-template` crate.

## All examples

The examples could be built and flashed conveniently with [`cargo-espflash`](https://github.com/esp-rs/espflash/). To run e.g. `light_wifi` on an e.g. ESP32-C3:
(Swap the Rust target and example name with the target corresponding for your ESP32 MCU and with the example you would like to build)

with `cargo-espflash`:
```sh
$ MCU=esp32c3 cargo espflash flash --target riscv32imc-esp-espidf --example light_wifi --features examples --monitor
```

| MCU | "--target" |
| --- | ------ |
| esp32c2 | riscv32imc-esp-espidf |
| esp32c3| riscv32imc-esp-espidf |
| esp32c6| riscv32imac-esp-espidf |
| esp32h2 | riscv32imac-esp-espidf |
| esp32p4 | riscv32imafc-esp-espidf |
| esp32 | xtensa-esp32-espidf |
| esp32s2 | xtensa-esp32s2-espidf |
| esp32s3 | xtensa-esp32s3-espidf |
