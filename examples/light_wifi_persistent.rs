//! An example utilizing the `EspWifiMatterStack` struct
//! and additionally persisting the `rs-matter` state to the NOR Flash.
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

    use embassy_futures::select::{select, Either};
    use esp_idf_matter::init_async_io;
    use esp_idf_matter::matter::crypto::{default_crypto, Crypto};
    use esp_idf_matter::matter::dm::clusters::app::on_off::test::TestOnOffDeviceLogic;
    use esp_idf_matter::matter::dm::clusters::app::on_off::{self, OnOffHandler, OnOffHooks};
    use esp_idf_matter::matter::dm::clusters::desc::{self, ClusterHandler as _, DescHandler};
    use esp_idf_matter::matter::dm::devices::test::{
        DAC_PRIVKEY, TEST_DEV_ATT, TEST_DEV_COMM, TEST_DEV_DET,
    };
    use esp_idf_matter::matter::dm::devices::DEV_TYPE_ON_OFF_LIGHT;
    use esp_idf_matter::matter::dm::{Async, Dataver, EmptyHandler, Endpoint, EpClMatcher, Node};
    use esp_idf_matter::matter::error::Error;
    use esp_idf_matter::matter::utils::init::InitMaybeUninit;
    use esp_idf_matter::matter::utils::select::Coalesce;
    use esp_idf_matter::matter::{clusters, devices};
    use esp_idf_matter::persist::EspKvBlobStore;
    use esp_idf_matter::wireless::{EspMatterWifi, EspWifiMatterStack};

    use esp_idf_svc::bt::reduce_bt_memory;
    use esp_idf_svc::eventloop::EspSystemEventLoop;
    use esp_idf_svc::hal::gpio::{Input, PinDriver, Pull};
    use esp_idf_svc::hal::peripherals::Peripherals;
    use esp_idf_svc::hal::task::block_on;
    use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
    use esp_idf_svc::io::vfs::MountedEventfs;
    use esp_idf_svc::nvs::EspDefaultNvsPartition;
    use esp_idf_svc::timer::EspTaskTimerService;

    use log::{error, info, warn};

    use static_cell::StaticCell;

    extern crate alloc;

    const STACK_SIZE: usize = 20 * 1024; // Can go down to 15K for esp32c6
    const BUMP_SIZE: usize = 15000;

    const RESET_SECS: u64 = 3;

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

        // Create the default crypto provider using the STD CSPRNG provided by the `rand` crate
        let crypto = default_crypto(rand::thread_rng(), DAC_PRIVKEY);

        let mut weak_rand = crypto.weak_rand()?;

        // Our "light" on-off handler.
        // It will toggle the light state every 5 seconds
        let on_off = OnOffHandler::new_standalone(
            Dataver::new_rand(&mut weak_rand),
            LIGHT_ENDPOINT_ID,
            TestOnOffDeviceLogic::new(true),
        );

        // Chain our endpoint clusters with the
        // (root) Endpoint 0 system clusters in the final handler
        let handler = EmptyHandler
            // Our on-off cluster, on Endpoint 1
            .chain(
                EpClMatcher::new(
                    Some(LIGHT_ENDPOINT_ID),
                    Some(TestOnOffDeviceLogic::CLUSTER.id),
                ),
                on_off::HandlerAsyncAdaptor(&on_off),
            )
            // Each Endpoint needs a Descriptor cluster too
            // Just use the one that `rs-matter` provides out of the box
            .chain(
                EpClMatcher::new(Some(LIGHT_ENDPOINT_ID), Some(DescHandler::CLUSTER.id)),
                Async(desc::DescHandler::new(Dataver::new_rand(&mut weak_rand)).adapt()),
            );

        // Create a KV BLOB store and load any previously saved state of `rs-matter`
        let mut kv = EspKvBlobStore::new_default(nvs.clone())?;
        stack.startup(&crypto, &mut kv).await?;

        if stack.is_commissioned() {
            info!(
                "To reset, press and hold the Boot Mode pin (GPIO9) for {} or more seconds",
                RESET_SECS
            );
        }

        {
            // Wrap the KV BLOB store as a shared reference, so that it can be used both by `rs-matter` and the user
            let kv = stack.create_shared_kv(&mut kv)?;

            // Run the Matter stack with our handler
            // Using `pin!` is completely optional, but reduces the size of the final future
            let mut matter = pin!(stack.run_coex(
                // The Matter stack needs the Wifi/BLE modem peripheral
                EspMatterWifi::new_with_builtin_mdns(
                    peripherals.modem,
                    sysloop,
                    timers,
                    nvs,
                    stack
                ),
                // The crypto provider
                &crypto,
                // Our `AsyncHandler` + `AsyncMetadata` impl
                (NODE, handler),
                // The Matter stack needs a blob store to store its state
                &kv,
                // No user future to run
                (),
            ));

            // Run Matter and also wait for a reset signal
            let mut wait_reset = pin!(wait_pin_low(PinDriver::input(
                peripherals.pins.gpio9,
                Pull::Down
            )?));

            select(&mut matter, &mut wait_reset).coalesce().await?;
        }

        // If we get here, with no errors, this means the user is willing to reset the storage
        // by holding the BOOT pin low 3 or more seconds
        warn!("Resetting storage");

        stack.reset(kv).await?;

        warn!("Rebooting...");

        esp_idf_svc::hal::reset::restart()
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
        endpoints: &[
            EspWifiMatterStack::<0, ()>::root_endpoint(),
            Endpoint::new(
                LIGHT_ENDPOINT_ID,
                devices!(DEV_TYPE_ON_OFF_LIGHT),
                clusters!(DescHandler::CLUSTER, TestOnOffDeviceLogic::CLUSTER),
            ),
        ],
    };

    async fn wait_pin_low(mut pin: PinDriver<'_, Input>) -> Result<(), Error> {
        loop {
            let _ = pin.wait_for_low().await;

            // Debounce
            embassy_time::Timer::after_millis(50).await;

            if pin.is_low() {
                warn!(
                    "Detected Boot Mode pin low, keep it low for {} more seconds to reset the storage",
                    RESET_SECS
                );

                let result = select(
                    pin.wait_for_high(),
                    embassy_time::Timer::after_secs(RESET_SECS),
                )
                .await;

                if matches!(result, Either::Second(())) {
                    break;
                }
            }
        }

        Ok(())
    }
}
