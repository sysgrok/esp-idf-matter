//! An example utilizing the `EspThreadMatterStack` struct.
//!
//! As the name suggests, this Matter stack assembly uses Thread as the main transport,
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
    #[cfg(any(esp32c6, esp32h2))]
    {
        example::main()
    }

    #[cfg(not(any(esp32c6, esp32h2)))]
    panic!("This example is only supported on ESP32-C6 and ESP32-H2 chips. Please select a different example or target.");
}

#[cfg(any(esp32c6, esp32h2))]
mod example {
    use core::pin::pin;

    use alloc::sync::Arc;

    use esp_idf_matter::init_async_io;
    use esp_idf_matter::matter::crypto::{default_crypto, Crypto};
    use esp_idf_matter::matter::dm::clusters::app::on_off::test::TestOnOffDeviceLogic;
    use esp_idf_matter::matter::dm::clusters::app::on_off::{self, OnOffHandler, OnOffHooks};
    use esp_idf_matter::matter::dm::clusters::desc::{self, ClusterHandler as _, DescHandler};
    use esp_idf_matter::matter::dm::devices::test::{
        DAC_PRIVKEY, TEST_DEV_ATT, TEST_DEV_COMM, TEST_DEV_DET,
    };
    use esp_idf_matter::matter::dm::devices::DEV_TYPE_ON_OFF_LIGHT;
    use esp_idf_matter::matter::dm::endpoints::ROOT_ENDPOINT_ID;
    use esp_idf_matter::matter::dm::{Async, Dataver, EmptyHandler, Endpoint, Node};
    use esp_idf_matter::matter::persist::DummyKvBlobStore;
    use esp_idf_matter::matter::utils::init::InitMaybeUninit;
    use esp_idf_matter::matter::{clusters, devices};
    use esp_idf_matter::wireless::{EspMatterThread, EspThreadMatterStack};

    #[cfg(esp_idf_bt_bluedroid_enabled)]
    use esp_idf_svc::bt::reduce_bt_memory;
    use esp_idf_svc::eventloop::EspSystemEventLoop;
    use esp_idf_svc::hal::peripherals::Peripherals;
    use esp_idf_svc::hal::task::block_on;
    use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
    use esp_idf_svc::io::vfs::MountedEventfs;
    use esp_idf_svc::nvs::EspDefaultNvsPartition;

    use log::{error, info};

    use rand::Rng;

    use static_cell::StaticCell;

    extern crate alloc;

    const STACK_SIZE: usize = 20 * 1024;
    const BUMP_SIZE: usize = 13000;

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
        let stack = MATTER_STACK.uninit().init_with(EspThreadMatterStack::init(
            &TEST_DEV_DET,
            TEST_DEV_COMM,
            &TEST_DEV_ATT,
        ));

        info!("Matter initialized");

        // Take some generic ESP-IDF stuff we'll need later
        let sysloop = EspSystemEventLoop::take()?;
        let nvs = EspDefaultNvsPartition::take()?;
        // `mut` is only needed for `reduce_bt_memory`'s reborrow below, under Bluedroid.
        #[cfg_attr(not(esp_idf_bt_bluedroid_enabled), allow(unused_mut))]
        let mut peripherals = Peripherals::take()?;

        let mounted_event_fs = Arc::new(MountedEventfs::mount(6)?);
        init_async_io(mounted_event_fs.clone())?;

        // Periodically report free + total internal RAM, so the steady-state headroom (once
        // rs-matter, OpenThread and the BLE host have all claimed their share) can be observed.
        std::thread::Builder::new()
            .stack_size(2048)
            .spawn(|| loop {
                use esp_idf_svc::sys::{
                    heap_caps_get_info, multi_heap_info_t, MALLOC_CAP_INTERNAL,
                };

                let mut info: multi_heap_info_t = unsafe { core::mem::zeroed() };
                unsafe { heap_caps_get_info(&mut info, MALLOC_CAP_INTERNAL) };
                info!(
                    "HEAP internal: free={} largest={} min_ever={} allocated={} (bytes)",
                    info.total_free_bytes,
                    info.largest_free_block,
                    info.minimum_free_bytes,
                    info.total_allocated_bytes,
                );

                std::thread::sleep(core::time::Duration::from_secs(5));
            })
            .unwrap();

        // Frees the Classic-BT memory pool in BLE-only mode. Only available with the Bluedroid
        // host; NimBLE (and the H2/C6, which have no Classic BT) have nothing to free here.
        #[cfg(esp_idf_bt_bluedroid_enabled)]
        reduce_bt_memory(unsafe { peripherals.modem.reborrow() })?;

        // Create the default crypto provider using the STD CSPRNG provided by the `rand` crate
        let crypto = default_crypto(rand::rng(), DAC_PRIVKEY);

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
            // The Endpoint 0 system clusters that are ours to provide.
            // The stack adds the operational network clusters (Network Commissioning,
            // General Commissioning, General Diagnostics and Wifi/Thread/Ethernet
            // Diagnostics) on top, because only it knows the network driver state.
            // Chain any extra Endpoint 0 clusters of your own the same way.
            .chain(
                |e, _| e == ROOT_ENDPOINT_ID,
                Async(EspThreadMatterStack::<0, ()>::root_handler(
                    &(),
                    &mut weak_rand,
                )),
            )
            // Our on-off cluster, on Endpoint 1
            .chain(
                |e, c| e == LIGHT_ENDPOINT_ID && c == TestOnOffDeviceLogic::CLUSTER.id,
                on_off::HandlerAsyncAdaptor(&on_off),
            )
            // Each Endpoint needs a Descriptor cluster too
            // Just use the one that `rs-matter` provides out of the box
            .chain(
                |e, c| e == LIGHT_ENDPOINT_ID && c == DescHandler::CLUSTER.id,
                Async(desc::DescHandler::new(Dataver::new_rand(&mut weak_rand)).adapt()),
            );

        // Create a KV BLOB store and load any previously saved state of `rs-matter`
        // `EspKvBlobStore` saves to an ESP-IDF NVS namespace
        // However, for this demo and for simplicity, we use a dummy KV BLOB store that does nothing
        let mut store = DummyKvBlobStore;
        stack.startup(&crypto, &mut store).await?;

        // Wrap the KV BLOB store as a shared reference, so that it can be used both by `rs-matter` and the user
        let kv = stack.kv(store);

        // Randomize the EUI-64 the Thread SRP host name is derived from, on every boot.
        //
        // This is a demo-only thing to do - a real device must keep the stable,
        // factory-programmed one. But this example deliberately starts uncommissioned every
        // time, and re-flashing it discards the ECDSA key OpenThread signs its SRP
        // registrations with. The SRP server would then reject the (identical) host name
        // for as long as the records from the previous key live on - up to 14 days.
        let mut srp_host_eui64 = [0; 8];
        weak_rand.fill_bytes(&mut srp_host_eui64);

        // Run the Matter stack with our handler
        // Using `pin!` is completely optional, but reduces the size of the final future
        let matter = pin!(stack.run_coex(
            // The Matter stack needs the Thread/BLE modem peripheral
            EspMatterThread::new(peripherals.modem, sysloop, nvs, mounted_event_fs, stack)
                .with_srp_host_eui64(srp_host_eui64),
            // The crypto provider
            &crypto,
            // Our `AsyncHandler` + `AsyncMetadata` impl
            (NODE, handler),
            // The Matter stack needs a blob store to store its state
            &kv,
            // No user future to run
            (),
        ));

        info!("Async Matter task runner initialized");

        info!("About to run Matter");

        // Run Matter
        matter.await?;

        Ok(())
    }

    /// The Matter stack is allocated statically to avoid
    /// program stack blowups.
    /// It is also a mandatory requirement when the `ThreadBle` stack variation is used.
    static MATTER_STACK: StaticCell<EspThreadMatterStack<BUMP_SIZE, ()>> = StaticCell::new();

    /// Endpoint 0 (the root endpoint) always runs
    /// the hidden Matter system clusters, so we pick ID=1
    const LIGHT_ENDPOINT_ID: u16 = 1;

    /// The Matter Light device Node
    const NODE: Node = Node {
        endpoints: &[
            EspThreadMatterStack::<0, ()>::root_endpoint(),
            Endpoint::new(
                LIGHT_ENDPOINT_ID,
                devices!(DEV_TYPE_ON_OFF_LIGHT),
                clusters!(DescHandler::CLUSTER, TestOnOffDeviceLogic::CLUSTER),
            ),
        ],
    };
}
