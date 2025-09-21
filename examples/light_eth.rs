//! An example utilizing the `EspEthMatterStack` struct.
//! As the name suggests, this Matter stack assembly uses Ethernet as the main transport, as well as for commissioning.
//!
//! Notice thart we actually don't use Ethernet for real, as ESP32s don't have Ethernet ports out of the box.
//! Instead, we utilize Wifi, which - from the POV of Matter - is indistinguishable from Ethernet as long as the Matter
//! stack is not concerned with connecting to the Wifi network, managing its credentials etc. and can assume it "pre-exists".
//!
//! The example implements a fictitious Light device (an On-Off Matter cluster).
#![allow(unexpected_cfgs)]
#![recursion_limit = "256"]

fn main() -> Result<(), anyhow::Error> {
    #[cfg(any(esp32, esp32s2, esp32s3, esp32c3, esp32c6))]
    {
        example::main()
    }

    #[cfg(not(any(esp32, esp32s2, esp32s3, esp32c3, esp32c6)))]
    panic!("This example is only supported on ESP32, ESP32-S2, ESP32-S3, ESP32-C3 and ESP32-C6 chips. Please select a different example or target.");
}

#[cfg(any(esp32, esp32s2, esp32s3, esp32c3, esp32c6))]
mod example {
    use core::pin::pin;

    use alloc::sync::Arc;

    use embassy_futures::select::select;
    use embassy_time::{Duration, Timer};

    use esp_idf_matter::eth::EspEthMatterStack;
    use esp_idf_matter::init_async_io;
    use esp_idf_matter::matter::dm::clusters::desc::{ClusterHandler as _, DescHandler};
    use esp_idf_matter::matter::dm::clusters::gen_diag::InterfaceTypeEnum;
    use esp_idf_matter::matter::dm::clusters::on_off::{ClusterHandler as _, OnOffHandler};
    use esp_idf_matter::matter::dm::devices::test::{TEST_DEV_ATT, TEST_DEV_COMM, TEST_DEV_DET};
    use esp_idf_matter::matter::dm::devices::DEV_TYPE_ON_OFF_LIGHT;
    use esp_idf_matter::matter::dm::{Async, Dataver, EmptyHandler, Endpoint, EpClMatcher, Node};
    use esp_idf_matter::matter::utils::init::InitMaybeUninit;
    use esp_idf_matter::matter::utils::select::Coalesce;
    use esp_idf_matter::matter::{clusters, devices};
    use esp_idf_matter::netif::{EspMatterNetStack, EspMatterNetif};

    use esp_idf_svc::eventloop::EspSystemEventLoop;
    use esp_idf_svc::hal::peripherals::Peripherals;
    use esp_idf_svc::hal::task::block_on;
    use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
    use esp_idf_svc::handle::RawHandle;
    use esp_idf_svc::io::vfs::MountedEventfs;
    use esp_idf_svc::nvs::EspDefaultNvsPartition;
    use esp_idf_svc::sys::{esp, esp_netif_create_ip6_linklocal};
    use esp_idf_svc::timer::EspTaskTimerService;
    use esp_idf_svc::wifi::{self, AsyncWifi, EspWifi};

    use log::{error, info};

    use rs_matter_stack::mdns::BuiltinMdns;
    use static_cell::StaticCell;

    extern crate alloc;

    const STACK_SIZE: usize = 20 * 1024; // Can go down to 15K for esp32c6
    const BUMP_SIZE: usize = 13500;

    const WIFI_SSID: &str = env!("WIFI_SSID");
    const WIFI_PASS: &str = env!("WIFI_PASS");

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
            .init_with(EspEthMatterStack::init_default(
                &TEST_DEV_DET,
                TEST_DEV_COMM,
                &TEST_DEV_ATT,
            ));

        // Take some generic ESP-IDF stuff we'll need later
        let sysloop = EspSystemEventLoop::take()?;
        let nvs = EspDefaultNvsPartition::take()?;
        let peripherals = Peripherals::take()?;

        let mounted_event_fs = Arc::new(MountedEventfs::mount(3)?);
        init_async_io(mounted_event_fs.clone())?;

        // Configure and start the Wifi first
        let mut wifi = Box::new(AsyncWifi::wrap(
            EspWifi::new(peripherals.modem, sysloop.clone(), Some(nvs.clone()))?,
            sysloop.clone(),
            EspTaskTimerService::new()?,
        )?);
        wifi.set_configuration(&wifi::Configuration::Client(wifi::ClientConfiguration {
            ssid: WIFI_SSID.try_into().unwrap(),
            password: WIFI_PASS.try_into().unwrap(),
            ..Default::default()
        }))?;
        wifi.start().await?;
        wifi.connect().await?;

        // Matter needs an IPv6 address to work
        esp!(unsafe { esp_netif_create_ip6_linklocal(wifi.wifi().sta_netif().handle() as _) })?;

        wifi.wait_netif_up().await?;

        // Our "light" on-off cluster.
        // Can be anything implementing `rs_matter::data_model::AsyncHandler`
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
                Async(DescHandler::new(Dataver::new_rand(stack.matter().rand())).adapt()),
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
        let mut matter = pin!(stack.run_preex(
            // The Matter stack needs UDP sockets to communicate with other Matter devices
            EspMatterNetStack::new(),
            // The Matter stack need access to the netif on which we'll operate
            // Since we are pretending to use a wired Ethernet connection - yet -
            // we are using a Wifi STA - provide the Wifi netif here
            EspMatterNetif::new(wifi.wifi().sta_netif(), InterfaceTypeEnum::WiFi, sysloop),
            // The Matter stack needs an mDNS service to advertise itself
            BuiltinMdns,
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
    static MATTER_STACK: StaticCell<EspEthMatterStack<BUMP_SIZE, ()>> = StaticCell::new();

    /// Endpoint 0 (the root endpoint) always runs
    /// the hidden Matter system clusters, so we pick ID=1
    const LIGHT_ENDPOINT_ID: u16 = 1;

    /// The Matter Light device Node
    const NODE: Node = Node {
        id: 0,
        endpoints: &[
            EspEthMatterStack::<0, ()>::root_endpoint(),
            Endpoint {
                id: LIGHT_ENDPOINT_ID,
                device_types: devices!(DEV_TYPE_ON_OFF_LIGHT),
                clusters: clusters!(DescHandler::CLUSTER, OnOffHandler::CLUSTER),
            },
        ],
    };
}
