//! An example of a Short Idle Time (SIT) Intermittently Connected Device (ICD) over Thread:
//! a battery-powered radiator valve (a heating-only Thermostat).
//!
//! A SIT ICD is the profile for battery devices that must be *reachable on the controller's
//! initiative* within seconds: a controller writes a setpoint or changes the mode, and the user
//! expects the valve to react right away; controllers also read and subscribe to its local
//! temperature. That is what distinguishes it from a sensor that only pushes readings now and
//! then - such a device is a Long Idle Time ICD, see `temp_sensor_battery_thread`, and making it
//! a SIT wastes its battery on polls nobody needs.
//!
//! On the network a SIT device is a Thread Sleepy End Device that keeps its receiver off between
//! data polls and polls its parent at most every 15 seconds, so anything a controller sends
//! arrives within that window. Its sleep model is *light sleep*: the MCU parks between polls
//! with its RAM retained, so the Thread attachment, the Matter sessions and the subscriptions
//! all stay alive, and every wake-up is a few milliseconds: poll the parent, handle whatever it
//! queued, sleep again. The valve motor itself draws next to nothing, so the radio duty cycle
//! is what decides the battery life.
//!
//! What this example wires up:
//! - the ICD Management cluster of a SIT-only device on the root endpoint (no features, just
//!   the mode timings), backed by the shared `Icd` power mode state machine;
//! - the Thread driver following that state: the node polls fast (`SAI`) while the ICD is in
//!   active mode and slowly (`SII`, capped to 15 s for SIT) while idle;
//! - an NVS-backed KV store, so the fabrics, the CASE resumption records and the thermostat's
//!   setpoints survive a reboot (OpenThread keeps its attachment state in NVS by itself);
//! - CASE session resumption (the `case-resumption` feature), so a controller coming back after
//!   a while re-establishes its session with one round trip instead of a full handshake;
//! - ESP-IDF automatic light sleep (the power management `light_sleep_enable`), which is what
//!   makes the MCU actually sleep between polls.
//!
//! NOTE: automatic light sleep needs these in the `sdkconfig.defaults` of the build, on top of
//! the usual ones: `CONFIG_PM_ENABLE=y`, `CONFIG_FREERTOS_USE_TICKLESS_IDLE=y` and
//! `CONFIG_IEEE802154_SLEEP_ENABLE=y`; and, as the BLE controller stays enabled for the whole
//! run (concurrent commissioning), `CONFIG_BT_LE_SLEEP_ENABLE=y` so that it sleeps too. Without
//! them the example runs all the same, polling as a Sleepy End Device, but the chip stays awake.
//!
//! The BOOT button (GPIO9) held for a few seconds factory-resets the device.
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

    use embassy_futures::select::{select, Either};
    use embassy_time::Timer;

    use esp_idf_matter::error::to_net_error;
    use esp_idf_matter::init_async_io;
    use esp_idf_matter::matter::crypto::{default_crypto, Crypto};
    use esp_idf_matter::matter::dm::clusters::app::thermostat::{
        self, ControlSequenceOfOperationEnum, RelayStateBitmap, SystemModeEnum, ThermostatHooks,
    };
    use esp_idf_matter::matter::dm::clusters::basic_info::BasicInfoConfig;
    use esp_idf_matter::matter::dm::clusters::decl::thermostat as thermostat_cluster;
    use esp_idf_matter::matter::dm::clusters::desc::{self, ClusterHandler as _, DescHandler};
    use esp_idf_matter::matter::dm::clusters::icd_mgmt::{
        ClusterHandler as _, Icd, IcdModeConfig, SitIcdMgmtHandler,
    };
    use esp_idf_matter::matter::dm::devices::test::{
        DAC_PRIVKEY, TEST_DEV_ATT, TEST_DEV_COMM, TEST_DEV_DET,
    };
    use esp_idf_matter::matter::dm::devices::{DEV_TYPE_ROOT_NODE, DEV_TYPE_THERMOSTAT};
    use esp_idf_matter::matter::dm::endpoints::ROOT_ENDPOINT_ID;
    use esp_idf_matter::matter::dm::{Async, Cluster, Dataver, EmptyHandler, Endpoint, Node};
    use esp_idf_matter::matter::error::Error;
    use esp_idf_matter::matter::persist::VENDOR_KEYS_START;
    use esp_idf_matter::matter::utils::cell::RefCell;
    use esp_idf_matter::matter::utils::init::InitMaybeUninit;
    use esp_idf_matter::matter::utils::select::Coalesce;
    use esp_idf_matter::matter::utils::sync::blocking::Mutex;
    use esp_idf_matter::matter::{clusters, devices, with};
    use esp_idf_matter::persist::EspKvBlobStore;
    use esp_idf_matter::sw_diag::EspSwDiag;
    use esp_idf_matter::wireless::{EspMatterThread, EspThreadMatterStack};

    use esp_idf_svc::eventloop::EspSystemEventLoop;
    use esp_idf_svc::hal::gpio::{Input, PinDriver, Pull};
    use esp_idf_svc::hal::peripherals::Peripherals;
    use esp_idf_svc::hal::task::block_on;
    use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
    use esp_idf_svc::io::vfs::MountedEventfs;
    use esp_idf_svc::nvs::EspDefaultNvsPartition;
    use esp_idf_svc::sys::esp_restart;

    use log::{error, info, warn};

    use static_cell::StaticCell;

    extern crate alloc;

    const STACK_SIZE: usize = 12 * 1024;
    const BUMP_SIZE: usize = 16000;

    /// How long the BOOT button has to be held to factory-reset the device.
    const RESET_SECS: u64 = 3;

    /// The ICD mode timings this device advertises.
    ///
    /// A SIT device: it stays active for a second after boot, and for a second after any Matter
    /// message, so that a multi-message exchange does not fall back to slow polling halfway
    /// through. The *polling* interval is the `SII` in `BASIC_INFO` (15 s, the most a SIT may
    /// take).
    ///
    /// `idle_mode_duration_s` is how long the device stays idle before it goes active on its own; a
    /// SIT has no Check-Ins to send on such wake-ups, so it only bounds the idle periods.
    const ICD_MODE: IcdModeConfig = IcdModeConfig {
        idle_mode_duration_s: 60,
        active_mode_duration_ms: 1000,
        active_mode_threshold_ms: 1000,
        user_active_mode_trigger_hint: 0,
        user_active_mode_trigger_instruction: "",
    };

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
        // `pin!` keeps the future in this frame: passed by value, `block_on` would copy it into its own
        let result = block_on(pin!(matter()));

        if let Err(e) = &result {
            error!("Matter aborted execution with error: {e:?}");
        }

        result
    }

    async fn matter() -> Result<(), anyhow::Error> {
        enable_light_sleep();

        // Initialize the Matter stack (can be done only once),
        // as we'll run it in this thread
        let stack = MATTER_STACK.uninit().init_with(EspThreadMatterStack::init(
            &BASIC_INFO,
            TEST_DEV_COMM,
            &TEST_DEV_ATT,
        ));

        // The ICD power mode state machine. Backs the ICD Management cluster handler, and is
        // followed by the Thread driver. A SIT device has no registrations and no Check-In
        // counter.
        let icd: &'static Icd = ICD.uninit().init_with(Icd::init(ICD_MODE));

        // Take some generic ESP-IDF stuff we'll need later
        let sysloop = EspSystemEventLoop::take()?;
        let nvs = EspDefaultNvsPartition::take()?;
        let peripherals = Peripherals::take()?;

        let mounted_event_fs = Arc::new(MountedEventfs::mount(6)?);
        init_async_io(mounted_event_fs.clone())?;

        // Create the default crypto provider using the STD CSPRNG provided by the `rand` crate
        let crypto = default_crypto(rand::rng(), DAC_PRIVKEY);

        let mut weak_rand = crypto.weak_rand()?;

        // The radiator valve: a heating-only Thermostat. The handler owns and persists the
        // setpoints and the mode; the logic below is the (simulated) valve itself.
        let valve = RadiatorValve::new();
        let thermostat = thermostat::ThermostatHandler::new(
            Dataver::new_rand(&mut weak_rand),
            VALVE_ENDPOINT_ID,
            VENDOR_KEYS_START + 0x10,
            &valve,
        );

        // Chain our endpoint clusters
        let handler = EmptyHandler
            // The Endpoint 0 system clusters that are ours to provide.
            // The stack adds the operational network clusters on top.
            .chain(
                |e, _| e == ROOT_ENDPOINT_ID,
                Async(EspThreadMatterStack::<0, ()>::root_handler(
                    &EspSwDiag,
                    &mut weak_rand,
                )),
            )
            // The ICD Management cluster of a SIT device, on Endpoint 0 as well: no features,
            // just the mode timings. Its `run` hook drives the ICD power mode state machine.
            .chain(
                |e, c| e == ROOT_ENDPOINT_ID && c == ICD_MGMT_CLUSTER.id,
                Async(SitIcdMgmtHandler::new(Dataver::new_rand(&mut weak_rand), icd).adapt()),
            )
            // Our thermostat cluster, on Endpoint 1
            .chain(
                |e, c| e == VALVE_ENDPOINT_ID && c == RadiatorValve::CLUSTER.id,
                Async(thermostat::HandlerAdaptor(&thermostat)),
            )
            // Each Endpoint needs a Descriptor cluster too
            .chain(
                |e, c| e == VALVE_ENDPOINT_ID && c == DescHandler::CLUSTER.id,
                Async(desc::DescHandler::new(Dataver::new_rand(&mut weak_rand)).adapt()),
            );

        // An NVS-backed KV BLOB store: an ICD has to remember its fabrics and its CASE
        // resumption records across reboots.
        let mut store = EspKvBlobStore::new_default(nvs.clone())?;
        stack.startup(&crypto, &mut store).await?;

        if stack.matter().has_fabrics() {
            info!(
                "To reset, press and hold the BOOT button (GPIO9) for {RESET_SECS} or more seconds"
            );
        }

        {
            let kv = stack.kv(&mut store);

            // Run the Matter stack with our handler. The Thread driver runs as a Sleepy End
            // Device following the ICD state. It keeps the factory-programmed EUI-64, so that the
            // node keeps its SRP host across reboots - the SRP key it registers with is persisted
            // along with the rest.
            let mut matter = pin!(stack.run_coex(
                EspMatterThread::new(peripherals.modem, sysloop, nvs, mounted_event_fs, stack)
                    .with_icd(icd),
                // The crypto provider
                &crypto,
                // Our `AsyncHandler` + `AsyncMetadata` impl
                (NODE, &handler),
                // The Matter stack needs a blob store to store its state
                &kv,
                // No user future to run
                (),
            ));

            // Run Matter and also wait for a reset signal. (The Thread driver logs every ICD
            // power mode transition, so a serial capture shows the duty cycle.)
            let mut wait_reset = pin!(wait_reset(PinDriver::input(
                peripherals.pins.gpio9,
                Pull::Up
            )?));

            select(&mut matter, &mut wait_reset).coalesce().await?;
        }

        // If we get here, with no errors, this means the user is willing to reset the storage
        // by holding the BOOT button RESET_SECS or more seconds
        warn!("Resetting storage");

        stack.reset(&crypto, (NODE, &handler), store).await?;

        warn!("Rebooting...");

        unsafe { esp_restart() }
    }

    /// Let the chip light-sleep whenever nothing holds it awake, if the build allows it (see the
    /// module docs): between the polls of the Sleepy End Device, for the most part.
    fn enable_light_sleep() {
        #[cfg(all(esp_idf_pm_enable, esp_idf_freertos_use_tickless_idle))]
        {
            use esp_idf_svc::sys::{
                esp, esp_pm_config_t, esp_pm_configure, CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
                CONFIG_XTAL_FREQ,
            };

            let config = esp_pm_config_t {
                max_freq_mhz: CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ as _,
                min_freq_mhz: CONFIG_XTAL_FREQ as _,
                light_sleep_enable: true,
            };

            match esp!(unsafe { esp_pm_configure(&config as *const _ as *const _) }) {
                Ok(()) => info!("Automatic light sleep enabled"),
                Err(e) => warn!("Cannot enable automatic light sleep: {e}"),
            }
        }

        #[cfg(not(all(esp_idf_pm_enable, esp_idf_freertos_use_tickless_idle)))]
        warn!(
            "Automatic light sleep needs CONFIG_PM_ENABLE and CONFIG_FREERTOS_USE_TICKLESS_IDLE; \
             the chip stays awake between polls"
        );
    }

    /// Resolve once the BOOT button is held low for `RESET_SECS`.
    async fn wait_reset(mut pin: PinDriver<'_, Input>) -> Result<(), Error> {
        loop {
            pin.wait_for_low().await.map_err(to_net_error)?;

            // Debounce
            Timer::after_millis(50).await;

            if pin.is_low() {
                warn!(
                    "BOOT button pressed, keep it pressed for {RESET_SECS} more seconds to reset the storage"
                );

                let result = select(pin.wait_for_high(), Timer::after_secs(RESET_SECS)).await;

                if matches!(result, Either::Second(())) {
                    break Ok(());
                }
            }
        }
    }

    /// The Matter stack is allocated statically to avoid
    /// program stack blowups.
    /// It is also a mandatory requirement when the `ThreadBle` stack variation is used.
    static MATTER_STACK: StaticCell<EspThreadMatterStack<BUMP_SIZE, ()>> = StaticCell::new();

    /// The ICD power mode state machine, shared by the ICD Management cluster handler and the
    /// Thread driver.
    static ICD: StaticCell<Icd> = StaticCell::new();

    /// Basic info about our device.
    ///
    /// The `SAI` / `SII` (session active / idle intervals) are what the device advertises to
    /// controllers *and* the polling intervals of the Thread Sleepy End Device: 300 ms while
    /// the ICD is active, 15 s while idle - the slowest a SIT device may poll.
    const BASIC_INFO: BasicInfoConfig = BasicInfoConfig {
        sai: Some(300),
        sii: Some(15_000),
        ..TEST_DEV_DET
    };

    /// Endpoint 0 (the root endpoint) always runs
    /// the hidden Matter system clusters, so we pick ID=1
    const VALVE_ENDPOINT_ID: u16 = 1;

    /// The simulated radiator valve: the thermostat logic behind the Thermostat cluster handler.
    ///
    /// A real one would read its temperature sensor and drive the valve motor from `apply`; this
    /// one keeps a fixed room temperature and just remembers what the handler asked for.
    struct RadiatorValve {
        state: Mutex<RefCell<ValveState>>,
    }

    struct ValveState {
        system_mode: SystemModeEnum,
        heating_setpoint: i16,
        local_temperature: i16,
    }

    impl RadiatorValve {
        const fn new() -> Self {
            Self {
                state: Mutex::new(RefCell::new(ValveState {
                    system_mode: SystemModeEnum::Off,
                    heating_setpoint: Self::OCCUPIED_HEATING_SETPOINT,
                    local_temperature: 1950,
                })),
            }
        }

        /// Whether the valve is open: heating, and the room below the setpoint.
        fn heating(&self) -> bool {
            self.state.lock(|state| {
                let state = state.borrow();

                state.system_mode == SystemModeEnum::Heat
                    && state.local_temperature < state.heating_setpoint
            })
        }
    }

    impl ThermostatHooks for RadiatorValve {
        /// A heating-only thermostat: the `HEAT` feature alone, the mandatory attributes plus
        /// the heat setpoint limits, and the one mandatory command.
        const CLUSTER: Cluster<'static> = thermostat_cluster::FULL_CLUSTER
            .with_revision(11)
            .with_features(thermostat_cluster::Feature::HEATING.bits())
            .with_attrs(with!(
                required;
                thermostat_cluster::AttributeId::AbsMinHeatSetpointLimit
                    | thermostat_cluster::AttributeId::AbsMaxHeatSetpointLimit
                    | thermostat_cluster::AttributeId::OccupiedHeatingSetpoint
                    | thermostat_cluster::AttributeId::MinHeatSetpointLimit
                    | thermostat_cluster::AttributeId::MaxHeatSetpointLimit
                    | thermostat_cluster::AttributeId::ThermostatRunningState
            ))
            .with_cmds(with!(thermostat_cluster::CommandId::SetpointRaiseLower))
            .with_events(with!());

        const CONTROL_SEQUENCE_OF_OPERATION: ControlSequenceOfOperationEnum =
            ControlSequenceOfOperationEnum::HeatingOnly;

        /// 20.00 °C to begin with.
        const OCCUPIED_HEATING_SETPOINT: i16 = 2000;

        fn local_temperature(&self) -> Option<i16> {
            Some(self.state.lock(|state| state.borrow().local_temperature))
        }

        fn running_state(&self) -> RelayStateBitmap {
            if self.heating() {
                RelayStateBitmap::HEAT
            } else {
                RelayStateBitmap::empty()
            }
        }

        /// Called whenever a controller (or the `SetpointRaiseLower` command) changes the mode
        /// or a setpoint - the moment a real valve would start its motor.
        fn apply(
            &self,
            system_mode: SystemModeEnum,
            heating_setpoint: i16,
            _cooling_setpoint: i16,
        ) {
            self.state.lock(|state| {
                let mut state = state.borrow_mut();

                state.system_mode = system_mode;
                state.heating_setpoint = heating_setpoint;
            });

            info!(
                "Valve: mode {:?}, setpoint {}.{:02} C, {}",
                system_mode,
                heating_setpoint / 100,
                (heating_setpoint % 100).abs(),
                if self.heating() { "open" } else { "closed" }
            );
        }
    }

    /// The ICD Management cluster metadata, exactly as served by `SitIcdMgmtHandler`: a SIT-only
    /// device, which claims no ICD features and advertises no `ICD` DNS-SD TXT key.
    const ICD_MGMT_CLUSTER: Cluster<'static> = SitIcdMgmtHandler::CLUSTER;

    /// The Matter Thermostat (radiator valve) Node.
    ///
    /// The root endpoint is the stack's usual Thread one plus the ICD Management cluster.
    const NODE: Node = Node {
        endpoints: &[
            Endpoint::new(
                ROOT_ENDPOINT_ID,
                devices!(DEV_TYPE_ROOT_NODE),
                clusters!(
                    thread,
                    sw_diag(heap, watermarks, thread),
                    thread_diag(
                        mle_counts,
                        mac_counts,
                        active_timestamp,
                        pending_timestamp,
                        delay
                    );
                    ICD_MGMT_CLUSTER
                ),
            ),
            Endpoint::new(
                VALVE_ENDPOINT_ID,
                devices!(DEV_TYPE_THERMOSTAT),
                clusters!(DescHandler::CLUSTER, RadiatorValve::CLUSTER),
            ),
        ],
    };
}
