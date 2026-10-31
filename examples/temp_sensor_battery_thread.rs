//! An example of a Long Idle Time (LIT) Intermittently Connected Device (ICD) over Thread.
//!
//! A LIT ICD is the profile for devices that report rarely and never need to react quickly -
//! a soil moisture, temperature or air quality sensor. Its slow polling interval is not
//! bounded to 15 s: it polls (and is reachable) once per `IdleModeDuration`, minutes apart.
//! Controllers that want to talk to it *register* as Check-In clients through the ICD
//! Management cluster; whenever the device wakes up from idle mode it sends them a Check-In
//! message if their subscription is gone, and they then have `ActiveModeDuration` to
//! re-establish it. Until at least one client is registered, a LIT-capable device behaves as
//! a SIT one (it polls at least every 15 s), so that plain controllers can commission it.
//!
//! With wake-ups that far apart, the sleep model is *deep sleep*: the chip powers everything
//! but the low-power timer down, and every wake-up is a reboot. That works because everything
//! that matters is persisted: the fabrics, the ICD registrations and Check-In counter, the CASE
//! resumption records and the subscriptions (in NVS, by `EspKvBlobStore`), and the Thread
//! attachment state (in NVS too, by OpenThread itself), so that OpenThread can re-attach to its
//! parent with a `Child Update Request` instead of a full attach. The reboot itself is the
//! dominant cost of a wake-up, which is why deep sleep only pays off with wake-ups minutes
//! apart, and why the SIT example (`trv_battery_thread`) uses light sleep instead.
//!
//! What this example wires up, on top of the `trv_battery_thread` (SIT) one:
//! - a LIT mode config (`ActiveModeThreshold >= 5 s`, minutes of idle time) and a `SII` of the
//!   same length, so that the advertised polling interval matches the sleep period;
//! - a user trigger (the `UserActiveModeTrigger` the cluster advertises): pulling `WAKE_GPIO`
//!   low wakes the device from deep sleep (GPIO4 on the ESP32-C6, which only has the low-power
//!   path a deep-sleep wake-up needs on GPIO0..7; the BOOT button, GPIO9, on the ESP32-H2);
//! - the deep-sleep loop: once the ICD state machine reports idle mode, the application waits
//!   for pending flash writes to drain and deep-sleeps until the next poll is due;
//! - the UTC time across deep sleeps: kept by the system clock, which the RTC timer runs on, and
//!   read from the Trusted Time Source once a day.
//!
//! The device is a Temperature Sensor: it takes one (simulated) reading per wake-up, which a
//! subscribed controller receives as a report during the active window. A wake-up counter kept
//! in RTC RAM across deep sleeps drives the simulation, so consecutive reports differ. The BOOT
//! button (GPIO9) held for a few seconds while the device is awake factory-resets it.
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

    use std::time::{Duration, SystemTime};

    use alloc::sync::Arc;

    use esp_idf_matter::matter::utils::timers::{Instant, Timer};

    use esp_idf_matter::error::to_net_error;
    use esp_idf_matter::init_async_io;
    use esp_idf_matter::matter::crypto::{default_crypto, Crypto};
    use esp_idf_matter::matter::dm::clusters::basic_info::BasicInfoConfig;
    use esp_idf_matter::matter::dm::clusters::decl::temperature_measurement::{
        self, ClusterHandler as _,
    };
    use esp_idf_matter::matter::dm::clusters::desc::{self, ClusterHandler as _, DescHandler};
    use esp_idf_matter::matter::dm::clusters::icd_mgmt::{
        ClusterHandler as _, Icd, IcdModeConfig, IcdPowerMode, LitIcd, LitIcdMgmtHandler,
        OperatingModeEnum,
    };
    use esp_idf_matter::matter::dm::clusters::time_sync::client::TimeSyncClient;
    use esp_idf_matter::matter::dm::clusters::time_sync::{
        GranularityEnum, TimeSourceEnum, UtcTime,
    };
    use esp_idf_matter::matter::dm::devices::test::{
        DAC_PRIVKEY, TEST_DEV_ATT, TEST_DEV_COMM, TEST_DEV_DET,
    };
    use esp_idf_matter::matter::dm::devices::DEV_TYPE_ROOT_NODE;
    use esp_idf_matter::matter::dm::endpoints::ROOT_ENDPOINT_ID;
    use esp_idf_matter::matter::dm::{
        Async, AsyncHandler, Cluster, Dataver, DeviceType, EmptyHandler, Endpoint, HandlerContext,
        MatchContext, Node, ReadContext, ReadReply,
    };
    use esp_idf_matter::matter::error::{Error, ErrorCode};
    use esp_idf_matter::matter::tlv::Nullable;
    use esp_idf_matter::matter::utils::epoch::MATTER_EPOCH_SECS;
    use esp_idf_matter::matter::utils::futures::{select, select3, Coalesce, Either};
    use esp_idf_matter::matter::utils::init::InitMaybeUninit;
    use esp_idf_matter::matter::{clusters, devices, with, Matter};
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
    use esp_idf_svc::sys::{
        esp, esp_deep_sleep_start, esp_restart, esp_sleep_enable_ext1_wakeup_io,
        esp_sleep_enable_timer_wakeup, esp_sleep_ext1_wakeup_mode_t_ESP_EXT1_WAKEUP_ANY_LOW,
        esp_sleep_get_wakeup_cause, esp_sleep_source_t_ESP_SLEEP_WAKEUP_EXT1,
        esp_sleep_source_t_ESP_SLEEP_WAKEUP_TIMER, gpio_pulldown_dis, gpio_pullup_en, settimeofday,
        timeval,
    };

    use log::{error, info, warn};

    use static_cell::StaticCell;

    extern crate alloc;

    const STACK_SIZE: usize = 12 * 1024;
    const BUMP_SIZE: usize = 16000;

    /// How long the BOOT button has to be held to factory-reset the device.
    const RESET_SECS: u64 = 3;

    /// The GPIO that wakes the device up from deep sleep when pulled low: the user trigger.
    #[cfg(esp32c6)]
    const WAKE_GPIO: i32 = 4;
    #[cfg(esp32h2)]
    const WAKE_GPIO: i32 = 9;

    /// How long the device sleeps between two wake-ups, in seconds. Also the `IdleModeDuration`
    /// and the `SII` the device advertises: they have to agree, as a deep-sleeping device polls
    /// only when it wakes up.
    const SLEEP_SECS: u32 = 300;

    /// The ICD mode timings this device advertises.
    ///
    /// A LIT device: it idles for `SLEEP_SECS` between its own wake-ups, stays active for five
    /// seconds after each one (enough for a registered client to re-subscribe after the
    /// Check-In) and for five seconds after any Matter message (the spec minimum for LIT).
    ///
    /// Until a client registers with it, though, it operates as a SIT - see
    /// `ICD_SIT_SLOW_POLL_MS`.
    const ICD_MODE: IcdModeConfig = IcdModeConfig {
        idle_mode_duration_s: SLEEP_SECS,
        active_mode_duration_ms: 5000,
        active_mode_threshold_ms: 5000,
        // `CustomInstruction`: the instruction string below tells the user how to wake it up.
        user_active_mode_trigger_hint: 0x0004,
        #[cfg(esp32c6)]
        user_active_mode_trigger_instruction: "Pull GPIO4 low to wake the device",
        #[cfg(esp32h2)]
        user_active_mode_trigger_instruction: "Press the BOOT button to wake the device",
    };

    /// How often the device polls while it operates as a SIT, i.e. until a client registers for
    /// Check-Ins with it: it then stays up (see `deep_sleep_when_idle`) and polls every 5 s,
    /// well under the 15 s a SIT may take, since that is the phase where somebody is waiting to
    /// reach it - the commissioner about to register, or a controller that does not know about
    /// LIT devices at all. Also the `SII` it advertises meanwhile.
    const ICD_SIT_SLOW_POLL_MS: u32 = 5000;

    /// How long the time kept across deep sleeps may go without being set from a real source
    /// (a controller, or the Trusted Time Source), in microseconds: the RTC timer drifts.
    const TIME_SYNC_EVERY_US: u64 = 24 * 60 * 60 * 1_000_000;

    /// How long to wait after entering idle mode before deep-sleeping, so that the background
    /// persistence tasks (fabrics, subscriptions) flush to flash first.
    const SLEEP_SETTLE_MS: u64 = 500;

    /// How far ahead the persisted Check-In counter boundary jumps: this many Check-Ins may be
    /// sent between two flash writes.
    const ICD_COUNTER_EPOCH: u32 = 100;

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
        // Why did we boot: a cold start, the sleep timer, or the user trigger?
        #[allow(non_upper_case_globals)]
        let woken = match unsafe { esp_sleep_get_wakeup_cause() } {
            esp_sleep_source_t_ESP_SLEEP_WAKEUP_TIMER => {
                info!("Woken up from deep sleep by the timer");
                true
            }
            esp_sleep_source_t_ESP_SLEEP_WAKEUP_EXT1 => {
                info!("Woken up by the user trigger (GPIO{WAKE_GPIO})");
                true
            }
            _ => {
                info!("Cold boot");
                false
            }
        };

        // Initialize the Matter stack (can be done only once),
        // as we'll run it in this thread
        let stack = MATTER_STACK.uninit().init_with(EspThreadMatterStack::init(
            &BASIC_INFO,
            TEST_DEV_COMM,
            &TEST_DEV_ATT,
        ));

        // The shared LIT ICD state: the registrations, the Check-In counter and the power mode
        // state machine. Backs the ICD Management cluster handler; the Thread driver and the
        // deep-sleep logic follow its power mode state machine.
        let lit: &'static LitIcd = LIT_ICD.uninit().init_with(LitIcd::init(
            ICD_COUNTER_EPOCH,
            ICD_MODE,
            ICD_SIT_SLOW_POLL_MS,
        ));
        let icd = lit.icd();

        // Take some generic ESP-IDF stuff we'll need later
        let sysloop = EspSystemEventLoop::take()?;
        let nvs = EspDefaultNvsPartition::take()?;
        let peripherals = Peripherals::take()?;

        let mounted_event_fs = Arc::new(MountedEventfs::mount(6)?);
        init_async_io(mounted_event_fs.clone())?;

        // Create the default crypto provider using the STD CSPRNG provided by the `rand` crate
        let crypto = default_crypto(rand::rng(), DAC_PRIVKEY);

        let mut weak_rand = crypto.weak_rand()?;

        // One reading per wake-up. The wake-up counter lives in RTC RAM, which deep sleep keeps.
        let wake_count = {
            // SAFETY: the only access to the static, before anything else runs.
            let count = unsafe { &mut *core::ptr::addr_of_mut!(WAKE_COUNT) };
            *count = count.wrapping_add(1);
            *count
        };
        let reading = simulated_reading(wake_count);
        let sensor = TemperatureSensor::new(Dataver::new_rand(&mut weak_rand), reading);
        info!("Wake-up #{wake_count}, temperature {reading} centi-degrees");

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
            // The ICD Management cluster, on Endpoint 0 as well. Its `run` hook drives the ICD
            // power mode state machine and sends the Check-In messages.
            .chain(
                |e, c| e == ROOT_ENDPOINT_ID && c == ICD_MGMT_CLUSTER.id,
                Async(LitIcdMgmtHandler::new(Dataver::new_rand(&mut weak_rand), lit).adapt()),
            )
            // Our temperature sensor, on Endpoint 1
            .chain(
                |e, c| e == SENSOR_ENDPOINT_ID && c == TemperatureSensor::CLUSTER.id,
                Async(temperature_measurement::HandlerAdaptor(&sensor)),
            )
            // Each Endpoint needs a Descriptor cluster too
            .chain(
                |e, c| e == SENSOR_ENDPOINT_ID && c == DescHandler::CLUSTER.id,
                Async(desc::DescHandler::new(Dataver::new_rand(&mut weak_rand)).adapt()),
            )
            // Reads the time from the Trusted Time Source when due; serves no cluster.
            .chain(|_, _| false, ClockKeeper);

        // An NVS-backed KV BLOB store: an ICD has to remember its fabrics, its ICD
        // registrations, its CASE resumption records and its subscriptions across reboots.
        let mut store = EspKvBlobStore::new_default(nvs.clone())?;
        stack.startup(&crypto, &mut store).await?;

        // The clock, through the deep sleep: Matter's lived in RAM, which is gone.
        if woken {
            restore_clock(stack.matter());
        }

        if stack.matter().has_fabrics() {
            info!(
                "To reset, press and hold the BOOT button (GPIO9) for {RESET_SECS} or more seconds"
            );
        }

        {
            let kv = stack.kv(&mut store);

            // Run the Matter stack with our handler. The Thread driver runs as a Sleepy End
            // Device following the ICD state. It keeps the factory-programmed EUI-64: this device
            // reboots on every wake-up, and a random one would register a new SRP host each time.
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

            // Deep-sleep as soon as the ICD state machine lets us. (The Thread driver logs every
            // ICD power mode transition, so a serial capture shows the duty cycle.)
            let mut deep_sleep = pin!(deep_sleep_when_idle(stack.matter(), icd));

            // Run Matter and also wait for a reset signal
            let mut wait_reset = pin!(wait_reset(PinDriver::input(
                peripherals.pins.gpio9,
                Pull::Up
            )?));

            select3(&mut matter, &mut deep_sleep, &mut wait_reset)
                .coalesce()
                .await?;
        }

        // If we get here, with no errors, this means the user is willing to reset the storage
        // by holding the BOOT button RESET_SECS or more seconds
        warn!("Resetting storage");

        stack.reset(&crypto, (NODE, &handler), store).await?;

        warn!("Rebooting...");

        unsafe { esp_restart() }
    }

    /// The wake-up counter, kept in RTC RAM across deep sleeps (zeroed on a cold boot only).
    #[link_section = ".rtc.data.wake_count"]
    static mut WAKE_COUNT: u32 = 0;

    /// What the clock was when the device went to sleep, kept in RTC RAM: see [`SleptClock`].
    #[link_section = ".rtc.data.slept_clock"]
    static mut SLEPT_CLOCK: SleptClock = SleptClock::UNKNOWN;

    /// What the device knew of the UTC time when it went to sleep; the time itself is in the
    /// system clock.
    #[derive(Clone, Copy)]
    struct SleptClock {
        /// Its `GranularityEnum`; `NoTimeGranularity` if it did not know the time.
        granularity: u8,
        /// When the time was last set from a real source (a controller, or the Trusted Time
        /// Source), Matter-epoch microseconds; 0 if not since a cold boot.
        set_utc_us: u64,
    }

    impl SleptClock {
        const UNKNOWN: Self = Self {
            granularity: GranularityEnum::NoTimeGranularity as u8,
            set_utc_us: 0,
        };
    }

    /// Reads the time from the Trusted Time Source when the device has none, or has not had it
    /// set from a real source for [`TIME_SYNC_EVERY_US`]: not on every wake-up, which would cost
    /// a CASE session each.
    #[derive(Clone, Copy)]
    struct ClockKeeper;

    impl AsyncHandler for ClockKeeper {
        async fn read(&self, _ctx: impl ReadContext, _reply: impl ReadReply) -> Result<(), Error> {
            Err(ErrorCode::AttributeNotFound.into())
        }

        fn bump_dataver(&self, _ctx: impl MatchContext) {}

        async fn run(&self, ctx: impl HandlerContext) -> Result<(), Error> {
            if time_sync_due(ctx.matter()) {
                info!("Reading the time from the Trusted Time Source");

                let client = TimeSyncClient::new(ctx.matter(), ctx.crypto());
                if let Err(e) = client.refresh_once(ctx.kv(), &ctx).await {
                    warn!("No time from the Trusted Time Source: {e:?}");
                }
            }

            core::future::pending().await
        }
    }

    /// A simulated temperature reading, in centi-degrees Celsius: a slow triangle wave around
    /// 21 °C, one step per wake-up.
    fn simulated_reading(wake_count: u32) -> i16 {
        const BASE: i16 = 2100;
        const STEPS: u32 = 20;

        let step = (wake_count % (2 * STEPS)) as i16;
        let delta = if step < STEPS as i16 {
            step
        } else {
            2 * STEPS as i16 - step
        };

        BASE + delta * 25
    }

    /// A minimal Temperature Measurement cluster
    ///
    /// Always reports the reading it was created with.
    /// In real life it would read from a sensor on each wake-up.
    struct TemperatureSensor {
        dataver: Dataver,
        reading: i16,
    }

    impl TemperatureSensor {
        const fn new(dataver: Dataver, reading: i16) -> Self {
            Self { dataver, reading }
        }
    }

    impl temperature_measurement::ClusterHandler for TemperatureSensor {
        // Only the mandatory attributes; `Tolerance` is optional and not served.
        const CLUSTER: Cluster<'static> = temperature_measurement::FULL_CLUSTER
            .with_attrs(with!(required))
            .with_cmds(with!());

        fn dataver(&self) -> u32 {
            self.dataver.get()
        }

        fn dataver_changed(&self) {
            self.dataver.changed();
        }

        fn measured_value(&self, _ctx: impl ReadContext) -> Result<Nullable<i16>, Error> {
            Ok(Nullable::some(self.reading))
        }

        fn min_measured_value(&self, _ctx: impl ReadContext) -> Result<Nullable<i16>, Error> {
            Ok(Nullable::some(-4000))
        }

        fn max_measured_value(&self, _ctx: impl ReadContext) -> Result<Nullable<i16>, Error> {
            Ok(Nullable::some(8500))
        }
    }

    /// Deep-sleep whenever the ICD enters idle mode, until the next poll is due.
    ///
    /// Only once the device is commissioned: an uncommissioned device has a commissioning
    /// window open (which keeps the ICD active anyway) or nothing to sleep for. And only while
    /// operating as a LIT: until a client registers for Check-Ins, a LIT-capable device operates
    /// as a SIT, which has to poll its parent every 15 s at most - a deep sleep of that length
    /// plus the reboot it ends in (Thread re-attach, SRP re-registration and all) would break
    /// that bound and cost more than staying up; that mode is left to OpenThread's own sleepy
    /// polling.
    ///
    /// Never returns: a deep sleep ends in a reboot.
    async fn deep_sleep_when_idle(matter: &Matter<'_>, icd: &Icd) -> Result<(), Error> {
        loop {
            icd.wait_idle().await;

            if icd.operating_mode() == OperatingModeEnum::SIT {
                // No client registered: stay up, and look again at the next idle period.
                icd.wait_active().await;
                continue;
            }

            // Let the persistence tasks flush whatever the active period changed.
            Timer::after_millis(SLEEP_SETTLE_MS).await;

            // Still idle? (A message might have arrived meanwhile.)
            if icd.power_mode() != IcdPowerMode::Idle {
                continue;
            }

            // Sleep until the next poll is due - never longer than the idle period, whichever
            // comes first: a LIT polls once per idle period.
            let poll_ms = icd.net_params().poll_interval_ms as u64;
            let idle_ms = icd
                .idle_until()
                .map(|until| until.saturating_duration_since(Instant::now()).as_millis())
                .unwrap_or(poll_ms);
            let sleep_ms = poll_ms.min(idle_ms).max(1000);

            info!("ICD idle: deep-sleeping for {sleep_ms} ms");

            save_clock(matter);

            deep_sleep(sleep_ms).map_err(to_net_error)?;
        }
    }

    /// Keep what the device knows of the UTC time through a deep sleep: the time in the system
    /// clock, the rest in RTC RAM.
    fn save_clock(matter: &Matter<'_>) {
        // SAFETY: the only access to the static while tasks run, right before the deep sleep.
        let slept = unsafe { core::ptr::addr_of!(SLEPT_CLOCK).read() };

        let (mut clock, utc_us) = matter.with_state(|state| match state.rtc.utc_time() {
            UtcTime::Reliable(utc_us) => {
                // Set this boot by a controller or the Trusted Time Source, not restored.
                let set = !matches!(
                    state.rtc.utc_time_source(),
                    TimeSourceEnum::None | TimeSourceEnum::Unknown
                );

                let clock = SleptClock {
                    granularity: state.rtc.utc_time_granularity() as u8,
                    set_utc_us: if set { utc_us } else { slept.set_utc_us },
                };

                (clock, Some(utc_us))
            }
            UtcTime::LastKnown(_) => (SleptClock::UNKNOWN, None),
        });

        if let Some(Err(e)) = utc_us.map(set_system_time) {
            warn!("Cannot set the system clock: {e}");
            clock = SleptClock::UNKNOWN;
        }

        // SAFETY: the only access to the static while tasks run, right before the deep sleep.
        unsafe { core::ptr::addr_of_mut!(SLEPT_CLOCK).write(clock) };
    }

    /// Whether the time is to be read from the Trusted Time Source: the device has none, or it
    /// has not been set from a real source for [`TIME_SYNC_EVERY_US`].
    fn time_sync_due(matter: &Matter<'_>) -> bool {
        // SAFETY: read-only, while no deep sleep is being prepared.
        let slept = unsafe { core::ptr::addr_of!(SLEPT_CLOCK).read() };

        match matter.with_state(|state| state.rtc.utc_time()) {
            UtcTime::Reliable(now_us) => {
                slept.set_utc_us == 0
                    || now_us.saturating_sub(slept.set_utc_us) > TIME_SYNC_EVERY_US
            }
            UtcTime::LastKnown(_) => true,
        }
    }

    /// Restore the UTC time after a deep sleep, from the system clock. Its granularity steps
    /// down, as for any time set, and its source is unknown: the RTC timer drifts.
    fn restore_clock(matter: &Matter<'_>) {
        // SAFETY: the only access to the static while tasks run, before the deep sleep.
        let slept = unsafe { core::ptr::addr_of!(SLEPT_CLOCK).read() };
        let granularity = match slept.granularity {
            4 => GranularityEnum::MicrosecondsGranularity,
            3 => GranularityEnum::MillisecondsGranularity,
            2 => GranularityEnum::SecondsGranularity,
            1 => GranularityEnum::MinutesGranularity,
            _ => return,
        };

        let Some(utc_us) = SystemTime::UNIX_EPOCH
            .elapsed()
            .ok()
            .and_then(|unix| unix.checked_sub(Duration::from_secs(MATTER_EPOCH_SECS)))
            .map(|utc| utc.as_micros() as u64)
        else {
            return;
        };

        matter.with_state(|state| {
            state
                .rtc
                .set_utc_time(utc_us, granularity, TimeSourceEnum::Unknown, &())
        });

        info!("Clock restored after the deep sleep, {granularity:?}");
    }

    /// Set the system clock, which the RTC timer keeps running in deep sleep, to `utc_us`
    /// (Matter-epoch microseconds).
    fn set_system_time(utc_us: u64) -> std::io::Result<()> {
        let unix_us = utc_us + MATTER_EPOCH_SECS * 1_000_000;
        let time = timeval {
            tv_sec: (unix_us / 1_000_000) as _,
            tv_usec: (unix_us % 1_000_000) as _,
        };

        if unsafe { settimeofday(&time, core::ptr::null()) } == 0 {
            Ok(())
        } else {
            Err(std::io::Error::last_os_error())
        }
    }

    /// Deep-sleep for `ms`, or until the user trigger is pulled low.
    ///
    /// Returns only if the wake-up sources cannot be set up.
    fn deep_sleep(ms: u64) -> Result<(), esp_idf_svc::sys::EspError> {
        esp!(unsafe { esp_sleep_enable_timer_wakeup(ms * 1000) })?;

        // The user trigger. The pad is pulled up, so it does not wake us on its own.
        esp!(unsafe {
            esp_sleep_enable_ext1_wakeup_io(
                1 << WAKE_GPIO,
                esp_sleep_ext1_wakeup_mode_t_ESP_EXT1_WAKEUP_ANY_LOW,
            )
        })?;
        esp!(unsafe { gpio_pullup_en(WAKE_GPIO) })?;
        esp!(unsafe { gpio_pulldown_dis(WAKE_GPIO) })?;

        unsafe { esp_deep_sleep_start() }
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

    /// The LIT ICD state, shared by the ICD Management cluster handler, the Thread driver and
    /// the deep-sleep logic.
    static LIT_ICD: StaticCell<LitIcd> = StaticCell::new();

    /// Basic info about our device.
    ///
    /// The `SAI` / `SII` (session active / idle intervals) are what the device advertises to
    /// controllers *and* the polling intervals of the Thread Sleepy End Device: 300 ms while
    /// the ICD is active, the sleep period while idle (a LIT is reachable once per idle period).
    ///
    /// Both are the values of the device's LIT operation. Until a client registers with the ICD
    /// Management cluster the device operates as a SIT, and the stack then caps both the actual
    /// polling interval and the advertised `SII` to `ICD_SIT_SLOW_POLL_MS`; nothing here needs to
    /// know about it.
    const BASIC_INFO: BasicInfoConfig = BasicInfoConfig {
        sai: Some(300),
        sii: Some(SLEEP_SECS * 1000),
        ..TEST_DEV_DET
    };

    /// Endpoint 0 (the root endpoint) always runs
    /// the hidden Matter system clusters, so we pick ID=1
    const SENSOR_ENDPOINT_ID: u16 = 1;

    /// The Temperature Sensor device type (Matter Device Library), not yet among the device
    /// types `rs-matter` predefines.
    const DEV_TYPE_TEMPERATURE_SENSOR: DeviceType = DeviceType {
        dtype: 0x0302,
        drev: 2,
    };

    /// The ICD Management cluster metadata, exactly as served by `LitIcdMgmtHandler`: the
    /// Check-In Protocol, Long Idle Time and User Active Mode Trigger features.
    const ICD_MGMT_CLUSTER: Cluster<'static> = LitIcdMgmtHandler::CLUSTER;

    /// The Matter Temperature Sensor Node.
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
                    time_sync(time_sync_client),
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
                SENSOR_ENDPOINT_ID,
                devices!(DEV_TYPE_TEMPERATURE_SENSOR),
                clusters!(DescHandler::CLUSTER, TemperatureSensor::CLUSTER),
            ),
        ],
    };
}
