#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use cyw43_pio::PioSpi;
use embassy_rp::{
    gpio::{Level, Output},
    peripherals::{DMA_CH0, PIO0},
    pio::Pio,
    uart::{self, UartRx},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

use embassy_time::Duration;
use lil_json::{JsonField, JsonValue};
use panic_persist as _;
use picoserve::{make_static, routing::{get, post}, AppBuilder, AppRouter};
use rand::Rng;

embassy_rp::bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<embassy_rp::peripherals::PIO0>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<embassy_rp::peripherals::USB>;
    UART0_IRQ => embassy_rp::uart::InterruptHandler<embassy_rp::peripherals::UART0>;
});

#[derive(Clone, Copy)]
enum GpioCommand {
    LedOn,
    LedOff,
}

/// Serial state shared between UART task and HTTP handlers
/// Tracks parsed time values from scoreboard packets
#[derive(Clone, Copy, PartialEq)]
struct SerialState {
    minutes: u8,
    seconds: u8,
    packet_count: u32,
}

#[derive(Clone, Copy)]
struct SharedSerialState(&'static Mutex<CriticalSectionRawMutex, SerialState>);

/// Scoreboard state: timer and team scores
/// Single writer (timer_task), multiple readers (HTTP handlers)
#[derive(Clone, Copy)]
struct ScoreboardState {
    total_seconds: u32,      // Countdown timer
    running: bool,           // Timer active
    home_score: u16,         // Home team score
    away_score: u16,         // Away team score
}

#[derive(Clone, Copy)]
struct SharedScoreboardState(&'static Mutex<CriticalSectionRawMutex, ScoreboardState>);

struct AppProps {
    gpio_cmd: embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, GpioCommand, 4>,
    scoreboard: SharedScoreboardState,
}

impl AppBuilder for AppProps {
    type PathRouter = impl picoserve::routing::PathRouter;

    fn build_app(self) -> picoserve::Router<Self::PathRouter> {
        let gpio_cmd = self.gpio_cmd;
        let scoreboard = self.scoreboard;
        picoserve::Router::new()
            .route("/", get(|| async move { 
                include_str!("../index.html")
            }))
            .route("/led/on", post(move || async move {
                let _ = gpio_cmd.send(GpioCommand::LedOn).await;
                "LED ON"
            }))
            .route("/led/off", post(move || async move {
                let _ = gpio_cmd.send(GpioCommand::LedOff).await;
                "LED OFF"
            }))
            .route("/api/timer/start", post(move || async move {
                let mut s = scoreboard.0.lock().await;
                s.running = true;
                log::info!("Timer started");
                "OK"
            }))
            .route("/api/timer/stop", post(move || async move {
                let mut s = scoreboard.0.lock().await;
                s.running = false;
                log::info!("Timer stopped");
                "OK"
            }))
            .route("/api/timer/reset", post(move || async move {
                let mut s = scoreboard.0.lock().await;
                s.total_seconds = 0;
                s.running = false;
                log::info!("Timer reset");
                "OK"
            }))
            .route("/api/timer/set", post(move || async move {
                // Note: picoserve doesn't support query params in POST easily,
                // would need to parse from body or use GET with different approach
                "Not implemented yet - awaiting query param support"
            }))
            .route("/api/score/home/inc", post(move || async move {
                let mut s = scoreboard.0.lock().await;
                s.home_score = s.home_score.saturating_add(1);
                log::info!("Home score: {}", s.home_score);
                "OK"
            }))
            .route("/api/score/home/dec", post(move || async move {
                let mut s = scoreboard.0.lock().await;
                s.home_score = s.home_score.saturating_sub(1);
                log::info!("Home score: {}", s.home_score);
                "OK"
            }))
            .route("/api/score/away/inc", post(move || async move {
                let mut s = scoreboard.0.lock().await;
                s.away_score = s.away_score.saturating_add(1);
                log::info!("Away score: {}", s.away_score);
                "OK"
            }))
            .route("/api/score/away/dec", post(move || async move {
                let mut s = scoreboard.0.lock().await;
                s.away_score = s.away_score.saturating_sub(1);
                log::info!("Away score: {}", s.away_score);
                "OK"
            }))
            .route("/api/status", get(move || async move {
                // Format JSON response using lil-json (no heap allocation)
                let s = scoreboard.0.lock().await;
                let minutes = s.total_seconds / 60;
                let seconds = s.total_seconds % 60;
                let running = s.running;
                let home = s.home_score;
                let away = s.away_score;
                drop(s); // Release lock early
                
                // Build time string: MM:SS
                let mut time_buf = [0u8; 5];
                time_buf[0] = b'0' + ((minutes / 10) % 10) as u8;
                time_buf[1] = b'0' + (minutes % 10) as u8;
                time_buf[2] = b':';
                time_buf[3] = b'0' + ((seconds / 10) % 10) as u8;
                time_buf[4] = b'0' + (seconds % 10) as u8;
                let time_str = core::str::from_utf8(&time_buf).unwrap_or("00:00");
                
                // Build JSON fields array
                let fields = [
                    JsonField::new("time", JsonValue::String(time_str)),
                    JsonField::new("running", JsonValue::Boolean(running)),
                    JsonField::new("home", JsonValue::Number(home as i64)),
                    JsonField::new("away", JsonValue::Number(away as i64)),
                ];
                
                // Serialize to static buffer
                static mut JSON_BUF: [u8; 128] = [0; 128];
                let mut output = unsafe { &mut JSON_BUF[..] };
                let len = lil_json::serialize_json_object(&mut output, &fields, 0)
                    .unwrap_or(0);
                
                // Return as &'static str (safe because HTTP response is synchronous)
                unsafe { core::str::from_utf8_unchecked(&JSON_BUF[..len]) }
            }))
    }
}


const WEB_TASK_POOL_SIZE: usize = 8;

#[embassy_executor::task]
async fn logger_task(usb: embassy_rp::Peri<'static, embassy_rp::peripherals::USB>) {
    let driver = embassy_rp::usb::Driver::new(usb, Irqs);
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

/// Timer task: decrements countdown every second if running
/// Updates Mutex-wrapped ScoreboardState
#[embassy_executor::task]
async fn timer_task(state: SharedScoreboardState) -> ! {
    loop {
        embassy_time::Timer::after_secs(1).await;
        
        let mut s = state.0.lock().await;
        if s.running {
            if s.total_seconds > 0 {
                s.total_seconds -= 1;
                log::debug!("Timer: {}s", s.total_seconds);
            } else {
                s.running = false;
                log::info!("Timer complete");
            }
        }
    }
}

/// Reads serial data from UART0 and parses scoreboard packets
/// Demonstrates the pattern from ScoreboardCtrl where serial task:
/// - Buffers bytes until sync byte (0x00) found
/// - Extracts min/sec from packet offsets
/// - Updates Mutex-wrapped state that HTTP handlers can read
/// - Only logs when time values change
#[embassy_executor::task]
async fn read_serial(
    mut rx: UartRx<'static, uart::Async>,
    state: SharedSerialState,
) -> ! {
    let mut byte_buf = [0; 1];
    let mut packet_buf = [0u8; 6];
    let mut buf_idx = 0;
    log::info!("Serial read task started");
    
    loop {
        match rx.read(&mut byte_buf).await {
            Ok(_) => {
                let byte = byte_buf[0];
                
                // Look for sync byte (0x00)
                if byte == 0x00 {
                    buf_idx = 0;
                    packet_buf[buf_idx] = byte;
                    buf_idx += 1;
                } else if buf_idx > 0 {
                    // We're in the middle of a packet
                    packet_buf[buf_idx] = byte;
                    buf_idx += 1;
                    
                    // Check if we have a complete packet (6 bytes)
                    if buf_idx == 6 {
                        // Extract time values from packet
                        let minutes = packet_buf[1];
                        let seconds = packet_buf[2];
                        
                        // Update state and log only on time change
                        let mut s = state.0.lock().await;
                        if s.minutes != minutes || s.seconds != seconds {
                            s.minutes = minutes;
                            s.seconds = seconds;
                            s.packet_count = s.packet_count.saturating_add(1);
                            log::info!("Time: {}:{:02} (packets={})", minutes, seconds, s.packet_count);
                        }
                        
                        buf_idx = 0; // Reset for next packet
                    }
                }
            }
            Err(e) => {
                log::warn!("UART read error: {:?}", e);
                buf_idx = 0; // Reset on error
            }
        }
    }
}

#[embassy_executor::task]
async fn wifi_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut stack: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    stack.run().await
}

#[embassy_executor::task(pool_size = WEB_TASK_POOL_SIZE)]
async fn web_task(
    task_id: usize,
    stack: embassy_net::Stack<'static>,
    app: &'static AppRouter<AppProps>,
    config: &'static picoserve::Config<Duration>,
) -> ! {
    let port = 80;
    let mut tcp_rx_buffer = [0; 1024];
    let mut tcp_tx_buffer = [0; 1024];
    let mut http_buffer = [0; 2048];

    picoserve::Server::new(app, config, &mut http_buffer)
        .listen_and_serve(task_id, stack, port, &mut tcp_rx_buffer, &mut tcp_tx_buffer)
        .await
        .into_never()
}



#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    let p = embassy_rp::init(Default::default());

    if let Some(panic_message) = panic_persist::get_panic_message_utf8() {
        loop {
            log::error!("{panic_message}");
            embassy_time::Timer::after_secs(5).await;
        }
    }

    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = cyw43_pio::PioSpi::new(
        &mut pio.common,
        pio.sm0,
        cyw43_pio::DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    let state = make_static!(cyw43::State, cyw43::State::new());
    let (net_device, mut control, wifi_runner) = cyw43::new(state, pwr, spi, fw).await;

    control.init(clm).await;

    let (stack, net_runner) = embassy_net::new(
        net_device,
        embassy_net::Config::ipv4_static(embassy_net::StaticConfigV4 {
            address: embassy_net::Ipv4Cidr::new(core::net::Ipv4Addr::new(192, 168, 0, 1), 24),
            gateway: None,
            dns_servers: Default::default(),
        }),
        make_static!(
            embassy_net::StackResources<WEB_TASK_POOL_SIZE>,
            embassy_net::StackResources::new()
        ),
        embassy_rp::clocks::RoscRng.gen(),
    );

    spawner.must_spawn(logger_task(p.USB));
    spawner.must_spawn(wifi_task(wifi_runner));
    spawner.must_spawn(net_task(net_runner));

    control
        .start_ap_open("Scoreboard", 8)
        .await;

    let mut led = Output::new(p.PIN_0, Level::Low);
    
    let (gpio_sender, gpio_receiver) = {
        let ch = make_static!(
            embassy_sync::channel::Channel<CriticalSectionRawMutex, GpioCommand, 4>,
            embassy_sync::channel::Channel::new()
        );
        (ch.sender(), ch.receiver())
    };

    // Create shared serial state - accessed by serial task (writer) and HTTP handlers (readers)
    let serial_state = SharedSerialState(
        make_static!(
            Mutex<CriticalSectionRawMutex, SerialState>,
            Mutex::new(SerialState { minutes: 0, seconds: 0, packet_count: 0 })
        )
    );

    // Create shared scoreboard state - timer and scores
    let scoreboard_state = SharedScoreboardState(
        make_static!(
            Mutex<CriticalSectionRawMutex, ScoreboardState>,
            Mutex::new(ScoreboardState { total_seconds: 0, running: false, home_score: 0, away_score: 0 })
        )
    );

    // Configure UART0 for serial reading
    let mut uart_config = embassy_rp::uart::Config::default();
    uart_config.baudrate = 9600;
    let rx = UartRx::new(p.UART0, p.PIN_17, Irqs, p.DMA_CH1, uart_config);
    spawner.must_spawn(read_serial(rx, serial_state));

    // Spawn timer task
    spawner.must_spawn(timer_task(scoreboard_state));

    let app = make_static!(AppRouter<AppProps>, AppProps { 
        gpio_cmd: gpio_sender,
        scoreboard: scoreboard_state,
    }.build_app());

    let config = make_static!(
        picoserve::Config<Duration>,
        picoserve::Config::new(picoserve::Timeouts {
            start_read_request: Some(Duration::from_secs(5)),
            persistent_start_read_request: Some(Duration::from_secs(1)),
            read_request: Some(Duration::from_secs(1)),
            write: Some(Duration::from_secs(1)),
        })
        .keep_connection_alive()
    );

    for task_id in 0..WEB_TASK_POOL_SIZE {
        spawner.must_spawn(web_task(task_id, stack, app, config));
    }

    let gpio = async {
        loop {
            match gpio_receiver.receive().await {
                GpioCommand::LedOn => led.set_high(),
                GpioCommand::LedOff => led.set_low(),
            }
        }
    };

    gpio.await;
}
