#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use cyw43_pio::PioSpi;
use embassy_rp::{
    gpio::{Level, Output},
    peripherals::{DMA_CH0, PIO0},
    pio::Pio,
};
use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use embassy_time::Duration;
use panic_persist as _;
use picoserve::{make_static, routing::{get, post}, AppBuilder, AppRouter};
use rand::Rng;

embassy_rp::bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<embassy_rp::peripherals::PIO0>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<embassy_rp::peripherals::USB>;
});

#[derive(Clone, Copy)]
enum GpioCommand {
    LedOn,
    LedOff,
}



type GpioCmdSender = embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, GpioCommand, 4>;

struct AppProps {
    gpio_cmd: GpioCmdSender,
}

impl AppBuilder for AppProps {
    type PathRouter = impl picoserve::routing::PathRouter;

    fn build_app(self) -> picoserve::Router<Self::PathRouter> {
        let gpio_cmd = self.gpio_cmd;
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
    }
}
    
    const WEB_TASK_POOL_SIZE: usize = 8;

#[embassy_executor::task]
async fn logger_task(usb: embassy_rp::Peri<'static, embassy_rp::peripherals::USB>) {
    let driver = embassy_rp::usb::Driver::new(usb, Irqs);
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
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

#[embassy_executor::task]
async fn gpio_task(
    mut led: Output<'static>,
    receiver: embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, GpioCommand, 4>,
) {
    loop {
        match receiver.receive().await {
            GpioCommand::LedOn => led.set_high(),
            GpioCommand::LedOff => led.set_low(),
        }
    }
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
        .start_ap_wpa2(
            example_secrets::WIFI_SSID,
            example_secrets::WIFI_PASSWORD,
            8,
        )
        .await;

    let led = Output::new(p.PIN_0, Level::Low);
    
    let (gpio_sender, gpio_receiver) = {
        let ch = make_static!(
            Channel<CriticalSectionRawMutex, GpioCommand, 4>,
            Channel::new()
        );
        (ch.sender(), ch.receiver())
    };

    spawner.must_spawn(gpio_task(led, gpio_receiver));

    let app = make_static!(AppRouter<AppProps>, AppProps { gpio_cmd: gpio_sender }.build_app());

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
}
