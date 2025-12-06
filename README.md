# Pico Hello - WiFi LED Control

A Raspberry Pi Pico W web server with HTTP-controlled GPIO LED. Demonstrates clean async patterns for embedded systems using Embassy.

## Features

- **WiFi Access Point** - Pico W broadcasts `pico_hello` WiFi AP
- **HTTP Web Server** - Serves HTML UI on port 80 (192.168.0.1)
- **LED Control** - HTTP endpoints to control GPIO0 LED
- **Web UI** - Styled HTML buttons to toggle LED on/off
- **USB Logger** - Debug logging via USB serial (embassy-usb-logger)
- **Serial Input** - UART0 reads external device data (9600 baud, GPIO17 RX)
- **Message-based Architecture** - Safe channel-based control (no unsafe code)

## Architecture

### Async Task Pattern for Embedded Systems

This project demonstrates clean resource ownership patterns in Embassy async code, inspired by [ScoreboardCtrl](https://github.com/ThomasRizzo/ScoreboardCtrl/blob/js-best-practices/src/main.rs):

```rust
spawner.must_spawn(logger_task(...));    // USB logging
spawner.must_spawn(wifi_task(...));      // WiFi driver
spawner.must_spawn(net_task(...));       // Network stack
spawner.must_spawn(read_serial(...));    // UART reader
spawner.must_spawn(web_task(...));       // HTTP server pool (8x)

let gpio = async {
    loop { /* LED control */ }
};
gpio.await;
```

**Key insight:** Different resource ownership models for different needs:
- `spawner.must_spawn()` for shared/stateful infrastructure (WiFi, network, serial I/O)
- Async blocks for exclusive peripheral control (GPIO, single-owner resources)

### Message Passing for GPIO Control

Instead of direct mutable access or unsafe code, use channels:

```rust
enum GpioCommand {
    LedOn,
    LedOff,
}

let (sender, receiver) = channel.split();

// HTTP handler
.route("/led/on", post(move || async move {
    let _ = sender.send(GpioCommand::LedOn).await;
    "LED ON"
}))

// GPIO task
let gpio = async {
    loop {
        match receiver.receive().await {
            GpioCommand::LedOn => led.set_high(),
            GpioCommand::LedOff => led.set_low(),
        }
    }
};
```

**Benefits:**
- No unsafe code
- No locks on hot path (HTTP handler just sends message)
- Clear separation: handlers signal intent, task executes
- Easy to extend: add more commands, more tasks

### Serial Data Handling

Serial input follows the pattern from ScoreboardCtrl: the UART task updates shared state via a `Mutex`, which other tasks can read:

```rust
/// Shared state updated by serial task
#[derive(Clone, Copy)]
struct SerialState {
    last_byte: u8,
    count: u32,
}

#[derive(Clone, Copy)]
struct SharedSerialState(&'static Mutex<CriticalSectionRawMutex, SerialState>);

// Serial task reads UART and updates state
#[embassy_executor::task]
async fn read_serial(mut rx: UartRx<'static, uart::Async>, state: SharedSerialState) -> ! {
    let mut buf = [0; 1];
    loop {
        if let Ok(_) = rx.read(&mut buf).await {
            let mut s = state.0.lock().await;  // Writer has exclusive lock
            s.last_byte = buf[0];
            s.count = s.count.saturating_add(1);
        }
    }
}

// Other tasks can read the state via lock
let s = serial_state.0.lock().await;
log::info!("Last byte: {}, count: {}", s.last_byte, s.count);
```

**Benefits:**
- Single writer (serial task), multiple readers (HTTP handlers, logging)
- No channels or message passing needed for read-only access
- Efficient: readers only lock when they need the data
- Scales to many readers naturally

## Lessons Learned

### 1. Avoiding Unsafe in Embedded Code

❌ **Initial approach:** Raw pointers and unsafe dereferencing
- Compiles but unsafe, error-prone
- No compile-time guarantees about exclusive access

❌ **Second approach:** `Output<'static>` behind `&` references  
- Requires unsafe to mutate through `&self`
- Still unsafe, just hidden

✅ **Final approach:** Message passing with channels
- Safe, idiomatic embassy code
- Rust's type system ensures exclusive resource access

### 2. Async Blocks Own Resources

In Embassy, async blocks naturally take ownership:

```rust
let led = Output::new(...);
let receiver = channel.receiver();

let gpio = async {
    // owns led and receiver - no parameters needed
    // compiles without unsafe
};
```

This is cleaner than passing references through task spawning.

### 3. Runner Ownership Constraints

WiFi and network runners can't be shared between async blocks:
- Each needs exclusive ownership
- `spawner.must_spawn()` is required for them
- Use spawner for infrastructure, async blocks for peripherals

### 4. Shared State via Mutex vs Message Passing

The project combines both patterns for different scenarios:

```rust
// Hardware tasks that update shared state
spawner.must_spawn(read_serial(rx, state));  // Single writer
spawner.must_spawn(web_task(...));           // Multiple readers of state
spawner.must_spawn(wifi_task(...));          // Stateful infrastructure

// Exclusive resource control via channels
let (sender, receiver) = gpio_channel.split();

let gpio = async {
    loop {
        match receiver.receive().await {
            GpioCommand::LedOn => led.set_high(),
            GpioCommand::LedOff => led.set_low(),
        }
    }
};
```

**Mutex (read-heavy data):** Serial state, scoreboard time, device status  
**Channels (commands/events):** LED control, motor commands, state transitions

## Dependencies

### Core Runtime
- **embassy-executor** - Async task executor for ARM Cortex-M
- **embassy-time** - Timer and sleep primitives
- **embassy-futures** - Utilities for joining multiple async tasks
- **cortex-m-rt** - ARM runtime for bare metal

### Hardware
- **embassy-rp** - Raspberry Pi RP2040 HAL
- **cyw43** - CYW43 WiFi chip driver
- **cyw43-pio** - SPI via PIO state machines (DMA-less)

### Networking
- **embassy-net** - Async network stack (TCP/IP)
- **picoserve** - Lightweight HTTP server for embedded systems
- **embedded-io-async** - Async I/O traits

### Synchronization & State
- **embassy-sync** - Async channels, mutexes
- **static_cell** - Type-safe static initialization
- **portable-atomic** - Platform-independent atomics

### Utilities
- **log** - Structured logging facade
- **embassy-usb-logger** - USB serial logging
- **panic-persist** - Persist panics in memory for debugging
- **rand** - Random number generation (for network config)

## Build & Run

```bash
cargo build --release
# Flash to Pico W
```

## Project Structure

- `src/main.rs` - Embassy async application (~240 lines)
  - Infrastructure tasks: logger, WiFi, network, HTTP server pool, UART reader
  - Shared state: `SerialState` via Mutex for read-heavy access patterns
  - Peripheral control: GPIO LED via async block with channel-based messaging
  - HTTP routing for LED on/off endpoints
- `index.html` - Responsive web UI for LED control
- `AGENTS.md` - AI agent instructions and issue tracking guide

## API

### HTTP Endpoints

```
GET /              - Serve index.html (LED control UI)
POST /led/on       - Turn LED on
POST /led/off      - Turn LED off
```

### Internal Data Structures

**GpioCommand** - Commands sent via channel from HTTP handlers to GPIO task:
```rust
enum GpioCommand {
    LedOn,
    LedOff,
}
```

**SerialState** - Shared state updated by UART reader, read by any task:
```rust
#[derive(Clone, Copy)]
struct SerialState {
    last_byte: u8,
    count: u32,
}
```

### Hardware Configuration

**GPIO:**
- LED control: GPIO0 (PIN_0), controlled via GpioCommand channel

**UART0 Serial Input:**
- Baud rate: 9600
- RX pin: GPIO17 (PIN_17)
- DMA: Channel 1
- Updates `SerialState` continuously with each received byte

## References

- [Embassy docs](https://embassy.dev/)
- [picoserve](https://github.com/sammysung/picoserve)
- [Pico W docs](https://www.raspberrypi.com/documentation/microcontrollers/rp2040.html)
- [CYW43 driver](https://github.com/embassy-rs/embassy/tree/main/cyw43)
