# ScoreboardCtrl - Pico W Scoreboard Controller

A Raspberry Pi Pico W web server with HTTP-controlled GPIO LED and UART scoreboard packet parsing. Demonstrates clean async patterns for embedded systems using Embassy.

## Features

- **WiFi Access Point** - Pico W broadcasts `Scoreboard` open WiFi AP
- **HTTP Web Server** - Serves HTML UI on port 80 (192.168.0.1)
- **LED Control** - HTTP endpoints to control GPIO0 LED
- **Web UI** - Styled HTML buttons to toggle LED on/off
- **USB Logger** - Debug logging via USB serial (embassy-usb-logger)
- **Serial Input** - UART0 reads scoreboard packets (9600 baud, GPIO17 RX)
- **Packet Parsing** - Extracts time (min/sec) from 6-byte scoreboard packets with sync byte framing
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

Serial input follows the pattern from ScoreboardCtrl: the UART task parses packets and updates shared state via a `Mutex`, which other tasks can read without locks during normal operation:

**Packet Structure (6 bytes):**
- Byte 0: 0x00 (sync marker)
- Byte 1: Minutes
- Byte 2: Seconds
- Byte 3: Shotclock
- Byte 4: 0x3F (reserved)
- Byte 5: CRC

**Parsing Strategy:**
1. Buffer single bytes from UART
2. When sync byte (0x00`) found, start accumulating packet
3. Once 6 bytes received, extract min/sec
4. Update Mutex-wrapped state and log only on change (not every byte)

```rust
/// Shared state updated by serial task
#[derive(Clone, Copy, PartialEq)]
struct SerialState {
    minutes: u8,
    seconds: u8,
    packet_count: u32,
}

// Serial task reads, buffers, and parses packets
#[embassy_executor::task]
async fn read_serial(mut rx: UartRx<'static, uart::Async>, state: SharedSerialState) -> ! {
    let mut packet_buf = [0u8; 6];
    let mut buf_idx = 0;
    
    loop {
        let mut byte_buf = [0; 1];
        if let Ok(_) = rx.read(&mut byte_buf).await {
            let byte = byte_buf[0];
            
            // Sync on 0x00, then buffer remaining 5 bytes
            if byte == 0x00 {
                buf_idx = 0;
                packet_buf[0] = byte;
                buf_idx = 1;
            } else if buf_idx > 0 {
                packet_buf[buf_idx] = byte;
                buf_idx += 1;
                
                // Full packet: extract time, update state, log on change only
                if buf_idx == 6 {
                    let minutes = packet_buf[1];
                    let seconds = packet_buf[2];
                    let mut s = state.0.lock().await;
                    
                    if s.minutes != minutes || s.seconds != seconds {
                        s.minutes = minutes;
                        s.seconds = seconds;
                        s.packet_count += 1;
                        log::info!("Time: {}:{:02}", minutes, seconds);
                    }
                    buf_idx = 0;
                }
            }
        }
    }
}

// Other tasks read the state via lock
let s = serial_state.0.lock().await;
log::info!("Current time: {}:{:02}", s.minutes, s.seconds);
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

- `src/main.rs` - Embassy async application (~250 lines)
  - Infrastructure tasks: logger, WiFi, network, HTTP server pool, UART reader
  - Shared state: `SerialState` via Mutex for scoreboard time (min/sec)
  - Packet parsing: Sync byte detection, 6-byte frame extraction, time value updates
  - Peripheral control: GPIO LED via async block with channel-based messaging
  - HTTP routing for LED on/off endpoints
- `index.html` - Responsive web UI for LED control
- `AGENTS.md` - AI agent instructions and issue tracking guide
- `Cargo.toml` - Rust dependencies (embassy, picoserve, cyw43, etc.)

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
#[derive(Clone, Copy, PartialEq)]
struct SerialState {
    minutes: u8,        // Extracted from packet byte 1
    seconds: u8,        // Extracted from packet byte 2
    packet_count: u32,  // Count of valid packets received
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

## Development

This project was developed with AI assistance from **Amp** (Sourcegraph's AI coding agent). Key contributions include:

- Async task architecture design following Embassy patterns
- Packet parsing implementation for serial data
- Clean separation of concerns (infrastructure tasks, shared state, exclusive resources)
- Documentation and README patterns

The codebase combines human direction with AI implementation, emphasizing safety (no unsafe code), clarity, and idiomatic Rust/Embassy patterns.

## References

- [Embassy docs](https://embassy.dev/)
- [picoserve](https://github.com/sammysung/picoserve)
- [Pico W docs](https://www.raspberrypi.com/documentation/microcontrollers/rp2040.html)
- [CYW43 driver](https://github.com/embassy-rs/embassy/tree/main/cyw43)
