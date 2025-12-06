# Pico Hello - WiFi LED Control

A Raspberry Pi Pico W web server with HTTP-controlled GPIO LED. Demonstrates clean async patterns for embedded systems using Embassy.

## Features

- **WiFi Access Point** - Pico W broadcasts `pico_hello` WiFi AP
- **HTTP Web Server** - Serves HTML UI on port 80 (192.168.0.1)
- **LED Control** - HTTP endpoints to control GPIO0 LED
- **Web UI** - Styled HTML buttons to toggle LED on/off
- **USB Logger** - Debug logging via USB serial (embassy-usb-logger)
- **Message-based GPIO** - Safe channel-based control (no unsafe code)

## Architecture

### Async Task Pattern for Embedded Systems

This project demonstrates clean resource ownership patterns in Embassy async code:

```rust
let infrastructure_task = spawner.must_spawn(...);  // WiFi, net, web servers
let peripheral_task = async {
    loop { /* own and control GPIO */ }
};
gpio_task.await;
```

**Key insight:** Each async block can own its peripherals directly. Use:
- `spawner.must_spawn()` for shared infrastructure (WiFi, networking)
- Async blocks for peripheral-specific tasks (GPIO, sensors, etc.)

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

### 4. Scalability Pattern

Easy to add more peripheral tasks:

```rust
let gpio = async { /* led control */ };
let button = async { /* button logic */ };
let sensor = async { /* temperature */ };

join3(gpio, button, sensor).await;
```

Each task owns its peripherals, no coordination needed.

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

- `src/main.rs` - Embassy async application (144 lines)
  - Infrastructure tasks (WiFi, network, web servers) spawned concurrently
  - GPIO control via async block with message passing
  - picoserve HTTP routing with State extraction
- `index.html` - Responsive web UI for LED control
- `.beads/issues.jsonl` - Issue tracking (see AGENTS.md)
- `AGENTS.md` - AI agent instructions and issue tracking guide

## API

### HTTP Endpoints

```
GET /              - Serve index.html (LED control UI)
POST /led/on       - Turn LED on
POST /led/off      - Turn LED off
```

### GpioCommand Enum

```rust
enum GpioCommand {
    LedOn,
    LedOff,
}
```

Messages sent via embassy_sync::channel between HTTP handlers and GPIO task.

## References

- [Embassy docs](https://embassy.dev/)
- [picoserve](https://github.com/sammysung/picoserve)
- [Pico W docs](https://www.raspberrypi.com/documentation/microcontrollers/rp2040.html)
- [CYW43 driver](https://github.com/embassy-rs/embassy/tree/main/cyw43)
