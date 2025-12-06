# Pico Hello - WiFi LED Control

A Raspberry Pi Pico W web server with HTTP-controlled GPIO LED.

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

## Build & Run

```bash
cargo build --release
# Flash to Pico
```

## Project Structure

- `src/main.rs` - Embassy app with async blocks for GPIO control
- `index.html` - Web UI with LED on/off buttons
- `.beads/issues.jsonl` - Issue tracking (see AGENTS.md)

## References

- [Embassy docs](https://embassy.dev/)
- [picoserve](https://github.com/sammysung/picoserve)
- [Pico W docs](https://www.raspberrypi.com/documentation/microcontrollers/rp2040.html)
