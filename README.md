# Kongle

Basic Rust firmware for PineTime by Pine64

```bash
rustup target add thumbv7em-none-eabihf
```

## Debug

Terminals:

  1. `sudo bash ./debug.sh`
  2. `cargo run --release`
  3. `nc localhost 6969 | defmt-print -e target/thumbv7em-none-eabihf/release/kongle`