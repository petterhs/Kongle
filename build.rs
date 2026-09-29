//! Select memory layout: standalone (FLASH at 0x0000) by default, or MCUBoot (FLASH at 0x08200) with `--features mcuboot`.

use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());

    let memory_src = if env::var("CARGO_FEATURE_MCUBOOT").is_ok() {
        manifest_dir.join("memory_mcuboot.x")
    } else {
        manifest_dir.join("memory_standalone.x")
    };

    let memory_dst = out_dir.join("memory.x");
    fs::copy(memory_src, &memory_dst).expect("copy memory layout into OUT_DIR");

    // Emit OUT_DIR first so the linker finds our memory.x, not any file in the crate root.
    println!("cargo:rustc-link-search=native={}", out_dir.display());
    println!("cargo:rerun-if-changed=memory_standalone.x");
    println!("cargo:rerun-if-changed=memory_mcuboot.x");
    println!("cargo:rerun-if-env-changed=CARGO_FEATURE_MCUBOOT");
}
