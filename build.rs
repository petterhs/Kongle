//! Select memory layout: standalone (FLASH at 0x0000) by default, or MCUBoot (FLASH at 0x08200) with `--features mcuboot`.

use std::env;
use std::fs;
use std::path::PathBuf;
use std::process::Command;

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
    // A commit advances the branch ref without changing .git/HEAD. Track both
    // paths so incremental builds do not keep advertising an old revision.
    let head_ref = Command::new("git")
        .args(["symbolic-ref", "-q", "HEAD"])
        .current_dir(&manifest_dir)
        .output()
        .ok()
        .filter(|output| output.status.success())
        .and_then(|output| String::from_utf8(output.stdout).ok());
    for path in [Some("HEAD"), head_ref.as_deref().map(str::trim)]
        .into_iter()
        .flatten()
    {
        if let Ok(output) = Command::new("git")
            .args(["rev-parse", "--git-path", path])
            .current_dir(&manifest_dir)
            .output()
        {
            if output.status.success() {
                if let Ok(git_path) = String::from_utf8(output.stdout) {
                    println!("cargo:rerun-if-changed={}", git_path.trim());
                }
            }
        }
    }
    let revision = Command::new("git")
        .args(["rev-parse", "--short=7", "HEAD"])
        .current_dir(&manifest_dir)
        .output()
        .ok()
        .filter(|output| output.status.success())
        .and_then(|output| String::from_utf8(output.stdout).ok())
        .map(|sha| sha.trim().to_owned())
        .filter(|sha| !sha.is_empty())
        .unwrap_or_else(|| "unknown".to_owned());
    println!("cargo:rustc-env=KONGLE_BUILD_ID={revision}");
}
