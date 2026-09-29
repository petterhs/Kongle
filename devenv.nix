{
  pkgs,
  ...
}:

{
  packages = [
    pkgs.git
    pkgs.gh
    pkgs.openocd
    pkgs.gcc-arm-embedded-13
    pkgs.probe-rs-tools
    pkgs.inetutils
    pkgs.cargo-edit
    pkgs.clang-tools
    pkgs.libclang
    pkgs.mcuboot-imgtool
  ];

  languages.rust = {
    enable = true;
    channel = "nightly";
    targets = [ "thumbv7em-none-eabihf" ];
  };
  languages.python.enable = true;

  # defmt-print is not packaged in the pinned nixpkgs. Install a locked host
  # tool explicitly, rather than relying on an old binary in the local state.
  scripts.install-debug-tools.exec = ''
    cargo install defmt-print --version 1.0.0 --locked --target ${pkgs.stdenv.hostPlatform.rust.rustcTarget}
  '';

  # https://devenv.sh/tasks/
  tasks = {
    "kongle:openocd" = {
      exec = "sudo openocd -f interface/stlink.cfg -f target/nrf52.cfg";
    };
    "kongle:run" = {
      exec = "cargo run --release";
    };
  };

  # https://devenv.sh/tests/
  enterTest = ''
    echo "Running tests"
    git --version | grep --color=auto "${pkgs.git.version}"
  '';

  env = {
    LIBCLANG_PATH = "${pkgs.libclang.lib}/lib";
    BINDGEN_EXTRA_CLANG_ARGS = "--sysroot=${pkgs.gcc-arm-embedded-13}/arm-none-eabi";
  };
}
