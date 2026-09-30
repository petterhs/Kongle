{
  pkgs,
  ...
}:
let
  defmtPrint = pkgs.callPackage ./nix/defmt-print.nix {};
in
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
    defmtPrint
  ];

  languages.rust = {
    enable = true;
    channel = "nightly";
    targets = [ "thumbv7em-none-eabihf" ];
  };
  languages.python.enable = true;

  # The old cargo-installed binary can outlive the Nix libraries it linked to.
  # A script takes precedence over cargo-install/bin and selects the rooted tool.
  scripts.defmt-print.exec = ''
    exec ${defmtPrint}/bin/defmt-print "$@"
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
