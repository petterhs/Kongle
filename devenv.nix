{
  pkgs,
  ...
}:

{
  packages = [
    pkgs.git
    pkgs.openocd
    pkgs.gcc-arm-embedded-13
    pkgs.probe-rs-tools
    pkgs.inetutils
    pkgs.cargo-edit
  ];

  languages.rust = {
    enable = true;
    channel = "nightly";
    targets = [ "thumbv7em-none-eabihf" ];
  };
  languages.python.enable = true;

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
}
