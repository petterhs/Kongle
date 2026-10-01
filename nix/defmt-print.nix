{ lib, rustPlatform, fetchCrate }:

rustPlatform.buildRustPackage rec {
  pname = "defmt-print";
  version = "1.0.0";

  src = fetchCrate {
    inherit pname version;
    hash = "sha256-rio5kAL6NR7vBtjPF0GxDcINeWw+LuZWe7nFN0UkdBg=";
  };
  cargoHash = "sha256-whKaubZRJTHxS/eULsnpYpM+VQPIuofPZtsb29/gIUk=";

  meta = {
    description = "Decode defmt logs from stdin using the matching firmware ELF";
    homepage = "https://github.com/knurling-rs/defmt";
    license = with lib.licenses; [ mit asl20 ];
    mainProgram = "defmt-print";
  };
}
