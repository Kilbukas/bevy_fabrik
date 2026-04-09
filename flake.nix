{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    utils.url = "github:numtide/flake-utils";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    {
      self,
      nixpkgs,
      utils,
      rust-overlay,
    }:
    utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [
            (import rust-overlay)
          ];
        };

        rust-toolchain = pkgs.rust-bin.fromRustupToolchainFile ./rust-toolchain.toml;

        defaultAttrs = with pkgs; rec {
          nativeBuildInputs = [
            rust-toolchain
            pkg-config
          ];

          buildInputs = lib.optionals (pkgs.stdenv.isLinux) [
            alsa-lib

            vulkan-loader
            vulkan-tools

            libudev-zero
            xorg.libX11
            xorg.libXcursor
            xorg.libXi
            xorg.libXrandr
            libxkbcommon
            wayland
          ];

          packages = [
            rust-analyzer
            (rustfmt.override { asNightly = true; })
            cargo-watch
          ];

          LD_LIBRARY_PATH = lib.makeLibraryPath buildInputs;
        };

        default = pkgs.mkShell defaultAttrs;

      in
      {
        devShells = { inherit default; };
      }
    );
}
