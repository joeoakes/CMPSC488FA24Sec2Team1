{
  description = "Flake to set up basic arduino cli tool and some quick shortcuts";

  # Flake inputs
  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";

  # Flake outputs
  outputs = { self, nixpkgs }:
    let
      system = "x86_64-linux";
      pkgs = nixpkgs.legacyPackages.${system};
    in
    {
      devShells.${system}.default = pkgs.mkShell {
          # The Nix packages provided in the environment
          # Add any you need here
          packages = with pkgs; [ arduino-cli python3 python312Packages.pyserial ];

          # Set any environment variables for your dev shell
          env = { };

          # Add any shell logic you want executed any time the environment is activated
          shellHook = ''
          '';

          nativeBuildInputs = [
            (pkgs.writeScriptBin "build" ''
              #!/usr/bin/env zsh
              arduino-cli compile --fqbn esp32:esp32:m5stack-cores3 --build-path build'')
            (pkgs.writeScriptBin "upload" ''
              #!/usr/bin/env zsh
              arduino-cli upload . --port /dev/ttyACM0 --fqbn esp32:esp32:m5stack-cores3 --input-dir build'')
            (pkgs.writeScriptBin "monitor" ''
              #!/usr/bin/env zsh
              arduino-cli monitor -p /dev/ttyACM0 -b esp32:esp32:m5stack-cores3 -c 115200'')
          ];
        };
    };
}
