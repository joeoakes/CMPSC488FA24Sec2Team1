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
          packages = with pkgs; [ arduino-cli ];

          # Set any environment variables for your dev shell
          env = { };

          # Add any shell logic you want executed any time the environment is activated
          shellHook = ''
          '';

          nativeBuildInputs = [
            (pkgs.writeScriptBin "build" ''
              #!/usr/bin/env zsh
              arduino-cli compile --fqbn arduino:avr:mega 6502-monitor'')
            (pkgs.writeScriptBin "upload" ''
              #!/usr/bin/env zsh
              arduino-cli upload --port /dev/ttyACM0 --fqbn arduino:avr:mega 6502-monitor'')
          ];
        };
    };
}
