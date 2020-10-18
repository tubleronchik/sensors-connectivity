{ nixpkgs ? import (builtins.fetchTarball https://github.com/airalab/airapkgs/archive/e88793a849dcff8b1e07d33e09541f16f443df04.tar.gz)
, system ? builtins.currentSystem
}:

let
  pkgs = nixpkgs { inherit system; };
in rec {
  package = pkgs.callPackage ./default.nix {  };
}
