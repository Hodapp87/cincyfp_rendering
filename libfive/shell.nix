{ pkgs ? import <nixpkgs> {} }:

let stdenv = pkgs.stdenv;
    libfive = stdenv.mkDerivation rec {

      name = "libfive-${version}";
      version = "7a46e316";

      src = pkgs.fetchFromGitHub {
        owner  = "libfive";
        repo   = "libfive";
        rev    = "7a46e316ecc755bb3597f4f3ca84e7ea9a8e75b8";
        sha256 = "1cjvmfsjbc3k5zfp22zccmjx0b2rmm2a0vm555f3bqj81yw9v7ps";
      };  
      buildInputs = with pkgs; [ cmake pkgconfig eigen3_3 zlib libpng
        boost qt5.qtimageformats guile ];

      enableParallelBuilding = true;
  
      meta = with stdenv.lib; {
        description = "Infrastructure for solid modeling";
        homepage = https://libfive.com/;
        #maintainers = with maintainers; [ abbradar ];
        license = licenses.lpgl2;
        platforms = platforms.linux;
      };
    };
in stdenv.mkDerivation rec {
  name = "libfive";

  buildInputs = [ libfive ];
}
