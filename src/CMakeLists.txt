rock_library(envire_fcl
    SOURCES
      fcl-impl.cpp
      Collision.cpp
    HEADERS
      fcl-extern.hpp
      Collision.hpp
    DEPS_PKGCONFIG
        fcl
        envire_maps
        smurf
        ccd
    )


