rock_library(envire_fcl
    SOURCES
      fcl-impl.cpp
      Collision.cpp
    HEADERS
      fcl-extern.hpp
      Collision.hpp
    DEPS_PKGCONFIG
        envire_maps
        fcl
        ccd
    )

rock_executable(envire_fcl_bin Main.cpp
    DEPS envire_fcl)

rock_executable(fcl_test fcl-test.cpp
   DEPS envire_fcl
   DEPS_PKGCONFIG
       octomap boost_serialization
         maps-viz base-viz
         envire_core
         envire_visualizer
         envire_maps
)

rock_executable(fcl_test_simple fcl-test-simple.cpp
   DEPS envire_fcl
   DEPS_PKGCONFIG
       octomap boost_serialization
         maps-viz base-viz
)


rock_executable(minimal test.cpp
   DEPS_PKGCONFIG fcl octomap boost_serialization)
   