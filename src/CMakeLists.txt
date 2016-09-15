rock_library(envire_fcl
    SOURCES
      Collision.cpp
      fcl-impl.cpp
    HEADERS
      Collision.hpp
      mls/mls.h
      mls/mls-inl.h
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
)

rock_executable(fcl_test_simple fcl-test-simple.cpp
   DEPS envire_fcl
   DEPS_PKGCONFIG
       octomap boost_serialization
         maps-viz base-viz
)


rock_executable(minimal test.cpp
   DEPS_PKGCONFIG fcl octomap boost_serialization)
   