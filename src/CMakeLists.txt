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

rock_executable(fcl_collidable_test fcl-collidable-test.cpp
   DEPS envire_fcl
   DEPS_PKGCONFIG
       octomap boost_serialization
         maps-viz base-viz
         envire_core
         envire_visualizer
         envire_maps
         envire_smurf
)

#rock_executable(fcl_collidable_test_sphere fcl-collidable-test-sphere.cpp
#   DEPS envire_fcl
#   DEPS_PKGCONFIG
#       octomap boost_serialization
#         maps-viz base-viz
#         envire_core
#         envire_visualizer
#         envire_maps
#         envire_smurf
#)
#
#rock_executable(fcl_collidable_test_multiple_objects fcl-collidable-test-multiple-objects.cpp
#   DEPS envire_fcl
#   DEPS_PKGCONFIG
#       octomap boost_serialization
#         maps-viz base-viz
#         envire_core
#         envire_visualizer
#         envire_maps
#         envire_smurf
#)
#
#rock_executable(fcl_collidable_test_robot fcl-collidable-test-robot.cpp
#   DEPS envire_fcl
#   DEPS_PKGCONFIG
#       octomap boost_serialization
#         maps-viz base-viz
#         envire_core
#         envire_visualizer
#         envire_maps
#         envire_smurf
#)
#
rock_executable(fcl_collidable_test_robot_precalculated fcl-collidable-test-robot-precalculated.cpp
   DEPS envire_fcl
   DEPS_PKGCONFIG
       octomap boost_serialization
         maps-viz base-viz
         envire_core
         envire_visualizer
         envire_maps
         envire_smurf
)

rock_executable(fcl_collidable_test_robot_crater_precalculated fcl-collidable-test-robot-crater-precalculated.cpp
   DEPS envire_fcl
   DEPS_PKGCONFIG
       octomap boost_serialization
         maps-viz base-viz
         envire_core
         envire_visualizer
         envire_maps
         envire_smurf
)

file(COPY smurf DESTINATION ./)

rock_executable(fcl_test_simple fcl-test-simple.cpp
   DEPS envire_fcl
   DEPS_PKGCONFIG
       octomap boost_serialization
         maps-viz base-viz
)


rock_executable(minimal test.cpp
   DEPS_PKGCONFIG fcl octomap boost_serialization)
   
