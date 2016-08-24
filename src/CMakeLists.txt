rock_library(envire_fcl
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(envire_fcl_bin Main.cpp
    DEPS envire_fcl)

