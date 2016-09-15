# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)
find_package(Rock)
set(ROCK_TEST_ENABLED ON)


rock_init(envire_fcl 0.1)

# fcl generates lots of annoying non-virtual-dtor warnings
# confer: https://github.com/flexible-collision-library/fcl/issues/178
remove_definitions(-Wnon-virtual-dtor)
rock_standard_layout()

