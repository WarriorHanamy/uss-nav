# libleidenalg CMake config
set(LIBLEIDENALG_VERSION "0.10.0")
set(LIBLEIDENALG_FOUND TRUE)

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

set(LIBLEIDENALG_INCLUDEDIR "/usr/include/libleidenalg")
set(LIBLEIDENALG_LIBRARIES "/usr/lib/x86_64-linux-gnu/liblibleidenalg.a")

# Import targets
if(NOT TARGET libleidenalg)
  include("${CMAKE_CURRENT_LIST_DIR}/libleidenalgTargets.cmake")
  include("${CMAKE_CURRENT_LIST_DIR}/libleidenalgTargets-release.cmake" OPTIONAL)
endif()
