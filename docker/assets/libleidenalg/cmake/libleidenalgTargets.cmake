# Create imported target libleidenalg
add_library(libleidenalg STATIC IMPORTED)

set_target_properties(libleidenalg PROPERTIES
  IMPORTED_LOCATION "/usr/lib/x86_64-linux-gnu/liblibleidenalg.a"
  INTERFACE_INCLUDE_DIRECTORIES "/usr/include/libleidenalg"
  INTERFACE_LINK_LIBRARIES "igraph"
)
