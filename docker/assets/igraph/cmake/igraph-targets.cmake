# Create imported target igraph
add_library(igraph STATIC IMPORTED)
set_target_properties(igraph PROPERTIES
  IMPORTED_LOCATION "/usr/lib/x86_64-linux-gnu/libigraph.a"
  INTERFACE_INCLUDE_DIRECTORIES "/usr/include/igraph"
)
