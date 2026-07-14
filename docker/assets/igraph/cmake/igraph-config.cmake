# igraph CMake config
set(IGRAPH_VERSION "0.10.13")
set(IGRAPH_INTEGER_SIZE 64)
set(IGRAPH_GLPK_SUPPORT ON)
set(IGRAPH_GRAPHML_SUPPORT ON)

set(igraph_FOUND TRUE)
set(igraph_INCLUDE_DIRS "/usr/include/igraph")
set(igraph_LIBRARIES "igraph")
set(igraph_LIBRARY_DIRS "/usr/lib/x86_64-linux-gnu")

if(NOT TARGET igraph)
  add_library(igraph STATIC IMPORTED)
  set_target_properties(igraph PROPERTIES
    IMPORTED_LOCATION "/usr/lib/x86_64-linux-gnu/libigraph.a"
    INTERFACE_INCLUDE_DIRECTORIES "/usr/include/igraph"
  )
endif()
