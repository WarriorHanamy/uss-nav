set(igraph_FOUND TRUE)
set(IGRAPH_VERSION 0.10.13)
set(igraph_INCLUDE_DIRS /usr/include/igraph)
set(igraph_LIBRARIES igraph)
set(igraph_LIBRARY_DIRS /usr/lib/x86_64-linux-gnu)
get_filename_component(_igraph_lib igraph HINTS /usr/lib/x86_64-linux-gnu NO_DEFAULT_PATH)
if(NOT TARGET igraph)
  add_library(igraph UNKNOWN IMPORTED)
  set_target_properties(igraph PROPERTIES
    IMPORTED_LOCATION "${_igraph_lib}"
    INTERFACE_INCLUDE_DIRECTORIES "/usr/include/igraph"
  )
endif()
