# Import target "igraph" for configuration "Release"
set_property(TARGET igraph APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(igraph PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "/usr/lib/x86_64-linux-gnu/libigraph.a"
)
