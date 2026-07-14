# Import target "libleidenalg" for configuration "Release"
set_property(TARGET libleidenalg APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(libleidenalg PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "/usr/lib/x86_64-linux-gnu/liblibleidenalg.a"
)
