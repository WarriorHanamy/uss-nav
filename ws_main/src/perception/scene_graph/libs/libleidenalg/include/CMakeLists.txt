target_sources(libleidenalg
  PUBLIC
  FILE_SET headers
  TYPE HEADERS
  BASE_DIRS 
    ${CMAKE_CURRENT_SOURCE_DIR}
  FILES 
    Optimiser.h 
    GraphHelper.h 
    MutableVertexPartition.h
    
    ModularityVertexPartition.h
    SignificanceVertexPartition.h
    SurpriseVertexPartition.h

    ResolutionParameterVertexPartition.h
    LinearResolutionParameterVertexPartition.h    
    CPMVertexPartition.h
    RBConfigurationVertexPartition.h
    RBERVertexPartition.h

  )
