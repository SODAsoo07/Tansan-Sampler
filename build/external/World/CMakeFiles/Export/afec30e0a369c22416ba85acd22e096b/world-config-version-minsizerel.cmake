#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "world::world" for configuration "MinSizeRel"
set_property(TARGET world::world APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(world::world PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_MINSIZEREL "CXX"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/lib/world.lib"
  )

list(APPEND _cmake_import_check_targets world::world )
list(APPEND _cmake_import_check_files_for_world::world "${_IMPORT_PREFIX}/lib/world.lib" )

# Import target "world::world_tool" for configuration "MinSizeRel"
set_property(TARGET world::world_tool APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(world::world_tool PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_MINSIZEREL "CXX"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/lib/world_tool.lib"
  )

list(APPEND _cmake_import_check_targets world::world_tool )
list(APPEND _cmake_import_check_files_for_world::world_tool "${_IMPORT_PREFIX}/lib/world_tool.lib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
