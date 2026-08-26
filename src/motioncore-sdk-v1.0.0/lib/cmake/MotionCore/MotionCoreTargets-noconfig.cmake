#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "MotionCore::motioncore" for configuration ""
set_property(TARGET MotionCore::motioncore APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(MotionCore::motioncore PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libmotioncore_cpp.so"
  IMPORTED_SONAME_NOCONFIG "libmotioncore_cpp.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS MotionCore::motioncore )
list(APPEND _IMPORT_CHECK_FILES_FOR_MotionCore::motioncore "${_IMPORT_PREFIX}/lib/libmotioncore_cpp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
