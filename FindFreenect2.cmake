# - Find Freenect2 alias libfreenect2
# This module finds an installed Freenect2 package.
#
# It sets the following variables:
#  Freenect2_FOUND       - Set to false, or undefined, if Freenect2 isn't found.
#  Freenect2_INCLUDE_DIRS - The Freenect2 include directory.
#  Freenect2_LIBRARIES     - The Freenect2 library to link against.
#  Freenect2_DASH_INCLUDES  - If includes have "-" instead of "_" in their names. New libfreenect2 has "_".

FIND_PATH(Freenect2_SUB_INCLUDE_DIRS registration.h PATH_SUFFIXES libfreenect2)
MESSAGE(STATUS "Freenect2_SUB_INCLUDE_DIRS: ${Freenect2_SUB_INCLUDE_DIRS}")
# include dir is one level higher
# so header files can be included with #include "libfreenect2/libfreenect2.hpp"
CMAKE_PATH(GET Freenect2_SUB_INCLUDE_DIRS PARENT_PATH Freenect2_INCLUDE_DIRS)
MESSAGE(STATUS "Freenect2_INCLUDE_DIRS: ${Freenect2_INCLUDE_DIRS}")

FIND_LIBRARY(Freenect2_LIBRARY NAMES freenect2)
MESSAGE(STATUS "Freenect2_LIBRARY: ${Freenect2_LIBRARY}")

FIND_FILE(tmp color-settings.h PATH_SUFFIXES libfreenect2)
SET(Freenect2_DASH_INCLUDES 0)
IF(tmp)
  SET(Freenect2_DASH_INCLUDES 1)
ENDIF(tmp)

IF (Freenect2_INCLUDE_DIRS AND Freenect2_LIBRARY)
   SET(Freenect2_FOUND TRUE)
ENDIF (Freenect2_INCLUDE_DIRS AND Freenect2_LIBRARY)

IF (Freenect2_FOUND)
   # show which Freenect2 was found only if not quiet
   SET(Freenect2_LIBRARIES ${Freenect2_LIBRARY})
   IF (NOT Freenect2_FIND_QUIETLY)
      MESSAGE(STATUS "Found Freenect2: ${Freenect2_LIBRARIES}")
   ENDIF (NOT Freenect2_FIND_QUIETLY)
ELSE (Freenect2_FOUND)
   # fatal error if Freenect2 is required but not found
   IF (Freenect2_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find Freenect2 (libfreenect2)")
   ENDIF (Freenect2_FIND_REQUIRED)
ENDIF (Freenect2_FOUND)

