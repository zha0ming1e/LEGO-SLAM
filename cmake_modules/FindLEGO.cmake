# find the header files
find_path(LEGO_INCLUDE_DIR lego/base/base_vertex.h
  ${LEGO_ROOT}/include
  $ENV{LEGO_ROOT}/include
  $ENV{LEGO_ROOT}
  /usr/local/include
  /usr/include
  /opt/local/include
  /sw/local/include
  /sw/include
  NO_DEFAULT_PATH
  )

# macro to unify finding both the debug and release versions of the
# libraries; this is adapted from the OpenSceneGraph FIND_LIBRARY
# macro.
macro(FIND_LEGO_LIBRARY MYLIBRARY MYLIBRARYNAME)

  find_library("${MYLIBRARY}_DEBUG"
    NAMES "lego_${MYLIBRARYNAME}_d"
    PATHS
    ${LEGO_ROOT}/lib/Debug
    ${LEGO_ROOT}/lib
    $ENV{LEGO_ROOT}/lib/Debug
    $ENV{LEGO_ROOT}/lib
    NO_DEFAULT_PATH
    )

  find_library("${MYLIBRARY}_DEBUG"
    NAMES "lego_${MYLIBRARYNAME}_d"
    PATHS
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64
    /opt/local/lib
    /sw/local/lib
    /sw/lib
    )
  
  find_library(${MYLIBRARY}
    NAMES "lego_${MYLIBRARYNAME}"
    PATHS
    ${LEGO_ROOT}/lib/Release
    ${LEGO_ROOT}/lib
    $ENV{LEGO_ROOT}/lib/Release
    $ENV{LEGO_ROOT}/lib
    NO_DEFAULT_PATH
    )

  find_library(${MYLIBRARY}
    NAMES "lego_${MYLIBRARYNAME}"
    PATHS
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64
    /opt/local/lib
    /sw/local/lib
    /sw/lib
    )
  
  if(NOT ${MYLIBRARY}_DEBUG)
    if(MYLIBRARY)
      set(${MYLIBRARY}_DEBUG ${MYLIBRARY})
    endif(MYLIBRARY)
  endif( NOT ${MYLIBRARY}_DEBUG)
  
endmacro(FIND_LEGO_LIBRARY LIBRARY LIBRARYNAME)

# find the base elements
FIND_LEGO_LIBRARY(LEGO_BASE_LIBRARY base)
FIND_LEGO_LIBRARY(LEGO_SLAM_LIBRARY slam)

# LEGO itself declared found if we found the base and slam libraries
set(LEGO_FOUND "NO")
if(LEGO_BASE_LIBRARY AND LEGO_SLAM_LIBRARY)
  set(LEGO_FOUND "YES")
endif(LEGO_BASE_LIBRARY AND LEGO_SLAM_LIBRARY)
