# FindWujiSdk.cmake — locate libwuji_sdk_c and wuji_sdk.h
#
# Search order:
#   1. WUJI_SDK_ROOT env / -DWUJI_SDK_ROOT=
#   2. CMAKE_PREFIX_PATH / typical install prefixes
#
# Sets: WujiSdk_FOUND, WujiSdk_INCLUDE_DIRS, WujiSdk_LIBRARIES

if(DEFINED ENV{WUJI_SDK_ROOT} AND NOT WUJI_SDK_ROOT)
  set(WUJI_SDK_ROOT "$ENV{WUJI_SDK_ROOT}")
endif()

find_path(WujiSdk_INCLUDE_DIR
  NAMES wuji_sdk.h
  HINTS
    ${WUJI_SDK_ROOT}
    ${WUJI_SDK_ROOT}/include
    /usr/local/include
    /opt/wuji/include
  PATH_SUFFIXES include
)

find_library(WujiSdk_LIBRARY
  NAMES wuji_sdk_c libwuji_sdk_c
  HINTS
    ${WUJI_SDK_ROOT}
    ${WUJI_SDK_ROOT}/lib
    /usr/local/lib
    /opt/wuji/lib
  PATH_SUFFIXES lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(WujiSdk
  REQUIRED_VARS WujiSdk_LIBRARY WujiSdk_INCLUDE_DIR
)

if(WujiSdk_FOUND)
  set(WujiSdk_INCLUDE_DIRS ${WujiSdk_INCLUDE_DIR})
  set(WujiSdk_LIBRARIES ${WujiSdk_LIBRARY})
  if(NOT TARGET WujiSdk::wuji_sdk_c)
    add_library(WujiSdk::wuji_sdk_c UNKNOWN IMPORTED)
    set_target_properties(WujiSdk::wuji_sdk_c PROPERTIES
      IMPORTED_LOCATION "${WujiSdk_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${WujiSdk_INCLUDE_DIR}"
    )
  endif()
endif()

mark_as_advanced(WujiSdk_INCLUDE_DIR WujiSdk_LIBRARY)
