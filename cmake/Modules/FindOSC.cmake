INCLUDE(FindPkgConfig)
#PKG_CHECK_MODULES(PC_OSC osc)

FIND_PATH(
    OSC_INCLUDE_DIRS
    NAMES oscpack/osc/OscPacketListener.h
    HINTS $ENV{OSC_DIR}/include
        ${PC_OSC_INCLUDEDIR}
    PATHS /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    OSC_LIBRARIES
    NAMES oscpack
    HINTS $ENV{SOC_DIR}/lib
        ${PC_OSC_LIBDIR}
    PATHS /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          /usr/x86_64-linux-gnu
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(OSC DEFAULT_MSG OSC_LIBRARIES OSC_INCLUDE_DIRS)
