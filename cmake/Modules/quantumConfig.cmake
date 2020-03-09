INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_QUANTUM quantum)

FIND_PATH(
    QUANTUM_INCLUDE_DIRS
    NAMES quantum/api.h
    HINTS $ENV{QUANTUM_DIR}/include
        ${PC_QUANTUM_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    QUANTUM_LIBRARIES
    NAMES gnuradio-quantum
    HINTS $ENV{QUANTUM_DIR}/lib
        ${PC_QUANTUM_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          )

include("${CMAKE_CURRENT_LIST_DIR}/quantumTarget.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(QUANTUM DEFAULT_MSG QUANTUM_LIBRARIES QUANTUM_INCLUDE_DIRS)
MARK_AS_ADVANCED(QUANTUM_LIBRARIES QUANTUM_INCLUDE_DIRS)

