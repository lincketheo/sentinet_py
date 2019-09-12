
set(WII_CONT_VERS "release")
set(WII_CONT_SUFFIX "v1.0.0")
set(WII_CONT_DIR "${CMAKE_SOURCE_DIR}/cpp/third_party/wii-controller-c-${WII_CONT_SUFFIX}")

ExternalProject_Add(
  wii-controller-c
  GIT_REPOSITORY "https://github.com/curmc/wii-controller-c"
  GIT_TAG "${WII_CONT_VERS}"

  UPDATE_COMMAND ""
  PATCH_COMMAND ""

  SOURCE_DIR "${WII_CONT_DIR}"
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/dependencies

  TEST_COMMAND ""
  )

if(BUILD_DEPENDENCIES)
ExternalProject_Add(
  yaml-cpp
  GIT_REPOSITORY "https://github.com/jbeder/yaml-cpp.git"
  GIT_TAG "yaml-cpp-0.6.2"

  UPDATE_COMMAND ""
  PATCH_COMMAND ""
  SOURCE_DIR "${CMAKE_SOURCE_DIR}/cpp/third_party/yaml-cpp-0.6.2"
  CMAKE_ARGS -DBUILD_SHARED_LIBS=ON

  TEST_COMMAND ""
)

set(PROTOBUF_TAR_GZ https://github.com/google/protobuf/archive/v3.4.0.tar.gz)
ExternalProject_Add(
  protobuf-external
  PREFIX protobuf
  URL ${PROTOBUF_TAR_GZ}
  BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/protobuf
  CMAKE_CACHE_ARGS
    "-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}"
    "-Dprotobuf_BUILD_TESTS:BOOL=OFF"
    "-Dprotobuf_BUILD_EXAMPLES:BOOL=OFF"
    "-Dprotobuf_WITH_ZLIB:BOOL=OFF"
    "-DCMAKE_CXX_COMPILER:STRING=${CMAKE_CXX_COMPILER}"
    # other project specific parameters
  SOURCE_SUBDIR cmake
  BUILD_ALWAYS 1
  STEP_TARGETS build
  INSTALL_COMMAND ""
)
endif()
set(wii-controller-c_INCLUDE_DIRS "${WII_CONT_DIR}/include")
include_directories(${wii-controller-c_INCLUDE_DIRS})
