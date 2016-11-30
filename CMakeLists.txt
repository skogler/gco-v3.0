project(gco)
cmake_minimum_required(VERSION 2.8)

set(PROJECT_VERSION_MAJOR 3)
set(PROJECT_VERSION_MINOR 0)
set(PROJECT_VERSIOM_PATCH 0)
set(PROJECT_NUMBER "v${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSIOM_PATCH}")

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

set(lib_srcs
  "${PROJECT_SOURCE_DIR}/src/GCoptimization.cpp"
  "${PROJECT_SOURCE_DIR}/src/GCoptimization.h"
  "${PROJECT_SOURCE_DIR}/src/LinkedBlockList.cpp"
  "${PROJECT_SOURCE_DIR}/src/LinkedBlockList.h"
  "${PROJECT_SOURCE_DIR}/src/block.h"
  "${PROJECT_SOURCE_DIR}/src/energy.h"
  "${PROJECT_SOURCE_DIR}/src/graph.h"
)

set(example_srcs
  "${PROJECT_SOURCE_DIR}/src/example.cpp"
)

add_library("${PROJECT_NAME}" ${lib_srcs})
add_executable("${PROJECT_NAME}-example" ${example_srcs})
target_link_libraries("${PROJECT_NAME}-example" "${PROJECT_NAME}")

install(TARGETS "${PROJECT_NAME}"
        ARCHIVE DESTINATION "lib"
        LIBRARY DESTINATION "lib"
        COMPONENT library)

install(DIRECTORY "${PROJECT_SOURCE_DIR}/src/"
        DESTINATION "include/${PROJECT_NAME}/"
        FILES_MATCHING PATTERN "*.h" PATTERN "*.inl"
        PATTERN "matlab" EXCLUDE)
