cmake_minimum_required(VERSION 3.0 VERSION 3.0.0)
project(gco)

set(CXX_STANDARD 11)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

set(LIB_SRC
  "${PROJECT_SOURCE_DIR}/src/GCoptimization.cpp"
  "${PROJECT_SOURCE_DIR}/src/GCoptimization.h"
  "${PROJECT_SOURCE_DIR}/src/LinkedBlockList.cpp"
  "${PROJECT_SOURCE_DIR}/src/LinkedBlockList.h"
  "${PROJECT_SOURCE_DIR}/src/block.h"
  "${PROJECT_SOURCE_DIR}/src/energy.h"
  "${PROJECT_SOURCE_DIR}/src/graph.h"
)

add_library("${PROJECT_NAME}" ${LIB_SRC})

set(APPS
  "example"
)

include_directories(
  "${PROJECT_SOURCE_DIR}/src"
  ${FFORMATION_INCLUDE_DIRS}
)

foreach(EXAMPLE ${APPS})
  add_executable("${PROJECT_NAME}-${EXAMPLE}" "${PROJECT_SOURCE_DIR}/app/${EXAMPLE}.cpp")
  target_link_libraries("${PROJECT_NAME}-${EXAMPLE}"
    ${PROJECT_NAME}
    )
endforeach(EXAMPLE)

install(TARGETS "${PROJECT_NAME}"
        ARCHIVE DESTINATION "lib"
        LIBRARY DESTINATION "lib"
        COMPONENT library)

install(DIRECTORY "${PROJECT_SOURCE_DIR}/src/"
        DESTINATION "include/${PROJECT_NAME}/"
        FILES_MATCHING PATTERN "*.h" PATTERN "*.inl"
        PATTERN "matlab" EXCLUDE)
