project(gco)
cmake_minimum_required(VERSION 2.8)

set(PROJECT_VERSION_MAJOR 3)
set(PROJECT_VERSION_MINOR 0)
set(PROJECT_VERSIOM_PATCH 0)
set(PROJECT_NUMBER "v${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSIOM_PATCH}")

# add c++11 compiler flag
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -DBOOST_SIGNALS_NO_DEPRECATION_WARNING")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(Boost 1.54 COMPONENTS program_options REQUIRED)
find_package(fformation REQUIRED)

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
  "fformation"
)

include_directories(
  "${PROJECT_SOURCE_DIR}/src"
  ${FFORMATION_INCLUDE_DIRS}
)

link_directories(${Boost_LIBRARY_DIRS})

foreach(EXAMPLE ${APPS})
  add_executable("${PROJECT_NAME}-${EXAMPLE}" "${PROJECT_SOURCE_DIR}/app/${EXAMPLE}.cpp")
  target_link_libraries("${PROJECT_NAME}-${EXAMPLE}"
    ${PROJECT_NAME}
    ${FFORMATION_LIBRARIES}
    ${Boost_LIBRARIES}
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
