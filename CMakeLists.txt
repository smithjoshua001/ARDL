cmake_minimum_required(VERSION 3.9)

#
# This creates a standard cmake project. You may extend this file with any cmake
# macro you see fit.
#
project(
  ARDL
  LANGUAGES CXX
  VERSION 0.1)

set(${CMAKE_PROJECT_NAME}_VERSION_MAJOR 0)
set(${CMAKE_PROJECT_NAME}_VERSION_MINOR 1)
set(${CMAKE_PROJECT_NAME}_VERSION_PATCH 0)
set(${CMAKE_PROJECT_NAME}_VERSION
    ${${CMAKE_PROJECT_NAME}_VERSION_MAJOR}.${${CMAKE_PROJECT_NAME}_VERSION_MINOR}.${${CMAKE_PROJECT_NAME}_VERSION_PATCH}
)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_FLAGS_RELEASE
    "${CMAKE_CXX_FLAGS} -DLOG_DEBUG_ON=6 -std=c++17 -g -O3 -march=native")
set(CMAKE_CXX_FLAGS_DEBUG
    "${CMAKE_CXX_FLAGS} -DLOG_DEBUG_ON=6 -std=c++17 -g -O3 -Wall")

string(REPLACE ":" ";" newstring "$ENV{CMAKE_MODULE_PATH}")
set(CMAKE_MODULE_PATH "${newstring}")

include(dependency)
include(compile_commands)

option(ARDL_VARIANT "Enable Variants for static polymorphism" TRUE)

option(ARDL_SHARED_PTR "Enable shared ptr for data handling" TRUE)

option(ARDL_PYTHON "Enable experimental python (through cppyy" FALSE)

option(ARDL_EXAMPLES "Enable ARDL Examples" FALSE)

find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(urdfdom_headers REQUIRED IMPORTED_TARGET
                  urdfdom_headers>=1.0.0)
pkg_search_module(urdfdom REQUIRED IMPORTED_TARGET urdfdom>=1.0.0)
find_package(assimp REQUIRED)

pkg_search_module(spdlog REQUIRED IMPORTED_TARGET spdlog)

pkg_search_module(CCD REQUIRED IMPORTED_TARGET ccd)

find_package(fcl REQUIRED)

find_package(Threads)

pkg_search_module(RapidJSON REQUIRED IMPORTED_TARGET RapidJSON)
pkg_search_module(tinyxml2 REQUIRED IMPORTED_TARGET tinyxml2)

generate_dependancies(fmt::fmt ASSIMP Threads)

list(
  APPEND
  DLIB_PATHS
  ${CMAKE_THREAD_LIBS_INIT}
  Eigen3::Eigen
  PkgConfig::urdfdom
  PkgConfig::spdlog
  PkgConfig::CCD
  fcl
  PkgConfig::RapidJSON
  PkgConfig::tinyxml2)

file(
  GLOB_RECURSE ${CMAKE_PROJECT_NAME}_HEADERS
  RELATIVE "${PROJECT_SOURCE_DIR}/include"
  "${PROJECT_SOURCE_DIR}/include/*.hpp")

add_library(${CMAKE_PROJECT_NAME} INTERFACE)

add_library(${CMAKE_PROJECT_NAME}::${CMAKE_PROJECT_NAME} ALIAS
            ${CMAKE_PROJECT_NAME})

target_include_directories(
  ${CMAKE_PROJECT_NAME}
  INTERFACE $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)
target_compile_options(${PROJECT_NAME} INTERFACE -Wall -Wextra -Wno-pedantic)

target_compile_definitions(
  ${PROJECT_NAME} INTERFACE ARDL_VARIANT=$<BOOL:${ARDL_VARIANT}>
                            ARDL_SHARED_PTR=$<BOOL:${ARDL_SHARED_PTR}>)

target_link_libraries(${CMAKE_PROJECT_NAME} INTERFACE ${DLIB_PATHS})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if(COMMAND add_dummy)
  check_header_update(${${CMAKE_PROJECT_NAME}_HEADERS})
endif()

list(APPEND DEFS ARDL_VARIANT=$<BOOL:${ARDL_VARIANT}>
     ARDL_SHARED_PTR=$<BOOL:${ARDL_SHARED_PTR}>)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${CMAKE_PROJECT_NAME}.pc.cmake
               ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.pc @ONLY)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.pc
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)

install(TARGETS ${CMAKE_PROJECT_NAME} EXPORT ${CMAKE_PROJECT_NAME}-targets)

set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${CMAKE_PROJECT_NAME})

export(
  EXPORT ${CMAKE_PROJECT_NAME}-targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Targets.cmake
  NAMESPACE ${CMAKE_PROJECT_NAME}::)
# Export the targets to a script
install(
  EXPORT ${CMAKE_PROJECT_NAME}-targets
  NAMESPACE ${CMAKE_PROJECT_NAME}::
  DESTINATION ${INSTALL_CONFIGDIR})

# Create a ConfigVersion.cmake file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion)

configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/${CMAKE_PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${INSTALL_CONFIGDIR})

# Install the config, configversion and custom find modules
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${INSTALL_CONFIGDIR})

# Register package in user's package registry
export(PACKAGE ${CMAKE_PROJECT_NAME})


if(COMMAND add_dummy)

  add_dummy(DLIB_INCLUDES DLIB_PATHS DEFS)

  generate_compile_commands()
endif()

if(ARDL_PYTHON)
  execute_process(
    COMMAND cling-config --cmake
    OUTPUT_VARIABLE CPPYY_MODULE_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  message("CPYY_MODULE_PATH: " ${CPPYY_MODULE_PATH})
  list(INSERT CMAKE_MODULE_PATH 0 ${CPPYY_MODULE_PATH})
  find_package(Cppyy REQUIRED)
  message(STATUS "VERSION: ${Cppyy_VERSION}")
  cppyy_add_bindings(
    "ARDL"
    "${PROJECT_VERSION}"
    "Joshua Smith"
    "Joshua.Smith@ed.ac.uk"
    LANGUAGE_STANDARD
    "17"
    GENERATE_OPTIONS
    "-D__PIC__;-Wno-macro-redefined;-std=c++17;-O3;-march=native;-DARDL_VARIANT=${ARDL_VARIANT};-DARDL_SHARED_PTR=${ARDL_SHARED_PTR};-DARDL_EXTERNAL_DATA=$<BOOL:${ARDL_EXTERNAL_DATA}>"
    COMPILE_OPTIONS
    "-std=c++17;-O3;-march=native;-DARDL_VARIANT=${ARDL_VARIANT};-DARDL_SHARED_PTR=${ARDL_SHARED_PTR};-DARDL_EXTERNAL_DATA=$<BOOL:${ARDL_EXTERNAL_DATA}>"
    INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include;$ENV{PhD_IS}/ARDL_ws/install/include/eigen3;$ENV{PhD_IS}/ARDL_ws/install/include"
    LINK_LIBRARIES
    "fmt::fmt;${DLIB_PATHS}"
    H_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    H_FILES
    "${${CMAKE_PROJECT_NAME}_HEADERS}")
  install(CODE "execute_process(COMMAND pip3 install ${PY_WHEEL_FILE})")
endif()

unset(DLIBS)
unset(DLIB_PATHS)
unset(DLIB_INCLUDES)

add_subdirectory(Examples)
