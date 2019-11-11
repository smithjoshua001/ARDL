cmake_minimum_required(VERSION 2.8.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(ARDL LANGUAGES CXX)

SET ( ${CMAKE_PROJECT_NAME}_VERSION_MAJOR 0 )
SET ( ${CMAKE_PROJECT_NAME}_VERSION_MINOR 1 )
SET ( ${CMAKE_PROJECT_NAME}_VERSION_PATCH 0 )
SET ( ${CMAKE_PROJECT_NAME}_VERSION
	${${CMAKE_PROJECT_NAME}_VERSION_MAJOR}.${${CMAKE_PROJECT_NAME}_VERSION_MINOR}.${${CMAKE_PROJECT_NAME}_VERSION_PATCH}
)


INCLUDE(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -std=c++14 -Ofast -march=native")

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -march=native -Wall -g")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -DLOG_DEBUG_ON=6 -std=c++14 -g -Og -march=native -Wall ")

find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(urdfdom_headers REQUIRED urdfdom_headers>=1.0.0)
pkg_search_module(urdfdom REQUIRED urdfdom>=1.0.0)
find_package(assimp REQUIRED)

pkg_search_module(CCD REQUIRED ccd)

pkg_search_module(FCL REQUIRED fcl)

find_package(Threads)

pkg_search_module(RapidJSON REQUIRED RapidJSON)


function(generate_dependancies)
    list(APPEND DLIBS ${ARGV})
    foreach(lib IN LISTS DLIBS)
        set(TLIB ${${lib}_LIBRARIES})
        set(TLIB_DIR ${${lib}_LIBRARY_DIRS})
        message(STATUS "TLIB: ${TLIB}")
        foreach(sublib IN LISTS TLIB)
            message(STATUS "SUB LIBS: ${sublib}")
            message(STATUS "SUB LIBS: ${${lib}_LIBRARY_DIRS}")
            if(DEFINED ${lib}_LIBRARIES)
                find_library(subDLIB_PATH NAMES ${sublib} PATHS ${${lib}_LIBRARY_DIRS} NO_DEFAULT_PATH)
                if(subDLIB_PATH)
                    list(APPEND DLIB_PATHS ${subDLIB_PATH})
                else()
                    list(APPEND DLIB_PATHS ${sublib})
                endif()
                unset(subDLIB_PATH CACHE)
            endif()
        endforeach(sublib)
        list(APPEND DLIB_INCLUDES ${${lib}_INCLUDE_DIRS})
        message(STATUS "INCLUDES: ${${lib}_INCLUDE_DIRS}")
    endforeach(lib)
    set(DLIB_PATHS "${DLIB_PATHS}" PARENT_SCOPE)
    set(DLIB_INCLUDES "${DLIB_INCLUDES}" PARENT_SCOPE)
endfunction()

# list(APPEND DEPENDENCY_LIBRARIES EIGEN3 SPDLOG FMT urdfdom_headers urdfdom ASSIMP CCD FCL Threads RBDL RBDL_URDFReader orocos_kdl KDL_Parser json11)

generate_dependancies(EIGEN3 SPDLOG FMT urdfdom_headers urdfdom ASSIMP CCD FCL Threads json11)
list(APPEND DLIB_PATHS ${CMAKE_THREAD_LIBS_INIT})
message(STATUS "GENERATED LIST: ${DLIB_PATHS}")
message(STATUS "GENERATED LIST: ${DLIB_INCLUDES}")


file(GLOB_RECURSE ${CMAKE_PROJECT_NAME}_HEADERS RELATIVE "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/include/*.hpp"
)

add_library(${CMAKE_PROJECT_NAME} INTERFACE)
target_sources(${CMAKE_PROJECT_NAME} INTERFACE "${${CMAKE_PROJECT_NAME}_HEADERS}")
target_include_directories(${CMAKE_PROJECT_NAME} INTERFACE $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include> $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_11)

target_link_libraries(${CMAKE_PROJECT_NAME} INTERFACE ${DLIB_PATHS})



function(check_header_update)
    foreach(HDR IN LISTS ARGV)
        file(TIMESTAMP "${PROJECT_SOURCE_DIR}/include/${HDR}" TMPHDRSTAMP)
        list(APPEND TSTAMPS ${TMPHDRSTAMP})
    endforeach()
    string(MD5 HDRHASHNEW "${ARGV}${TSTAMPS}")
    message(STATUS "HASH1 ${HDRHASHNEW}")
    if(NOT DEFINED HDRHASH)
        set(HDRHASH "tmp" CACHE internal "")
        message(STATUS "HASH ${HDRHASH}")
    endif()
    message(STATUS "HASH ${HDRHASH}")
    string(COMPARE EQUAL "${HDRHASH}" "${HDRHASHNEW}" HASHEQUAL)
    if(NOT ${HASHEQUAL})
        file(WRITE ${PROJECT_SOURCE_DIR}/src/dummy.cpp "/*DUMMY FILE FOR INCLUDES!*/")#\n ${EXTRA_DUMMY_DEFINES}
        foreach(HDR IN LISTS ARGV)
        file(APPEND ${PROJECT_SOURCE_DIR}/src/dummy.cpp "\n#include \"${HDR}\"")
        endforeach(HDR)
        file(APPEND ${PROJECT_SOURCE_DIR}/src/dummy.cpp "\nint main(int argc, char* argv[]){ return 0; }")
        set(HDRHASH "${HDRHASHNEW}" CACHE internal "" FORCE)
    endif()
    
endfunction()
check_header_update(${${CMAKE_PROJECT_NAME}_HEADERS})

add_executable(DUMMY ${PROJECT_SOURCE_DIR}/src/dummy.cpp)
target_include_directories(DUMMY PUBLIC ${${PROJECT_NAME}_SOURCE_DIR}/include ${DLIB_INCLUDES})
target_compile_features(DUMMY PUBLIC cxx_std_11)
target_link_libraries(DUMMY PUBLIC ${DLIB_PATHS})

add_subdirectory(tests)
add_subdirectory(ARDL-Trajectory)

message(STATUS "HERE I AM!! ${PROJECT_BINARY_DIR}")
add_custom_command(OUTPUT "${PROJECT_BINARY_DIR}/compile_commands_full.json" 
COMMAND /usr/local/bin/compdb -p ${PROJECT_BINARY_DIR}/ list -1 > "${PROJECT_BINARY_DIR}/compile_commands_full.json"
# WORKING_DIRECTORY "${PROJECT_BINARY_DIR}"
#DEPENDS "${PROJECT_BINARY_DIR}/compile_commands.json"
COMMENT "compile commands"
VERBATIM
)
# add_custom_target(vsphere_header ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/vsphere.h)
# add_dependencies(DUMMY )
message(STATUS "HERE I AM2!! ${PROJECT_BINARY_DIR}")
add_custom_target(compile_commands ALL DEPENDS "${PROJECT_BINARY_DIR}/compile_commands_full.json" VERBATIM)
add_dependencies(DUMMY compile_commands)
if(EXISTS ${PROJECT_BINARY_DIR}/compile_commands_full.json)
file(RENAME ${PROJECT_BINARY_DIR}/compile_commands_full.json ${PROJECT_SOURCE_DIR}/compile_commands.json)
endif()

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})