
find_package(orocos_kdl REQUIRED)
find_package(manif REQUIRED)
pkg_search_module(KDL_Parser REQUIRED KDL_Parser)
find_library(TinyXML NAMES tinyxml)
pkg_search_module(RBDL rbdl)
if (${RBDL_FOUND})
pkg_search_module(RBDL_URDFReader REQUIRED rbdl_urdfreader)
# include_directories(${RBDL_INCLUDE_DIRS})
# include_directories(${RBDL_URDFReader_INCLUDE_DIRS})
# find_library(RBDL_LIBRARY ${RBDL_LIBRARIES} HINTS ${RBDL_LIBRARY_DIRS})
# find_library(RBDL_URDFReader_LIBRARY ${RBDL_URDFReader_LIBRARIES} HINTS ${RBDL_URDFReader_LIBRARY_DIRS})
endif()

string(REPLACE ":" ";" newstring "$ENV{CMAKE_MODULE_PATH}")
set(CMAKE_MODULE_PATH "${newstring}")

pkg_search_module(pinocchio REQUIRED pinocchio)

generate_dependancies(pinocchio RBDL RBDL_URDFReader orocos_kdl KDL_Parser manif)



list(APPEND DLIB_PATHS ${TinyXML} "stdc++fs")
list(APPEND DLIB_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(Kinematics)
add_subdirectory(Dynamics)
add_subdirectory(Model)
add_subdirectory(Math)

add_subdirectory(Collision)
