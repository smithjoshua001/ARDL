
find_package(orocos_kdl REQUIRED)
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


generate_dependancies(orocos_kdl KDL_Parser RBDL RBDL_URDFReader)

list(APPEND DLIB_PATHS ${TinyXML})
list(APPEND DLIB_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR})

message(STATUS "GENERATED LIST: ${DLIB_PATHS}")
message(STATUS "GENERATED LIST: ${DLIB_INCLUDES}")
add_subdirectory(Kinematics)
add_subdirectory(Dynamics)
add_subdirectory(Model)