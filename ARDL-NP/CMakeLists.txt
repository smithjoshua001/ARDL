
project(ARDL-NP LANGUAGES CXX VERSION 0.1)
#generate_dependancies()
# add_library(${PROJECT_NAME} STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/IGMMModel.cpp)
# target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/include ${DLIB_INCLUDES})
# target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
# target_link_libraries(${PROJECT_NAME} PUBLIC ${DLIB_PATHS})

add_library(${PROJECT_NAME} INTERFACE)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} INTERFACE $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include> $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)
target_compile_options(${PROJECT_NAME} INTERFACE -Wall -Wextra -Wpedantic)
target_compile_definitions(${PROJECT_NAME} INTERFACE ARDL_VARIANT=${ARDL_VARIANT} ARDL_SHARED_PTR=${ARDL_SHARED_PTR})
target_link_libraries(${PROJECT_NAME} INTERFACE ARDL::ARDL)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(TARGETS ${PROJECT_NAME}
EXPORT ${PROJECT_NAME}-targets)

set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

export(EXPORT ${PROJECT_NAME}-targets
FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake
NAMESPACE ${PROJECT_NAME}::)
#Export the targets to a script
install(EXPORT ${PROJECT_NAME}-targets
NAMESPACE
  ${PROJECT_NAME}::
DESTINATION
  ${INSTALL_CONFIGDIR}
)

#Create a ConfigVersion.cmake file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

#Install the config, configversion and custom find modules
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${INSTALL_CONFIGDIR}
)


#Register package in user's package registry
export(PACKAGE ${PROJECT_NAME})

# add_subdirectory(tests)

# CONFIGURE_FILE (
#   ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.pc.cmake
#   ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc @ONLY
#   )

# INSTALL (
#   FILES  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc
#   DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
#   )
