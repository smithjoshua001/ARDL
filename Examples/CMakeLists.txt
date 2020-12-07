project(
  ARDL-Examples
  LANGUAGES CXX
  VERSION 0.1)
find_package(PkgConfig REQUIRED)

# install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

list(APPEND DLIB_PATHS "stdc++fs")
list(APPEND DLIB_PATHS ${CMAKE_THREAD_LIBS_INIT} ARDL::ARDL)

set(CMAKE_CXX_STANDARD 17)

add_executable(${PROJECT_NAME}_DynamicsExample
               ${CMAKE_CURRENT_SOURCE_DIR}/src/DynamicsExample.cpp)
target_link_libraries(${PROJECT_NAME}_DynamicsExample PUBLIC ${DLIB_PATHS})
target_include_directories(
  ${PROJECT_NAME}_DynamicsExample
  PUBLIC $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}> ${DLIB_INCLUDES})
target_compile_features(${PROJECT_NAME}_DynamicsExample PUBLIC cxx_std_17)
install(TARGETS ${PROJECT_NAME}_DynamicsExample
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
