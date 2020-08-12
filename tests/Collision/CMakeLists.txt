
find_package(PkgConfig REQUIRED)
pkg_search_module(raylib REQUIRED raylib)
#find_package(raylib REQUIRED)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

generate_dependancies(OPENGL GLEW raylib)


list(APPEND DLIB_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(-DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR)
add_executable(${PROJECT_NAME}_CollisionTests ${CMAKE_CURRENT_SOURCE_DIR}/CollisionAligned.cpp)
target_include_directories(${PROJECT_NAME}_CollisionTests PRIVATE ${${PROJECT_NAME}_SOURCE_DIR}/include ${DLIB_INCLUDES})
target_compile_features(${PROJECT_NAME}_CollisionTests PRIVATE cxx_std_17)
target_compile_definitions(${PROJECT_NAME}_CollisionTests PRIVATE ARDL_VARIANT=${ARDL_VARIANT} ARDL_SHARED_PTR=${ARDL_SHARED_PTR} ARDL_EXTERNAL_DATA=$<BOOL:${ARDL_EXTERNAL_DATA}>)
message(STATUS "DYANMICS+========================= $<BOOL:${ARDL_EXTERNAL_DATA}>")
target_link_libraries(${PROJECT_NAME}_CollisionTests  PRIVATE ${DLIB_PATHS} X11 Xxf86vm Xrandr Xi orocos-kdl)
target_compile_options(${PROJECT_NAME}_CollisionTests PRIVATE ${pinocchio_CFLAGS_OTHER})
install(TARGETS ${PROJECT_NAME}_CollisionTests
     RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
