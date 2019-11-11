pkg_search_module(MotionGenerators REQUIRED MotionGenerators)

find_package(PkgConfig REQUIRED)
# pkg_search_module(NLOPT REQUIRED nlopt)
# message(STATUS "NLOPT ${NLOPT_LIBRARIES} ${NLOPT_INCLUDE_DIRS}")
find_package(NLopt REQUIRED)

find_package(Pagmo REQUIRED)

# pkg_search_module(raylib REQUIRED raylib)
find_package(raylib REQUIRED)

# pkg_search_module(NLOPT REQUIRED nlopt)
message(STATUS "NLOPT ${NLOPT_LIBRARIES} ${NLOPT_INCLUDE_DIRS}")
# find_package(RapidJSON)
pkg_search_module(RapidJSON REQUIRED RapidJSON rapidjson)
pkg_search_module(Json11 REQUIRED Json11 json11)
message(STATUS "MotionGenerators: ${MotionGenerators_FOUND} ${MotionGenerators_INCLUDE_DIRS} ${MotionGenerators_LIBRARIES}")

generate_dependancies(NLOPT Pagmo MotionGenerators RapidJSON Json11 raylib)

message(STATUS "OPTIMIZER GENERATED LIST: ${DLIB_PATHS}")
message(STATUS "OPTIMIZER GENERATED LIST: ${DLIB_INCLUDES}")

add_executable(OptimizeTrajectory ${CMAKE_CURRENT_SOURCE_DIR}/src/IdentificationTrajectory/OptimizeTrajectory.cpp)
target_include_directories(OptimizeTrajectory PUBLIC ${PROJECT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/include ${DLIB_INCLUDES})
target_compile_features(OptimizeTrajectory PUBLIC cxx_std_11)
target_link_libraries(OptimizeTrajectory PUBLIC ${DLIB_PATHS} Pagmo::pagmo)

add_executable(MFTVisualizer ${CMAKE_CURRENT_SOURCE_DIR}/src/Visualization/MFTVisualizer.cpp)
target_include_directories(MFTVisualizer PUBLIC ${PROJECT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/include ${DLIB_INCLUDES})
target_compile_features(MFTVisualizer PUBLIC cxx_std_11)
target_link_libraries(MFTVisualizer PUBLIC ${DLIB_PATHS} Pagmo::pagmo)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS OptimizeTrajectory
     RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(TARGETS MFTVisualizer
     RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})