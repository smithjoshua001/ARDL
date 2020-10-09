pkg_search_module(MotionGenerators REQUIRED MotionGenerators)

find_package(PkgConfig REQUIRED)
# pkg_search_module(NLOPT REQUIRED nlopt)
# message(STATUS "NLOPT ${NLOPT_LIBRARIES} ${NLOPT_INCLUDE_DIRS}")
find_package(NLopt REQUIRED)

find_package(Pagmo REQUIRED)

#pkg_search_module(raylib REQUIRED raylib)
find_package(raylib REQUIRED)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

# pkg_search_module(NLOPT REQUIRED nlopt)
message(STATUS "NLOPT ${NLOPT_LIBRARIES} ${NLOPT_INCLUDE_DIRS}")
# find_package(RapidJSON)
pkg_search_module(RapidJSON REQUIRED RapidJSON rapidjson)
pkg_search_module(Json11 REQUIRED Json11 json11)
message(STATUS "MotionGenerators: ${MotionGenerators_FOUND} ${MotionGenerators_INCLUDE_DIRS} ${MotionGenerators_LIBRARIES}")

generate_dependancies(NLOPT Pagmo MotionGenerators RapidJSON Json11 raylib OPENGL GLEW)
# generate_dependancies(MotionGenerators RapidJSON Json11)
message(STATUS "OPTIMIZER GENERATED LIST: ${DLIB_PATHS}")
message(STATUS "OPTIMIZER GENERATED LIST: ${DLIB_INCLUDES}")

add_executable(OptimizeTrajectory ${CMAKE_CURRENT_SOURCE_DIR}/src/IdentificationTrajectory/OptimizeTrajectory.cpp)
target_include_directories(OptimizeTrajectory PUBLIC ${PROJECT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/include ${DLIB_INCLUDES})
target_compile_features(OptimizeTrajectory PUBLIC cxx_std_17)

target_compile_definitions(OptimizeTrajectory PUBLIC ${DEFS})
target_link_libraries(OptimizeTrajectory PUBLIC ${DLIB_PATHS} Pagmo::pagmo stdc++fs)

add_executable(MFTVisualizer ${CMAKE_CURRENT_SOURCE_DIR}/src/Visualization/MFTVisualizer.cpp)
target_include_directories(MFTVisualizer PUBLIC ${PROJECT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/include ${DLIB_INCLUDES})
target_compile_features(MFTVisualizer PUBLIC cxx_std_17)
target_compile_definitions(MFTVisualizer PUBLIC ${DEFS})
target_link_libraries(MFTVisualizer PUBLIC ${DLIB_PATHS} X11 Xxf86vm Xrandr Xi stdc++fs ${CMAKE_DL_LIBS})

# add_executable(VerifyTrajectories ${CMAKE_CURRENT_SOURCE_DIR}/src/TrajectoryVerification/VerifyTrajectories.cpp)
# target_include_directories(VerifyTrajectories PUBLIC ${PROJECT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/include ${DLIB_INCLUDES})
# target_compile_features(VerifyTrajectories PUBLIC cxx_std_17)
# target_link_libraries(VerifyTrajectories PUBLIC ${DLIB_PATHS})

add_executable(VerifyTrajectoriesMulti ${CMAKE_CURRENT_SOURCE_DIR}/src/TrajectoryVerification/VerifyTrajectoriesMulti.cpp)
target_include_directories(VerifyTrajectoriesMulti PUBLIC ${PROJECT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/include ${DLIB_INCLUDES})
target_compile_features(VerifyTrajectoriesMulti PUBLIC cxx_std_17)
target_compile_definitions(VerifyTrajectoriesMulti PUBLIC ${DEFS})
target_link_libraries(VerifyTrajectoriesMulti PUBLIC ${DLIB_PATHS} stdc++fs)

add_executable(WorkspaceSubsets ${CMAKE_CURRENT_SOURCE_DIR}/src/IdentificationTrajectory/WorkspaceSubsets.cpp)
target_include_directories(WorkspaceSubsets PUBLIC ${PROJECT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/include ${DLIB_INCLUDES})
target_compile_features(WorkspaceSubsets PUBLIC cxx_std_17)
target_compile_definitions(WorkspaceSubsets PUBLIC ${DEFS})
target_link_libraries(WorkspaceSubsets PUBLIC ${DLIB_PATHS} stdc++fs)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY scripts/ DESTINATION ${CMAKE_INSTALL_BINDIR})
install(TARGETS OptimizeTrajectory
     RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(TARGETS MFTVisualizer
     RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
# install(TARGETS VerifyTrajectories
#      RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(TARGETS VerifyTrajectoriesMulti
     RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(TARGETS WorkspaceSubsets
     RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
