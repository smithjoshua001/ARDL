list(APPEND DLIB_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(${PROJECT_NAME}_KinematicsTestsUnaligned ${CMAKE_CURRENT_SOURCE_DIR}/KinematicsUnaligned.cpp)
target_include_directories(${PROJECT_NAME}_KinematicsTestsUnaligned PRIVATE ${${PROJECT_NAME}_SOURCE_DIR}/include ${DLIB_INCLUDES})
target_compile_features(${PROJECT_NAME}_KinematicsTestsUnaligned PRIVATE cxx_std_11)
target_link_libraries(${PROJECT_NAME}_KinematicsTestsUnaligned PRIVATE ${DLIB_PATHS})

add_executable(${PROJECT_NAME}_KinematicsTests ${CMAKE_CURRENT_SOURCE_DIR}/KinematicsAligned.cpp)
target_include_directories(${PROJECT_NAME}_KinematicsTests PRIVATE ${${PROJECT_NAME}_SOURCE_DIR}/include ${DLIB_INCLUDES})
target_compile_features(${PROJECT_NAME}_KinematicsTests PRIVATE cxx_std_11)
target_link_libraries(${PROJECT_NAME}_KinematicsTests PRIVATE ${DLIB_PATHS})