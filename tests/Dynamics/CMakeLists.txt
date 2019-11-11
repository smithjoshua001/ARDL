list(APPEND DLIB_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(${PROJECT_NAME}_DynamicsTestsUnaligned ${CMAKE_CURRENT_SOURCE_DIR}/DynamicsUA.cpp)
target_include_directories(${PROJECT_NAME}_DynamicsTestsUnaligned PRIVATE ${${PROJECT_NAME}_SOURCE_DIR}/include ${DLIB_INCLUDES})
target_compile_features(${PROJECT_NAME}_DynamicsTestsUnaligned PRIVATE cxx_std_11)
target_link_libraries(${PROJECT_NAME}_DynamicsTestsUnaligned PRIVATE ${DLIB_PATHS})
