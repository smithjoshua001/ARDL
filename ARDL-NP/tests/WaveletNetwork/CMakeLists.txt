list(APPEND DLIB_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR})
add_executable(${PROJECT_NAME}_WaveletTests ${CMAKE_CURRENT_SOURCE_DIR}/WaveletTests.cpp)
target_include_directories(${PROJECT_NAME}_WaveletTests PRIVATE ${${PROJECT_NAME}_SOURCE_DIR}/include ${DLIB_INCLUDES})
target_compile_features(${PROJECT_NAME}_WaveletTests PRIVATE cxx_std_17)

target_link_libraries(${PROJECT_NAME}_WaveletTests  PRIVATE ${DLIB_PATHS})
install(TARGETS ${PROJECT_NAME}_WaveletTests
     RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
