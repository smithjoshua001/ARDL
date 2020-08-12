list(APPEND DLIB_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(${PROJECT_NAME}_MathTests ${CMAKE_CURRENT_SOURCE_DIR}/MathAligned.cpp)
target_include_directories(${PROJECT_NAME}_MathTests PRIVATE ${${PROJECT_NAME}_SOURCE_DIR}/include ${DLIB_INCLUDES})
target_compile_features(${PROJECT_NAME}_MathTests PRIVATE cxx_std_11)
target_link_libraries(${PROJECT_NAME}_MathTests PRIVATE ${DLIB_PATHS})

target_compile_definitions(${PROJECT_NAME}_MathTests PRIVATE ARDL_VARIANT=${ARDL_VARIANT} ARDL_SHARED_PTR=${ARDL_SHARED_PTR} ARDL_EXTERNAL_DATA=$<BOOL:${ARDL_EXTERNAL_DATA}>)

target_compile_options(${PROJECT_NAME}_MathTests PUBLIC ${pinocchio_CFLAGS_OTHER})
install(TARGETS ${PROJECT_NAME}_MathTests
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
