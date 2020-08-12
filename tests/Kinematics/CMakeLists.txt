list(APPEND DLIB_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR})

# add_executable(${PROJECT_NAME}_KinematicsTestsUnaligned ${CMAKE_CURRENT_SOURCE_DIR}/KinematicsUnaligned.cpp)
# target_include_directories(${PROJECT_NAME}_KinematicsTestsUnaligned PRIVATE ${${PROJECT_NAME}_SOURCE_DIR}/include ${DLIB_INCLUDES})
# target_compile_features(${PROJECT_NAME}_KinematicsTestsUnaligned PRIVATE cxx_std_11)
# target_link_libraries(${PROJECT_NAME}_KinematicsTestsUnaligned PRIVATE ${DLIB_PATHS})

add_executable(${PROJECT_NAME}_KinematicsTests ${CMAKE_CURRENT_SOURCE_DIR}/KinematicsAligned.cpp)
target_include_directories(${PROJECT_NAME}_KinematicsTests PRIVATE ${${PROJECT_NAME}_SOURCE_DIR}/include ${DLIB_INCLUDES})
target_compile_features(${PROJECT_NAME}_KinematicsTests PRIVATE cxx_std_17)
if(${ARDL_VARIANT})
target_compile_definitions(${PROJECT_NAME}_KinematicsTests PRIVATE ARDL_VARIANT=${ARDL_VARIANT})
endif(${ARDL_VARIANT})
target_link_libraries(${PROJECT_NAME}_KinematicsTests PRIVATE ${DLIB_PATHS} orocos-kdl)
target_compile_options(${PROJECT_NAME}_KinematicsTests PUBLIC ${pinocchio_CFLAGS_OTHER})
install(TARGETS ${PROJECT_NAME}_KinematicsTests
     RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
