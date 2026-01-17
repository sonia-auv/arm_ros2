add_executable(${PROJECT_NAME}
	${CMAKE_SOURCE_DIR}/src/cli.cpp
	${CMAKE_SOURCE_DIR}/src/config.cpp
	${CMAKE_SOURCE_DIR}/src/main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE yaml-cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include)
