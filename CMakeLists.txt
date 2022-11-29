cmake_minimum_required(VERSION 3.10)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(ISO22133 LANGUAGES C CXX)

# Dependencies
include(FetchContent)
execute_process(
	COMMAND ping www.github.com -c 1 -q
	ERROR_QUIET
	RESULT_VARIABLE NO_CONNECTION
)
FetchContent_Declare(googletest
	GIT_REPOSITORY https://github.com/google/googletest.git
	GIT_TAG main
)
FetchContent_MakeAvailable(googletest)


set(ISO22133_TARGET ${TARGET_NAMESPACE}${PROJECT_NAME})

include(GNUInstallDirs)

add_library(${ISO22133_TARGET} SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/positioning.c
        ${CMAKE_CURRENT_SOURCE_DIR}/iso22133.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/header.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/footer.c
)

target_include_directories(${ISO22133_TARGET} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/include
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(${ISO22133_TARGET} m)

set_property(TARGET ${ISO22133_TARGET} PROPERTY
	PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/iso22133.h
)
set_property(TARGET ${ISO22133_TARGET} APPEND PROPERTY
	PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/positioning.h
)

install(CODE "MESSAGE(STATUS \"Installing target ${ISO22133_TARGET}\")")
install(TARGETS ${ISO22133_TARGET} 
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)


# Tests
enable_testing()
file(GLOB TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp)
add_executable(${ISO22133_TARGET}_test
	${TEST_SOURCES}
)
target_link_libraries(${ISO22133_TARGET}_test
	gtest_main
	${ISO22133_TARGET}
)
target_include_directories(${ISO22133_TARGET}_test PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/include
)
include(GoogleTest)
gtest_discover_tests(${ISO22133_TARGET}_test)
