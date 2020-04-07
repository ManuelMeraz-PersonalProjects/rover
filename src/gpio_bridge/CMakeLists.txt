cmake_minimum_required(VERSION 3.10)
project(GPIOBridg)

# set thirdparty_libraries
include(cmake/Setup.cmake)
add_subdirectory(src)

if(BUILD_TESTING)
    include(CTest)
    add_subdirectory(tests)
endif()

# Add all targets to the build-tree export set
export(TARGETS motor_controls FILE "${PROJECT_BINARY_DIR}/GPIOBridgeTargets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE GPIOBridge)

# Create the GPIOBridgeConfig.cmake and GPIOBridgeConfigVersion.cmake files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}" "${INSTALL_INCLUDE_DIR}")
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(cmake/GPIOBridgeConfig.cmake.in "${PROJECT_BINARY_DIR}/GPIOBridgeConfig.cmake" @ONLY)
# ... for the install tree
set(CONF_INCLUDE_DIRS "\${GPIOBRIDGE_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(cmake/GPIOBridgeConfig.cmake.in "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/GPIOBridgeConfig.cmake" @ONLY)
# ... for both
configure_file(cmake/GPIOBridgeConfigVersion.cmake.in "${PROJECT_BINARY_DIR}/GPIOBridgeConfigVersion.cmake" @ONLY)

# Install the GPIOBridgeConfig.cmake and GPIOBridgeConfigVersion.cmake
install(FILES
        "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/GPIOBridgeConfig.cmake"
        "${PROJECT_BINARY_DIR}/GPIOBridgeConfigVersion.cmake"
        DESTINATION
        "${INSTALL_CMAKE_DIR}" COMPONENT dev)

install(DIRECTORY include/ DESTINATION ${INSTALL_INCLUDE_DIR})

# Install the export set for use with the install-tree
install(EXPORT GPIOBridgeTargets
        DESTINATION "${INSTALL_CMAKE_DIR}"
        COMPONENT dev)
