macro(mock_find_package_for_older_cmake PACKAGE_NAME)
  # Only if CMake < 3.24 (no OVERRIDE_FIND_PACKAGE)
  if(NOT CMAKE_VERSION VERSION_GREATER_EQUAL "3.24")
    set(MOCK_CONFIG_DIR "${CMAKE_BINARY_DIR}/cmake-mock-configs")
    if(NOT EXISTS "${MOCK_CONFIG_DIR}")
      file(MAKE_DIRECTORY "${MOCK_CONFIG_DIR}")
    endif()

    list(APPEND CMAKE_PREFIX_PATH "${MOCK_CONFIG_DIR}")
    list(REMOVE_DUPLICATES CMAKE_PREFIX_PATH)
    set(CMAKE_PREFIX_PATH
        "${CMAKE_PREFIX_PATH}"
        CACHE STRING "Mock config path" FORCE)

    set(MOCK_CONFIG_FILE "${MOCK_CONFIG_DIR}/${PACKAGE_NAME}Config.cmake")

    if(NOT EXISTS "${MOCK_CONFIG_FILE}")
      file(WRITE "${MOCK_CONFIG_FILE}" "set(${PACKAGE_NAME}_FOUND TRUE)\n")
    endif()
  endif()
endmacro()
