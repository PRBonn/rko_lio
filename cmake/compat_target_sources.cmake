# compat_target_sources macro: backward-compatible target_sources with FILE_SET
# for CMake < 3.23. Adds private sources normally, adds include directories
# fallback using BASE_DIRS
macro(compat_target_sources target)
  list(REMOVE_AT ARGV 0)
  set(args ${ARGV})

  if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.23")
    target_sources(${target} ${args})
  else()
    set(private_sources)
    set(base_dir "")
    set(in_private OFF)
    set(in_public OFF)
    foreach(arg IN LISTS args)
      if(arg STREQUAL "PRIVATE")
        set(in_private ON)
        set(in_public OFF)
        continue()
      elseif(arg STREQUAL "PUBLIC")
        set(in_private OFF)
        set(in_public ON)
        continue()
      elseif(arg STREQUAL "BASE_DIRS")
        set(in_private OFF)
        set(in_public OFF)
        continue()
      endif()

      if(in_private)
        list(APPEND private_sources ${arg})
      elseif(
        NOT in_private
        AND NOT in_public
        AND NOT base_dir)
        set(base_dir ${arg})
      endif()
    endforeach()

    target_sources(${target} PRIVATE ${private_sources})
    target_include_directories(${target} PUBLIC "${base_dir}")
  endif()
endmacro()
