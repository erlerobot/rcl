find_package(ament_cmake_gtest REQUIRED)

set(extra_test_libraries)
set(extra_memory_tools_env PHONY=)  # Use a phony env var so there is always at least one.
# rcl_lib_dir is passed as APPEND_LIBRARY_DIRS for each ament_add_gtest call so
# the librcl that they link against is on the library path.
# This is especially improtant on Windows.
set(rcl_lib_dir $<TARGET_FILE_DIR:${PROJECT_NAME}>)
ament_find_gtest()  # For GTEST_LIBRARIES
if(APPLE)
  add_library(${PROJECT_NAME}_memory_tools_interpose SHARED memory_tools_osx_interpose.cpp)
  target_link_libraries(${PROJECT_NAME}_memory_tools_interpose ${GTEST_LIBRARIES})
  set_target_properties(${PROJECT_NAME}_memory_tools_interpose
    PROPERTIES COMPILE_FLAGS "-std=c++11")
  list(APPEND extra_test_libraries ${PROJECT_NAME}_memory_tools_interpose)
  list(APPEND extra_memory_tools_env
    DYLD_INSERT_LIBRARIES=$<TARGET_FILE:${PROJECT_NAME}_memory_tools_interpose>)
endif()
add_library(${PROJECT_NAME}_memory_tools SHARED memory_tools.cpp)
if(NOT WIN32)
  set_target_properties(${PROJECT_NAME}_memory_tools PROPERTIES COMPILE_FLAGS "-std=c++11")
endif()
if(UNIX AND NOT APPLE)
  list(APPEND extra_test_libraries dl)
  list(APPEND extra_memory_tools_env DL_PRELOAD=$<TARGET_FILE:${PROJECT_NAME}_memory_tools>)
endif()
target_link_libraries(${PROJECT_NAME}_memory_tools ${extra_test_libraries})
target_compile_definitions(${PROJECT_NAME}_memory_tools
  PRIVATE "RCL_MEMORY_TOOLS_BUILDING_DLL")
list(APPEND extra_test_libraries ${PROJECT_NAME}_memory_tools)

if(NOT WIN32)
  ament_add_gtest(test_memory_tools test_memory_tools.cpp
    ENV ${extra_memory_tools_env} APPEND_LIBRARY_DIRS ${rcl_lib_dir})
  if(TARGET test_memory_tools)
    target_include_directories(test_memory_tools PUBLIC
      ${rcl_interfaces_INCLUDE_DIRS}
      ${rmw_INCLUDE_DIRS}
    )
    if(NOT WIN32)
      set_target_properties(test_memory_tools PROPERTIES COMPILE_FLAGS "-std=c++11")
    endif()
    target_link_libraries(test_memory_tools ${PROJECT_NAME} ${extra_test_libraries})
  endif()
endif()

ament_add_gtest(test_allocator rcl/test_allocator.cpp
  ENV ${extra_memory_tools_env} APPEND_LIBRARY_DIRS ${rcl_lib_dir})
if(TARGET test_allocator)
  target_include_directories(test_allocator PUBLIC
    ${rcl_interfaces_INCLUDE_DIRS}
    ${rmw_INCLUDE_DIRS}
  )
  if(NOT WIN32)
    set_target_properties(test_allocator PROPERTIES COMPILE_FLAGS "-std=c++11")
  endif()
  target_link_libraries(test_allocator ${PROJECT_NAME} ${extra_test_libraries})
endif()

ament_add_gtest(test_time rcl/test_time.cpp
  ENV ${extra_memory_tools_env} APPEND_LIBRARY_DIRS ${rcl_lib_dir})
if(TARGET test_time)
  target_include_directories(test_time PUBLIC
    ${rcl_interfaces_INCLUDE_DIRS}
    ${rmw_INCLUDE_DIRS}
  )
  if(NOT WIN32)
    set_target_properties(test_time PROPERTIES COMPILE_FLAGS "-std=c++11")
  endif()
  target_link_libraries(test_time ${PROJECT_NAME} ${extra_test_libraries})
endif()

ament_add_gtest(test_common
  rcl/test_common.cpp
  ENV
    ${extra_memory_tools_env}
    EMPTY_TEST=
    NORMAL_TEST=foo
  APPEND_LIBRARY_DIRS
    ${rcl_lib_dir}
)
if(TARGET test_common)
  target_include_directories(test_common PUBLIC
    ${rcl_interfaces_INCLUDE_DIRS}
    ${rmw_INCLUDE_DIRS}
  )
  if(NOT WIN32)
    set_target_properties(test_common PROPERTIES COMPILE_FLAGS "-std=c++11")
  endif()
  target_link_libraries(test_common ${PROJECT_NAME} ${extra_test_libraries})
endif()

ament_add_gtest(test_rcl rcl/test_rcl.cpp
  ENV ${extra_memory_tools_env} APPEND_LIBRARY_DIRS ${rcl_lib_dir})
if(TARGET test_rcl)
  target_include_directories(test_rcl PUBLIC
    ${rcl_interfaces_INCLUDE_DIRS}
    ${rmw_INCLUDE_DIRS}
  )
  if(NOT WIN32)
    set_target_properties(test_rcl PROPERTIES COMPILE_FLAGS "-std=c++11")
  endif()
  target_link_libraries(test_rcl ${PROJECT_NAME} ${extra_test_libraries})
endif()

ament_add_gtest(test_node rcl/test_node.cpp
  ENV ${extra_memory_tools_env} APPEND_LIBRARY_DIRS ${rcl_lib_dir})
if(TARGET test_node)
  target_include_directories(test_node PUBLIC
    ${rcl_interfaces_INCLUDE_DIRS}
    ${rmw_INCLUDE_DIRS}
  )
  if(NOT WIN32)
    set_target_properties(test_node PROPERTIES COMPILE_FLAGS "-std=c++11")
  endif()
  target_link_libraries(test_node ${PROJECT_NAME} ${extra_test_libraries})
endif()