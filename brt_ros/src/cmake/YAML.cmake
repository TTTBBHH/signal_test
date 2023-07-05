find_package(PkgConfig REQUIRED)

include_directories(/usr/local/include/yaml-cpp)#头文件路径
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
include_directories(${YAML_CPP_INCLUDEDIR})
message(STATUS "Find YAML include at ${YAML_CPPDIRS}")
message(STATUS "Find YAML_INCLUDE include at ${YAML_CPP_INCLUDEDIR}")
list(APPEND ALL_TARGET_LIBRARIES ${YAML_CPP_LIBRARIES})