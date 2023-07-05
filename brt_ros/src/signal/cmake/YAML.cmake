find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
include_directories(/usr/local/include/yaml-cpp)
list(APPEND ALL_TARGET_LIBRARIES ${YAML_CPP_LIBRARIES} /usr/local/lib/libyaml-cpp.so)
