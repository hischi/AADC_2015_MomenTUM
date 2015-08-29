include_directories(./include)
include_directories(../../adtfBase/AADC_ADTF_BaseFilters/include)
include_directories(./src/sensor_fusion/shared_types)

add_subdirectory(src/utile)
add_subdirectory(src/vision)
add_subdirectory(src/sensor_fusion)
add_subdirectory(src/decision_making)
