set(FILTER_NAME aadc_crossing_detection)
message(STATUS "OpenCV_INCLUDE_DIR: ${OpenCV_INCLUDE_DIR}")
# include directories
include_directories(${OpenCV_INCLUDE_DIR}) 

# Specify the filter module target
adtf_add_filter(${FILTER_NAME}
                stdafx.cpp
                stdafx.h
                crossingDetection.h
                crossingDetection.cpp
)

# linking libs
target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} aadcuser/implementation/vision) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
