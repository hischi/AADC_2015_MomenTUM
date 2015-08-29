set(FILTER_NAME mtum_ObstacleGrid)
 

# include directories
include_directories(${OPENNI2_INCLUDE_DIR} ${OpenCV_INCLUDE_DIR}) 

# Specify the filter module target
adtf_add_filter(${FILTER_NAME}
                 stdafx.cpp
                 stdafx.h
                 cCubic.h
                 Sensor.h
                 Sensor.cpp
                 SensorIR_Short.h
                 SensorIR_Short.cpp
                 SensorIR_Combined.h
                 SensorIR_Combined.cpp
                 ObstacleGrid.h
                 ObstacleGrid.cpp
)

#target_link_libraries(${FILTER_NAME} shared_fusion_types)
target_link_libraries(${FILTER_NAME} ${OPENNI2_LIBS} ${OpenCV_LIBS})

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} aadcuser/implementation/sensor_fusion) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
