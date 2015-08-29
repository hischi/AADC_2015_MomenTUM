set(FILTER_NAME aadc_lane_detection)

# include directories
include_directories(${OpenCV_INCLUDE_DIR}) 

# Specify the filter module target
adtf_add_filter(${FILTER_NAME}
        stdafx.cpp
        stdafx.h
        ContourModel.h
        ContourModel.cpp
        Feature.h
        Feature.cpp
        IPMapper.h
        IPMapper.cpp
        laneDetection.h
        laneDetection.cpp
        LaneDetector.h
        LaneDetector.cpp
        LaneModel.h
        LaneModel.cpp
        PointSet.cpp
        PointSet.h
        StrongClassifier.h
        StrongClassifier.cpp
        WeakClassifier.h
        WeakClassifier.cpp
)

# linking libs
target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} aadcuser/implementation/vision) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
