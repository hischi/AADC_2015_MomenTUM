set(FILTER_NAME mtum_crossing_detection_ml)

# include directories
include_directories(${OpenCV_INCLUDE_DIR}) 

# Specify the filter module target
adtf_add_filter(${FILTER_NAME}
        stdafx.cpp
        stdafx.h
        Feature.h
        Feature.cpp
        crossingDetection_ML.h
        crossingDetection_ML.cpp
        StrongClassifier.h
        StrongClassifier.cpp
        WeakClassifier.h
        WeakClassifier.cpp
        CrossingModel.cpp
        CrossingModel.h
        LaneDetector.cpp
        LaneDetector.h
)

# linking libs
target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} aadcuser/implementation/vision) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
