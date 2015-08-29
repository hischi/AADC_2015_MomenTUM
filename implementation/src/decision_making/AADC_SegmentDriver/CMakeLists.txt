set(FILTER_NAME aadc_SegmentDriver)
 
#set (GRID_LINK_DIR ${CMAKE_INSTALL_PREFIX}/${AADC_INSTALL_DIR_LIBS})
#link_directories(${GRID_LINK_DIR})
#include_directories(${AADC_GRID_INCLUDE_DIR})

# include directories
include_directories(${OPENNI2_INCLUDE_DIR} ${OpenCV_INCLUDE_DIR})

# Specify the filter module target
adtf_add_filter(${FILTER_NAME}
                 stdafx.cpp
                 stdafx.h
                 SegmentDriver.h
                 SegmentDriver.cpp
)

#target_link_libraries(${FILTER_NAME} shared_fusion_types)
target_link_libraries(${FILTER_NAME} ${OPENNI2_LIBS} ${OpenCV_LIBS})

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} aadcuser/implementation/decision_making) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
