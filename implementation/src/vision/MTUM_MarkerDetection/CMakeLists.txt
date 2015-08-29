if(NOT CMAKE_CL_64)
    set(FILTER_NAME mtum_MarkerDetection)

    #set include directories
    include_directories(${ARUCO_INCLUDE_DIR})
    include_directories(${OpenCV_INCLUDE_DIR}) 

    # add the source
    adtf_add_filter(${FILTER_NAME}
                    stdafx.cpp  
                    stdafx.h
                    cMarkerDetectFilter.h
                    cMarkerDetectFilter.cpp
            )

    # linking libs
    target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS} ${ARUCO_LIBS})

    # set folder (only needed and available for MSVC)
    adtf_set_folder(${FILTER_NAME} aadcuser/imageproc) 

    # Specify where it should be installed to
    adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
	
endif(NOT CMAKE_CL_64)