add_library( crcalib SHARED
    camera_calibration.cpp
    linkage_calibration.cpp
)

target_link_libraries( crcalib
    ${THIRD_PARTY_LIBS}
)
