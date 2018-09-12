add_executable( run_camera_cap cameraCap.cpp )
target_link_libraries( run_camera_cap crcalib)

add_executable( run_camera_rtsp cameraNetCap.cpp )
target_link_libraries( run_camera_rtsp crcalib ${HC_LIBS} -lpthread)

add_executable( run_camera_calib cameraCalib.cpp )
target_link_libraries( run_camera_calib crcalib )

add_executable( run_linkage_calib_chessboard linkageCalib_chessboard.cpp )
target_link_libraries( run_linkage_calib_chessboard crcalib )

add_executable( run_linkage_calib linkageCalib.cpp )
target_link_libraries( run_linkage_calib crcalib )

add_executable( run_linkage_view linkage_view.cpp )
target_link_libraries( run_linkage_view crcalib )

add_executable( run_linkage_state_calib linkage_state_calib.cpp )
target_link_libraries( run_linkage_state_calib crcalib ${HC_LIBS})

add_executable( run_linkage_state linkage_test_state.cpp )
target_link_libraries( run_linkage_state crcalib ${HC_LIBS})

add_executable( run_linkage_ball linkage_test_ball.cpp )
target_link_libraries( run_linkage_ball crcalib ${HC_LIBS})
