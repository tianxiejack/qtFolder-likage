cmake_minimum_required( VERSION 2.8 )
project ( linkage )

set( CMAKE_CXX_COMPILER "g++" )
#set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

set(OpenCV_DIR /home/alex/soft/opencv-3.2.0/build)
############### dependencies ######################
# Eigen
include_directories( "/usr/include/eigen3" )
# OpenCV
find_package( OpenCV 3.1 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
# Sophus 
#find_package( Sophus REQUIRED )
#include_directories( ${Sophus_INCLUDE_DIRS} )
# G2O
#find_package( G2O REQUIRED )
#include_directories( ${G2O_INCLUDE_DIRS} )

set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
 #   ${Sophus_LIBRARIES}
 #   g2o_core g2o_stuff g2o_types_sba
)


set( HC_LIBS
    /home/alex/QTwork/HCNetSDK/lib/libhcnetsdk.so
    /home/alex/QTwork/HCNetSDK/lib/libhpr.so
    /home/alex/QTwork/HCNetSDK/lib/libHCCore.so
    /home/alex/QTwork/HCNetSDK/lib/HCNetSDKCom/libHCCoreDevCfg.so
    /home/alex/QTwork/HCNetSDK/lib/HCNetSDKCom/libStreamTransClient.so
    /home/alex/QTwork/HCNetSDK/lib/HCNetSDKCom/libSystemTransform.so
    /home/alex/QTwork/HCNetSDK/lib/HCNetSDKCom/libHCPreview.so
    /home/alex/QTwork/HCNetSDK/lib/HCNetSDKCom/libHCAlarm.so
    /home/alex/QTwork/HCNetSDK/lib/HCNetSDKCom/libHCGeneralCfgMgr.so
    /home/alex/QTwork/HCNetSDK/lib/HCNetSDKCom/libHCIndustry.so
    /home/alex/QTwork/HCNetSDK/lib/HCNetSDKCom/libHCPlayBack.so
    /home/alex/QTwork/HCNetSDK/lib/HCNetSDKCom/libHCVoiceTalk.so
    /home/alex/QTwork/HCNetSDK/lib/HCNetSDKCom/libanalyzedata.so
    /home/alex/QTwork/HCNetSDK/lib/HCNetSDKCom/libHCDisplay.so
)

############### dependencies ######################

add_definitions( -D__linux__)

include_directories( ${PROJECT_SOURCE_DIR}/include )
add_subdirectory( src )
add_subdirectory( test )
