cmake_minimum_required(VERSION 3.10.0)
project(pcammls_cs)

enable_language(CSharp)
set(CMAKE_CSharp_FLAGS "/langversion:default")

if(CMAKE_CL_64)
    set(CMAKE_CSharp_FLAGS "/platform:x64")
else()
    set(CMAKE_CSharp_FLAGS "/platform:x86")
endif()

file(GLOB CS_SUPPORT_FILES ${PCAM_LIB_SUPPORT_FILE_DIR}/*.cs)

add_library(pcammls_cs SHARED ${CS_SUPPORT_FILES})

set (REF_LIST 
    "Microsoft.CSharp"
    "System"
    "System.Core"
    "System.Data"
    "System.Deployment"
    "System.Drawing"
    "System.Xml"
)


set(LIBRARY_OUTPUT_PATH  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_LIB_PATH})
set(EXECUTABLE_OUTPUT_PATH   ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_LIB_PATH})

message("target net version is ${NET_VERSION}")

set_property(TARGET pcammls_cs PROPERTY VS_DOTNET_REFERENCES ${REF_LIST})
set_property(TARGET pcammls_cs PROPERTY VS_DOTNET_TARGET_FRAMEWORK_VERSION ${NET_VERSION})

#add_executable(test_cs ${CMAKE_CURRENT_SOURCE_DIR}/test.cs )
#set_property(TARGET test_cs PROPERTY VS_DOTNET_TARGET_FRAMEWORK_VERSION ${NET_VERSION})
#set_property(TARGET test_cs PROPERTY VS_DOTNET_REFERENCES ${REF_LIST} )

if(${MSVC_VERSION} LESS 1915)	#VS2017
	option(OpenCV_USE "Using opencv" OFF)
else()
    option(OpenCV_USE "Using opencv" ON)
endif()

if(OpenCV_USE)
	add_executable(fetch_frame_cs ${CMAKE_CURRENT_SOURCE_DIR}/fetch_frame_opencv.cs ${CMAKE_CURRENT_SOURCE_DIR}/TY_ISP.cs)
	set_property(TARGET fetch_frame_cs PROPERTY VS_PACKAGE_REFERENCES "OpenCvSharp4_4.6.0.20220608;OpenCvSharp4.runtime.win_4.6.0.20220608")
else()
	add_executable(fetch_frame_cs ${CMAKE_CURRENT_SOURCE_DIR}/fetch_frame.cs ${CMAKE_CURRENT_SOURCE_DIR}/TY_ISP.cs)
endif()
set_property(TARGET fetch_frame_cs PROPERTY VS_DOTNET_TARGET_FRAMEWORK_VERSION ${NET_VERSION})
set_property(TARGET fetch_frame_cs PROPERTY VS_DOTNET_REFERENCES ${REF_LIST} )

add_executable(fetch_trigger_cs ${CMAKE_CURRENT_SOURCE_DIR}/fetch_trigger.cs ${CMAKE_CURRENT_SOURCE_DIR}/TY_ISP.cs)
set_property(TARGET fetch_trigger_cs PROPERTY VS_DOTNET_TARGET_FRAMEWORK_VERSION ${NET_VERSION})
set_property(TARGET fetch_trigger_cs PROPERTY VS_DOTNET_REFERENCES ${REF_LIST} )

add_executable(fetch_point3d_cs ${CMAKE_CURRENT_SOURCE_DIR}/fetch_point3d.cs)
set_property(TARGET fetch_point3d_cs PROPERTY VS_DOTNET_TARGET_FRAMEWORK_VERSION ${NET_VERSION})
set_property(TARGET fetch_point3d_cs PROPERTY VS_DOTNET_REFERENCES ${REF_LIST} )

add_executable(fetch_IR_cs ${CMAKE_CURRENT_SOURCE_DIR}/fetch_IR.cs)
set_property(TARGET fetch_IR_cs PROPERTY VS_DOTNET_TARGET_FRAMEWORK_VERSION ${NET_VERSION})
set_property(TARGET fetch_IR_cs PROPERTY VS_DOTNET_REFERENCES ${REF_LIST} )

#add_executable(fetch_color3d_cs ${CMAKE_CURRENT_SOURCE_DIR}/fetch_color3d.cs ${CMAKE_CURRENT_SOURCE_DIR}/TY_ISP.cs)
#set_property(TARGET fetch_color3d_cs PROPERTY VS_DOTNET_TARGET_FRAMEWORK_VERSION ${NET_VERSION})
#set_property(TARGET fetch_color3d_cs PROPERTY VS_DOTNET_REFERENCES ${REF_LIST} )




target_compile_options(fetch_frame_cs PUBLIC "/unsafe")
target_compile_options(fetch_trigger_cs PUBLIC "/unsafe")
target_compile_options(fetch_point3d_cs PUBLIC "/unsafe")
target_compile_options(fetch_IR_cs PUBLIC "/unsafe")

add_dependencies(fetch_point3d_cs pcammls_cs)
add_dependencies(fetch_trigger_cs pcammls_cs)
#add_dependencies(fetch_color3d_cs pcammls_cs)
add_dependencies(fetch_frame_cs pcammls_cs)
add_dependencies(fetch_IR_cs pcammls_cs)
#add_dependencies(test_cs pcammls_cs)

#target_link_libraries(test_cs pcammls_cs)
target_link_libraries(fetch_frame_cs pcammls_cs)
target_link_libraries(fetch_trigger_cs pcammls_cs)
#target_link_libraries(fetch_color3d_cs pcammls_cs)
target_link_libraries(fetch_point3d_cs pcammls_cs)
target_link_libraries(fetch_IR_cs pcammls_cs)
