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
	"System.Windows.Forms"
)


set(LIBRARY_OUTPUT_PATH  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_LIB_PATH})
set(EXECUTABLE_OUTPUT_PATH   ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_LIB_PATH})

message("target net version is ${NET_VERSION}")

set_property(TARGET pcammls_cs PROPERTY VS_DOTNET_REFERENCES ${REF_LIST})
set_property(TARGET pcammls_cs PROPERTY VS_DOTNET_TARGET_FRAMEWORK_VERSION ${NET_VERSION})

add_executable(fetch_frame_cs 
    ${CMAKE_CURRENT_SOURCE_DIR}/fetch_frame.cs 
	${CMAKE_CURRENT_SOURCE_DIR}/Program.cs 
	${CMAKE_CURRENT_SOURCE_DIR}/Form1.Designer.cs
	${CMAKE_CURRENT_SOURCE_DIR}/Form1.resx)
set_property(TARGET fetch_frame_cs PROPERTY VS_DOTNET_TARGET_FRAMEWORK_VERSION ${NET_VERSION})
set_property(TARGET fetch_frame_cs PROPERTY VS_DOTNET_REFERENCES ${REF_LIST} )


add_executable(fetch_ISP
    ${CMAKE_CURRENT_SOURCE_DIR}/fetch_isp.cs 
	${CMAKE_CURRENT_SOURCE_DIR}/Program.cs 
	${CMAKE_CURRENT_SOURCE_DIR}/Form1.Designer.cs
	${CMAKE_CURRENT_SOURCE_DIR}/Form1.resx)
set_property(TARGET fetch_ISP PROPERTY VS_DOTNET_TARGET_FRAMEWORK_VERSION ${NET_VERSION})
set_property(TARGET fetch_ISP PROPERTY VS_DOTNET_REFERENCES ${REF_LIST} )

add_executable(fetch_IR_cs 
    ${CMAKE_CURRENT_SOURCE_DIR}/fetch_IR.cs 
	${CMAKE_CURRENT_SOURCE_DIR}/Program.cs 
	${CMAKE_CURRENT_SOURCE_DIR}/Form1.Designer.cs
	${CMAKE_CURRENT_SOURCE_DIR}/Form1.resx)
set_property(TARGET fetch_IR_cs PROPERTY VS_DOTNET_TARGET_FRAMEWORK_VERSION ${NET_VERSION})
set_property(TARGET fetch_IR_cs PROPERTY VS_DOTNET_REFERENCES ${REF_LIST} )

add_executable(fetch_registration_cs 
    ${CMAKE_CURRENT_SOURCE_DIR}/fetch_registration.cs 
	${CMAKE_CURRENT_SOURCE_DIR}/Program.cs 
	${CMAKE_CURRENT_SOURCE_DIR}/Form1.Designer.cs
	${CMAKE_CURRENT_SOURCE_DIR}/Form1.resx)
set_property(TARGET fetch_registration_cs PROPERTY VS_DOTNET_TARGET_FRAMEWORK_VERSION ${NET_VERSION})
set_property(TARGET fetch_registration_cs PROPERTY VS_DOTNET_REFERENCES ${REF_LIST} )

add_executable(fetch_trigger_cs 
    ${CMAKE_CURRENT_SOURCE_DIR}/fetch_trigger.cs 
	${CMAKE_CURRENT_SOURCE_DIR}/Program.cs 
	${CMAKE_CURRENT_SOURCE_DIR}/Form1.Designer.cs
	${CMAKE_CURRENT_SOURCE_DIR}/Form1.resx)
set_property(TARGET fetch_trigger_cs PROPERTY VS_DOTNET_TARGET_FRAMEWORK_VERSION ${NET_VERSION})
set_property(TARGET fetch_trigger_cs PROPERTY VS_DOTNET_REFERENCES ${REF_LIST} )

add_executable(fetch_point3d_cs 
    ${CMAKE_CURRENT_SOURCE_DIR}/fetch_point3d.cs)
set_property(TARGET fetch_point3d_cs PROPERTY VS_DOTNET_TARGET_FRAMEWORK_VERSION ${NET_VERSION})
set_property(TARGET fetch_point3d_cs PROPERTY VS_DOTNET_REFERENCES ${REF_LIST} )

target_link_libraries(fetch_frame_cs pcammls_cs)
target_link_libraries(fetch_ISP pcammls_cs)
target_link_libraries(fetch_IR_cs pcammls_cs)
target_link_libraries(fetch_registration_cs pcammls_cs)
target_link_libraries(fetch_trigger_cs pcammls_cs)
target_link_libraries(fetch_point3d_cs pcammls_cs)