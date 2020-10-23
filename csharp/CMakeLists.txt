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

# .NET reference libraries.
set (REF_LIST 
    "Microsoft.CSharp"
    "System"
    "System.Core"
    "System.Data"
    "System.Deployment"
    "System.Drawing"
    "System.Xml"
)
set_property(TARGET pcammls_cs PROPERTY VS_DOTNET_REFERENCES ${REF_LIST})
set_property(TARGET pcammls_cs PROPERTY VS_DOTNET_TARGET_FRAMEWORK_VERSION "v4.0")

add_executable(test_cs ${CMAKE_CURRENT_SOURCE_DIR}/test.cs )
set_property(TARGET test_cs PROPERTY VS_DOTNET_TARGET_FRAMEWORK_VERSION "v4.0")
set_property(TARGET test_cs PROPERTY VS_DOTNET_REFERENCES ${REF_LIST} )

add_executable(fetch_frame_cs ${CMAKE_CURRENT_SOURCE_DIR}/fetch_frame.cs ${CMAKE_CURRENT_SOURCE_DIR}/TY_ISP.cs)
set_property(TARGET fetch_frame_cs PROPERTY VS_DOTNET_TARGET_FRAMEWORK_VERSION "v4.0")
set_property(TARGET fetch_frame_cs PROPERTY VS_DOTNET_REFERENCES ${REF_LIST} )

add_executable(fetch_registration_cs ${CMAKE_CURRENT_SOURCE_DIR}/fetch_registration.cs ${CMAKE_CURRENT_SOURCE_DIR}/TY_ISP.cs)
set_property(TARGET fetch_registration_cs PROPERTY VS_DOTNET_TARGET_FRAMEWORK_VERSION "v4.0")
set_property(TARGET fetch_registration_cs PROPERTY VS_DOTNET_REFERENCES ${REF_LIST} )

add_executable(fetch_point3d_cs ${CMAKE_CURRENT_SOURCE_DIR}/fetch_point3d.cs)
set_property(TARGET fetch_point3d_cs PROPERTY VS_DOTNET_TARGET_FRAMEWORK_VERSION "v4.0")
set_property(TARGET fetch_point3d_cs PROPERTY VS_DOTNET_REFERENCES ${REF_LIST} )

add_dependencies(fetch_point3d_cs pcammls_cs)
add_dependencies(fetch_registration_cs pcammls_cs)
add_dependencies(fetch_frame_cs pcammls_cs)
add_dependencies(test_cs pcammls_cs)
target_link_libraries(test_cs pcammls_cs)
target_link_libraries(fetch_frame_cs pcammls_cs)
target_link_libraries(fetch_registration_cs pcammls_cs)
target_link_libraries(fetch_point3d_cs pcammls_cs)

install (TARGETS pcammls_cs DESTINATION  "./")
install (TARGETS test_cs DESTINATION  "./")
install (TARGETS fetch_frame_cs DESTINATION  "./")
install (TARGETS fetch_registration_cs DESTINATION  "./")
install (TARGETS fetch_point3d_cs DESTINATION  "./")
install (FILES ${CMAKE_CURRENT_SOURCE_DIR}/fetch_point3d.cs  DESTINATION  "./")
install (FILES ${CMAKE_CURRENT_SOURCE_DIR}/fetch_registration.cs ${CMAKE_CURRENT_SOURCE_DIR}/TY_ISP.cs  DESTINATION  "./")
install (FILES ${CMAKE_CURRENT_SOURCE_DIR}/fetch_frame.cs ${CMAKE_CURRENT_SOURCE_DIR}/TY_ISP.cs  DESTINATION  "./")
install (FILES ${CMAKE_CURRENT_SOURCE_DIR}/test.cs  DESTINATION  "./")

