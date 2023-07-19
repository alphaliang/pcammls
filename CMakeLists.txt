cmake_minimum_required(VERSION 3.10.0)
project(pcammls)
include(ExternalProject)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)   
set(CAMPORT_DIR  "${CMAKE_SOURCE_DIR}/camport3" CACHE  PATH "Percipio SDK Path")
set(CAMPORT_ARCH  x64 CACHE STRING "Percipio SDK Arch")
set_property(CACHE CAMPORT_ARCH PROPERTY STRINGS x64 i686 Aarch64 armv7hf)
set(TARGET_LANGUAGE "PYTHON" CACHE STRING "target language")
set(PYTHON_VERSION "3.8" CACHE STRING "python version")
set(CSHARP_NET_VERSION  "v4.8" CACHE STRING ".NET version")

#set(SWIG_DIR  CACHE PATH "SWIG tool path")

message("target language is ${TARGET_LANGUAGE}")

macro(configure_msvc_runtime)
  if(MSVC)

    # Default to statically-linked runtime.
    if("${MSVC_RUNTIME}" STREQUAL "")
      set(MSVC_RUNTIME "static")
    endif()

    # Set compiler options.
    set(variables
      CMAKE_C_FLAGS_DEBUG
      CMAKE_C_FLAGS_MINSIZEREL
      CMAKE_C_FLAGS_RELEASE
      CMAKE_C_FLAGS_RELWITHDEBINFO
      CMAKE_CXX_FLAGS_DEBUG
      CMAKE_CXX_FLAGS_MINSIZEREL
      CMAKE_CXX_FLAGS_RELEASE
      CMAKE_CXX_FLAGS_RELWITHDEBINFO
    )
    if(${MSVC_RUNTIME} STREQUAL "static")
      message(STATUS
        "MSVC -> forcing use of statically-linked runtime."
      )
      foreach(variable ${variables})
        if(${variable} MATCHES "/MD")
          string(REGEX REPLACE "/MD" "/MT" ${variable} "${${variable}}")
        endif()
      endforeach()
    else()
      message(STATUS
        "MSVC -> forcing use of dynamically-linked runtime."
      )
      foreach(variable ${variables})
        if(${variable} MATCHES "/MT")
          string(REGEX REPLACE "/MT" "/MD" ${variable} "${${variable}}")
        endif()
      endforeach()
    endif()
  endif()
endmacro()

configure_msvc_runtime()
##################################################

#tycam
if ( (NOT EXISTS "${CAMPORT_DIR}") OR (NOT EXISTS "${CAMPORT_DIR}/include"))
    message(FATAL_ERROR "need set CAMPORT_DIR to sdk directory")
endif()

add_library(tycam SHARED IMPORTED GLOBAL)
target_include_directories(tycam INTERFACE ${CAMPORT_DIR}/include ./swig/)
if (MSVC)#for windows
    set(CAMPORT_LIB_ROOT "${CAMPORT_DIR}/lib/win/hostapp")
    if(CMAKE_CL_64) #x64
        set_property(TARGET tycam PROPERTY IMPORTED_LOCATION ${CAMPORT_LIB_ROOT}/x64/tycam.dll)
        set_property(TARGET tycam PROPERTY IMPORTED_IMPLIB  ${CAMPORT_LIB_ROOT}/x64/tycam.lib)
    else()
        set_property(TARGET tycam PROPERTY IMPORTED_LOCATION ${CAMPORT_LIB_ROOT}/x86/tycam.dll)
        set_property(TARGET tycam PROPERTY IMPORTED_IMPLIB ${CAMPORT_LIB_ROOT}/x86/tycam.lib)
    endif()
else(MSVC)
    if(CAMPORT_ARCH)
        if (NOT EXISTS "${CAMPORT_DIR}/lib/linux/lib_${CAMPORT_ARCH}/libtycam.so")
            message(FATAL_ERROR "not supported arch :${CAMPORT_ARCH} ")
        endif()
        set_property(TARGET tycam PROPERTY IMPORTED_LOCATION "${CAMPORT_DIR}/lib/linux/lib_${CAMPORT_ARCH}/libtycam.so")
    else()
        find_library(CAM_LIB_FILEPATH NAMES tycam) 
        if (NOT EXISTS ${CAM_LIB_FILEPATH})
            message(FATAL_ERROR "not found tycam library in system path")
        endif()
        set_target_properties( tycam PROPERTIES IMPORTED_LOCATION ${CAM_LIB_FILEPATH})
    endif()
endif(MSVC)


get_property(tylib_file TARGET tycam PROPERTY IMPORTED_LOCATION)
if (UNIX)
    install(FILES ${all_so_ext} DESTINATION "${TARGET_LANGUAGE}")
    get_filename_component(tylib_path ${tylib_file} DIRECTORY) 
    #copy all so file to install path
    file (GLOB all_so_ext ${tylib_path}/*.so )
    file (GLOB all_so_with_ver ${tylib_path}/*.so.* )
    install(FILES ${all_so_ext} DESTINATION "${TARGET_LANGUAGE}")
    install(FILES ${all_so_with_ver} DESTINATION "${TARGET_LANGUAGE}")
else(UNIX)
    install(FILES ${tylib_file} DESTINATION "${TARGET_LANGUAGE}")
endif(UNIX)

##################################################
#SWIG
CMAKE_POLICY(SET CMP0078 NEW)
CMAKE_POLICY(SET CMP0086 NEW)
if (WIN32)
    set(SWIG_EXECUTABLE ${SWIG_DIR}/swig.exe)
    set(SWIG_DIR ${SWIG_DIR})
endif()

set (USE_TARGET_INCLUDE_DIRECTORIES TRUE)
set (UseSWIG_TARGET_NAME_PREFERENCE STANDARD)
find_package(SWIG REQUIRED)
include(${SWIG_USE_FILE})
if (EXISTS "${SWIG_DIR}/Lib")
    include_directories(${SWIG_DIR}/lib )
endif()

set(FINAL_LOCATION_DIR ${CMAKE_CURRENT_BINARY_DIR}/Bin)

#build interface library
#create a dummy target which depneds on swig lib project
#because swig library traget can't be GLOBAL 
add_custom_target(pcam_build_stamp) 
add_subdirectory(swig)

if (${TARGET_LANGUAGE} STREQUAL "PYTHON")
    file(GLOB py_files ${CMAKE_CURRENT_SOURCE_DIR}/python/*.py) 
    install( FILES ${PCAM_LIB_SUPPORT_FILE_DIR}/pcammls.py DESTINATION  "${TARGET_LANGUAGE}")
    install( FILES ${py_files} DESTINATION  "${TARGET_LANGUAGE}")
endif()

if (${TARGET_LANGUAGE} STREQUAL "CSHARP")
    set(PROJECT_TEMP_DIR  Bin)
    ExternalProject_Add(csharp_libs
        SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/csharp"
        BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/csharp_build"
        DEPENDS 
            pcam_build_stamp
        CMAKE_ARGS 
        -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/install_cache
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
        -DCMAKE_STATIC_LINKER_FLAGS=${CMAKE_EXE_LINKER_FLAGS}
        -DCMAKE_STATIC_LINKER_FLAGS_DEBUG=${CMAKE_EXE_LINKER_FLAGS_DEBUG}
        -DCMAKE_STATIC_LINKER_FLAGS_RELEASE=${CMAKE_EXE_LINKER_FLAGS_RELEASE}
        -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}
        -DCMAKE_CXX_FLAGS_RELEASE=${CMAKE_CXX_FLAGS_RELEASE}
        -DCMAKE_CXX_FLAGS_DEBUG=${CMAKE_CXX_FLAGS_DEBUG}
        -DCMAKE_C_FLAGS=${CMAKE_C_FLAGS}
        -DCMAKE_C_FLAGS_DEBUG=${CMAKE_C_FLAGS_DEBUG}
        -DCMAKE_C_FLAGS_RELEASE=${CMAKE_C_FLAGS_RELEASE} 
        -DCMAKE_VS_PLATFORM_TOOLSET_HOST_ARCHITECTURE=${CMAKE_VS_PLATFORM_TOOLSET_HOST_ARCHITECTURE}
        -DPCAM_LIB_SUPPORT_FILE_DIR=${PCAM_LIB_SUPPORT_FILE_DIR}
        -DPROJECT_LIB_PATH=${PROJECT_TEMP_DIR}
        -DNET_VERSION=${CSHARP_NET_VERSION}
        INSTALL_COMMAND ${CMAKE_COMMAND} -E copy_directory ./${PROJECT_TEMP_DIR} ${FINAL_LOCATION_DIR}
        )
endif()


