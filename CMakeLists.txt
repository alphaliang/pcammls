cmake_minimum_required(VERSION 3.1.0)
project(pcammls)

set(CAMPORT_DIR "" CACHE  PATH "Percipio SDK Path")
set(CAMPORT_ARCH "" CACHE STRING "Percipio SDK Arch")
set(TARGET_LANGUAGE "PYTHON" CACHE STRING "target language")
set(SWIG_DIR "" CACHE PATH "SWIG tool path")

# OpenCV
find_package(OpenCV REQUIRED)
if (NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV library not found")
else()
    include_directories(${OpenCV_INCLUDE_DIRS})
    include_directories(${OpenCV2_INCLUDE_DIRS})
    link_directories(${OpenCV_LIB_DIR})
endif()
option (OpenCV_STATIC OFF)

#python
if (${TARGET_LANGUAGE} STREQUAL "PYTHON")
    set(PYTHON_DIR "" CACHE PATH "python dir")
    message(${TARGET_LANGUAGE})
    find_package(PythonLibs REQUIRED)
    include_directories(${PYTHON_INCLUDE_PATH})
endif()


#tycam
if (EXISTS "${CAMPORT_DIR}")
    add_library(tycam SHARED IMPORTED GLOBAL)
    target_include_directories(tycam INTERFACE ${CAMPORT_DIR}/include)
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
            set_property(TARGET tycam PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../lib/linux/lib_${CAMPORT_ARCH}/libtycam.so)
        else()
            #set(CAMPORT_LIB -ltycam)#let compiler to find lib
            target_compile_options(tycam PUBLIC -ltycam)
        endif()
    endif(MSVC)
endif()

#SWIG
if (WIN32)
    set(SWIG_EXECUTABLE ${SWIG_DIR}/swig.exe)
    set(SWIG_DIR ${SWIG_DIR})
else()
    set(SWIG_EXECUTABLE "")
endif()


set (USE_TARGET_INCLUDE_DIRECTORIES TRUE)
set (UseSWIG_TARGET_NAME_PREFERENCE STANDARD)
find_package(SWIG REQUIRED)
include(${SWIG_USE_FILE})


set_source_files_properties(pcammls.i PROPERTIES CPLUSPLUS ON)
#set_source_files_properties(pcammls.i PROPERTIES SWIG_FLAGS "-includeall")
set_source_files_properties(pcammls.i PROPERTIES INCLUDE_DIRECTORIES "${CAMPORT_DIR}/include")    
get_source_file_property (aaa pcammls.i INCLUDE_DIRECTORIES)
if (EXISTS "${SWIG_DIR}/Lib")
    include_directories(${SWIG_DIR}/Lib)
    include_directories(${SWIG_DIR}/Lib ${SWIG_DIR}/Lib/python)
endif()

swig_add_library(pcammls 
		LANGUAGE python  
		SOURCES pcammls.i ${sources} 
		OUTFILE_DIR ${CMAKE_CURRENT_BINARY_DIR}/swig/_outfile
		OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/swig/_output
)

set_target_properties(pcammls PROPERTIES PREFIX "_")
get_property(support_files TARGET pcammls PROPERTY SWIG_SUPPORT_FILES)
target_link_libraries(pcammls tycam   ${PYTHON_LIBRARIES} ${OpenCV_LIBS})


if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=gnu++0x)
    add_definitions(-fpermissive)
endif()

