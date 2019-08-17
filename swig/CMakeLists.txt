##################################################
#
#output target language file path: ${PCAM_LIB_SUPPORT_FILE_DIR}
#
##################################################
cmake_minimum_required(VERSION 3.10.0)

if(NOT EXISTS "${CAMPORT_DIR}/include/TYApi.h")
    message(FATAL_ERROR "not found TYApi.h")
endif()


set(PCAM_OUTFILE_DIR ${CMAKE_CURRENT_BINARY_DIR}/swig_outfile)
set(PCAM_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/swig_output)

set(PCAM_LIBNAME pcammls)
if (${TARGET_LANGUAGE} STREQUAL "PYTHON") #python
    include_directories(BEFORE ${SWIG_DIR}/Lib/python )
    set(PYTHON_DIR "" CACHE PATH "python dir")
	option(WITH_NUMPY ON)
	if(NOT ${PYTHON_VERSION}  STREQUAL "")
		find_package(PythonLibs ${PYTHON_VERSION} EXACT REQUIRED)
	else()
		find_package(PythonLibs ${PYTHON_VERSION} REQUIRED)
	endif()
    include_directories(${PYTHON_INCLUDE_PATH})
    if (SWIG_VERSION VERSION_LESS "4.0.0")
        set(swig_src_flag "")
    else()
        set(swig_src_flag "-doxygen")
    endif()
	if (WITH_NUMPY)
		find_package(NumPy REQUIRED)
        set(swig_src_flag "${swig_src_flag};-DWITH_NUMPY=1")
		list(APPEND SWIG_INTERFACE_INCLUDE_DIR "${PYTHON_NUMPY_INCLUDE_DIR}" )
	endif()
	set_source_files_properties(pcammls.i PROPERTIES SWIG_FLAGS  "${swig_src_flag}")
endif()

if (${TARGET_LANGUAGE} STREQUAL "CSHARP") # csharp
    include_directories( BEFORE ${SWIG_DIR}/Lib/csharp )
	set_property(SOURCE pcammls.i PROPERTY SWIG_MODULE_NAME pcammls)
	#message( "${CMAKE_SWIG_FLAGS}")
	#set_source_files_properties(pcammls.i PROPERTIES SWIG_FLAGS  )
endif()

if (${TARGET_LANGUAGE} STREQUAL "JAVA") # java
    #set(swig_src_flag "")
    include_directories(BEFORE ${SWIG_DIR}/Lib/java )
	find_package(JNI REQUIRED)
    #set(swig_src_flag "${swig_src_flag};-package com.percipio.pcammls")
	#:wset (CMAKE_SWIG_FLAGS "${CMAKE_SWIG_FLAGS};-package com.percipio.pcammls")
	set(PCAM_OUTPUT_DIR "${PCAM_OUTPUT_DIR}/com/percipio/pcammls")
	#set_source_files_properties(pcammls.i PROPERTIES SWIG_FLAGS  "${swig_src_flag}")
endif()

#source files#################################################
set_source_files_properties(pcammls.i PROPERTIES CPLUSPLUS ON)
#set_source_files_properties(pcammls.i PROPERTIES SWIG_FLAGS "-includeall")

list(APPEND SWIG_INTERFACE_INCLUDE_DIR "${CAMPORT_DIR}/include" )
set_source_files_properties(pcammls.i PROPERTIES INCLUDE_DIRECTORIES "${SWIG_INTERFACE_INCLUDE_DIR}")    
file(GLOB c_headers ${CAMPORT_DIR}/include/*.h)
#aed_library#################################################

swig_add_library(${PCAM_LIBNAME}
        LANGUAGE ${TARGET_LANGUAGE}
        SOURCES pcammls.i ${c_headers}
        OUTFILE_DIR ${PCAM_OUTFILE_DIR}
        OUTPUT_DIR ${PCAM_OUTPUT_DIR}
)

file(GLOB swig_dep_files ${CMAKE_CURRENT_SOURCE_DIR}/*.i)
set (SWIG_CLEAN_STAMP_FILE  ${CMAKE_CURRENT_BINARY_DIR}/swig/swig_clean.stamp)

add_custom_command(
		OUTPUT  ${SWIG_CLEAN_STAMP_FILE}
		COMMAND ${CMAKE_COMMAND} -E echo run swig dir clean ...
		COMMAND ${CMAKE_COMMAND} -E remove_directory  ${PCAM_OUTFILE_DIR}
		COMMAND ${CMAKE_COMMAND} -E remove_directory  ${PCAM_OUTPUT_DIR}
		COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/swig/
		COMMAND ${CMAKE_COMMAND} -E touch ${SWIG_CLEAN_STAMP_FILE}
		DEPENDS ${swig_dep_files}
		COMMENT "clean files..."
)

add_dependencies(pcam_build_stamp ${PCAM_LIBNAME})

#hack to run  before swig build
add_custom_target(clean_swig_dir DEPENDS ${SWIG_CLEAN_STAMP_FILE})
if(CMAKE_GENERATOR MATCHES "Make")
	add_dependencies( ${PCAM_LIBNAME}_swig_compilation clean_swig_dir)
else()	
	add_dependencies( ${PCAM_LIBNAME} clean_swig_dir)
endif ()



get_property(support_files TARGET ${PCAM_LIBNAME} PROPERTY SWIG_SUPPORT_FILES)
target_link_libraries(${PCAM_LIBNAME} tycam )
set(PCAM_LIB_SUPPORT_FILE_DIR ${CMAKE_CURRENT_BINARY_DIR}/swig_output PARENT_SCOPE)
set(PCAM_LIB_BINARY_NAME $<TARGET_FILE:${PCAM_LIBNAME}> PARENT_SCOPE)

if (${TARGET_LANGUAGE} STREQUAL "PYTHON")
	set_target_properties(${PCAM_LIBNAME} PROPERTIES PREFIX "_")
	#target_sources(${PCAM_LIBNAME} PUBLIC ${py_sources})
	target_include_directories(${PCAM_LIBNAME} PUBLIC ${PYTHON_NUMPY_INCLUDE_DIR})
	target_link_libraries(${PCAM_LIBNAME} ${PYTHON_LIBRARIES} )
endif()

if (${TARGET_LANGUAGE} STREQUAL "CSHARP")
	set_property(TARGET ${PCAM_LIBNAME} PROPERTY SWIG_COMPILE_OPTIONS -namespace pcammls )
endif()

if (${TARGET_LANGUAGE} STREQUAL "JAVA") # java
	target_include_directories(${PCAM_LIBNAME} PUBLIC ${JNI_INCLUDE_DIRS})
	target_link_libraries(${PCAM_LIBNAME} ${JNI_LIBRARIES} )
endif()

if(OpenCV_FOUND)
	target_link_libraries(${PCAM_LIBNAME} ${OpenCV_LIBS})
endif()

##################################################
install( TARGETS ${PCAM_LIBNAME} DESTINATION "${TARGET_LANGUAGE}")

if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=gnu++0x)
    add_definitions(-fpermissive)
endif()


