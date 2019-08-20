cmake_minimum_required(VERSION 3.10.0)
project(pcammls_java)

find_package(Java REQUIRED)
#include(UseJava)

function(add_jar _TARGET_NAME)
    cmake_parse_arguments(_add_jar
      ""
      "WORK_PATH;OUTPUT_DIR;OUTPUT_NAME;ENTRY_POINT;MANIFEST"
      "SOURCES;INCLUDE_JARS;GENERATE_NATIVE_HEADERS"
      ${ARGN}
    )
	set(_INCLUDE_JARS ${_add_jar_INCLUDE_JARS})
    set(_JAVA_SOURCE_FILES ${_add_jar_SOURCES} ${_add_jar_UNPARSED_ARGUMENTS})
	set(CMAKE_FILES_TARGET_DIR "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${_TARGET_NAME}.dir/")
	set(CMAKE_JAVA_CLASS_OUTPUT_PATH "${CMAKE_FILES_TARGET_DIR}/java_class")
	set(WORK_PATH ${_add_jar_WORK_PATH})
	if (_add_jar_ENTRY_POINT)
		set(_ENTRY_POINT_PARAM "--main-class=${_add_jar_ENTRY_POINT}")
	endif()
	if (EXISTS ${WORK_PATH})
		set (WORK_PATH_VALID TRUE)
	else() 
		set (WORK_PATH_VALID FALSE)
	endif()
	set(JAVA_INCLUDE_PATH ".")
	set(_INCLUDE_DEPENDS "")
	foreach (_path IN LISTS _INCLUDE_JARS)
        if (TARGET ${_path})
			get_target_property (_jar_path ${_path} JAR_FILE)
			list (APPEND _INCLUDE_DEPENDS "${_jar_path}")
			list (APPEND JAVA_INCLUDE_PATH "${_jar_path}")
        elseif (EXISTS "${_path}")
			list (APPEND JAVA_INCLUDE_PATH "${_path}")
        endif()
	endforeach()

	set(complied_java_class "")
	foreach(_JAVA_SOURCE_FILE IN LISTS _JAVA_SOURCE_FILES)
	    get_filename_component(_JAVA_FULL ${_JAVA_SOURCE_FILE} ABSOLUTE)
		get_filename_component(_JAVA_PATH ${_JAVA_FULL} PATH)
		get_filename_component(_JAVA_FILE ${_JAVA_FULL} NAME_WE)
		if(WORK_PATH_VALID)
			file(RELATIVE_PATH _JAVA_REL_PATH ${WORK_PATH} ${_JAVA_PATH})
			set(_JAVA_CLASS_DIR "${CMAKE_JAVA_CLASS_OUTPUT_PATH}/${_JAVA_REL_PATH}/")
			set(_JAVA_CLASS_FILE "${_JAVA_CLASS_DIR}/${_JAVA_FILE}.class")
		else()
			set(_JAVA_CLASS_DIR "${CMAKE_JAVA_CLASS_OUTPUT_PATH}/")
			set(_JAVA_CLASS_FILE "${CMAKE_JAVA_CLASS_OUTPUT_PATH}/${_JAVA_FILE}.class")
		endif()
		add_custom_command(
			OUTPUT ${_JAVA_CLASS_FILE}
			COMMAND ${CMAKE_COMMAND} -E echo 
			         ${Java_JAVAC_EXECUTABLE} 
			         -d  ${_JAVA_CLASS_DIR} ${_JAVA_FULL}
					 --source-path  ${_JAVA_PATH}
					 --class-path ${JAVA_INCLUDE_PATH}
			COMMAND ${Java_JAVAC_EXECUTABLE} 
			         -d  ${_JAVA_CLASS_DIR} ${_JAVA_FULL}
					 --source-path  ${_JAVA_PATH}
					 --class-path "${JAVA_INCLUDE_PATH}"
	        DEPENDS ${_INCLUDE_DEPENDS} ${_JAVA_FULL}
			WORKING_DIRECTORY ${WORK_PATH}
			COMMENT "compiling ${_JAVA_FULL}"
		)
		list(APPEND complied_java_class  "${_JAVA_CLASS_FILE}")
	endforeach()
	set(rel_complied_java_class "")
	foreach(_complied_name IN LISTS complied_java_class)
			file(RELATIVE_PATH _JAVA_REL_PATH ${CMAKE_JAVA_CLASS_OUTPUT_PATH} ${_complied_name})
			list(APPEND rel_complied_java_class ${_JAVA_REL_PATH})
	endforeach()
	set(TARGET_JAR_OUTPUT_FILE ${CMAKE_CURRENT_BINARY_DIR}/${_TARGET_NAME}.jar)
	add_custom_command(
		OUTPUT ${TARGET_JAR_OUTPUT_FILE}
		COMMAND ${Java_JAR_EXECUTABLE} 
				-cvf ${TARGET_JAR_OUTPUT_FILE}  ${rel_complied_java_class}
		COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_FILES_TARGET_DIR}/java_class_compiled_${_TARGET_NAME}
		WORKING_DIRECTORY ${CMAKE_JAVA_CLASS_OUTPUT_PATH}
		DEPENDS  ${complied_java_class}
		COMMENT "create jar pacakge ${TARGET_JAR_OUTPUT_FILE}"
	)

	#entry
	set (_TARGET_BUILD_OK_STAMP "${CMAKE_FILES_TARGET_DIR}/java_class_compiled_${_TARGET_NAME}")
	if (_ENTRY_POINT_PARAM)
		add_custom_command(
		OUTPUT ${_TARGET_BUILD_OK_STAMP}
		COMMAND ${Java_JAR_EXECUTABLE} --update -f ${TARGET_JAR_OUTPUT_FILE}  ${_ENTRY_POINT_PARAM} 
		COMMAND ${CMAKE_COMMAND} -E touch ${_TARGET_BUILD_OK_STAMP}
			WORKING_DIRECTORY ${CMAKE_JAVA_CLASS_OUTPUT_PATH}
		DEPENDS  ${TARGET_JAR_OUTPUT_FILE}
			COMMENT "update jar pacakge ${TARGET_JAR_OUTPUT_FILE}" )
	else()
		add_custom_command(
		OUTPUT ${_TARGET_BUILD_OK_STAMP}
		COMMAND ${CMAKE_COMMAND} -E touch ${_TARGET_BUILD_OK_STAMP}
			WORKING_DIRECTORY ${CMAKE_JAVA_CLASS_OUTPUT_PATH}
		DEPENDS  ${TARGET_JAR_OUTPUT_FILE} )

	endif()

	add_custom_target(${_TARGET_NAME}  
		ALL DEPENDS ${_TARGET_BUILD_OK_STAMP}
		COMMENT "compile ${_TARGET_NAME}" )

    set_property( TARGET ${_TARGET_NAME} PROPERTY INSTALL_FILES ${TARGET_JAR_OUTPUT_FILE})
    set_property( TARGET ${_TARGET_NAME} PROPERTY JAR_FILE ${TARGET_JAR_OUTPUT_FILE})

endfunction()

function(INSTALL_JAR _TARGET_NAME)
    if (ARGC EQUAL 2)
      set (_DESTINATION ${ARGV1})
    else()
      cmake_parse_arguments(_install_jar
        ""
        "DESTINATION;COMPONENT"
        ""
        ${ARGN})
      if (_install_jar_DESTINATION)
        set (_DESTINATION ${_install_jar_DESTINATION})
      else()
        message(SEND_ERROR "install_jar: ${_TARGET_NAME}: DESTINATION must be specified.")
      endif()

      if (_install_jar_COMPONENT)
        set (_COMPONENT COMPONENT ${_install_jar_COMPONENT})
      endif()
    endif()

    get_property(__FILES
        TARGET
            ${_TARGET_NAME}
        PROPERTY
            INSTALL_FILES
    )
    set_property(
        TARGET
            ${_TARGET_NAME}
        PROPERTY
            INSTALL_DESTINATION
            ${_DESTINATION}
    )

    if (__FILES)
        install(
            FILES
                ${__FILES}
            DESTINATION
                ${_DESTINATION}
            ${_COMPONENT}
        )
    else ()
        message(SEND_ERROR "install_jar: The target ${_TARGET_NAME} is not known in this scope.")
    endif ()
endfunction()



file(GLOB JAVA_SUPPORT_FILES "${PCAM_LIB_SUPPORT_FILE_DIR}/com/percipio/pcammls/*.java")
add_jar(pcammls_java ${JAVA_SUPPORT_FILES} WORK_PATH "${PCAM_LIB_SUPPORT_FILE_DIR}")
add_jar(test_java SOURCES  test.java  INCLUDE_JARS pcammls_java ENTRY_POINT test)
install_jar(test_java DESTINATION "./")
install_jar(pcammls_java DESTINATION "./")


