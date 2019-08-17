cmake_minimum_required(VERSION 3.10.0)
project(pcammls_java)

find_package(Java REQUIRED)
include(UseJava)


#set(CMAKE_JAR_CLASSES_PREFIX com/percipio/pcammls)
file(GLOB JAVA_SUPPORT_FILES ${CMAKE_CURRENT_BINARY_DIR}/com/percipio/pcammls/*.java)
add_jar(pcammls_java ${JAVA_SUPPORT_FILES}  )
install_jar(pcammls_java "./")
message("===============")
message("${pcammls_jar}")
add_jar(test_java SOURCES  test.java  INCLUDE_JARS pcammls_java ENTRY_POINT test)
#add_dependencies(test_java pcammls_java)
install(FILES test.java DESTINATION "./" )
install_jar(test_java "./")
