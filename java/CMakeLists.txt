cmake_minimum_required(VERSION 3.10.0)
project(pcammls_java)

find_package(Java REQUIRED)
include(UseJava)

file(GLOB_RECURSE JAVA_SUPPORT_FILES "${PCAM_LIB_SUPPORT_FILE_DIR}/*.java")
add_jar(pcammls_java ${JAVA_SUPPORT_FILES} "${PCAM_LIB_SUPPORT_FILE_DIR}")
add_jar(test_java SOURCES  test.java  INCLUDE_JARS pcammls_java ENTRY_POINT test)
install_jar(test_java DESTINATION "./")
install_jar(pcammls_java DESTINATION "./")


