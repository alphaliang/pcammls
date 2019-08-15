cmake_minimum_required(VERSION 3.10.0)
project(pcammls_java)

find_package(Java REQUIRED)
include(UseJava)
file(GLOB JAVA_SUPPORT_FILES ${PCAM_LIB_SUPPORT_FILE_DIR}/*.java)
add_jar(pcammls_java ${JAVA_SUPPORT_FILES})
install_jar(pcammls_java "./")

