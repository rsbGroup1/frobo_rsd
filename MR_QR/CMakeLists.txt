cmake_minimum_required(VERSION 2.6)
project(mr_qr)

find_package(OpenCV REQUIRED)

add_executable(${PROJECT_NAME} src/qrNode.cpp)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${ZBAR_LIBRARIES} zbar)

