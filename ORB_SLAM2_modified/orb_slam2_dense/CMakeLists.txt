project(orb_slam2_dense)
cmake_minimum_required(VERSION 2.6)
find_package(Qt4 REQUIRED)

include_directories(${QT_INCLUDES} ${CMAKE_CURRENT_BINARY_DIR})

set(orb_slam2_dense_SRCS orb_slam2_dense.cpp main.cpp)
qt4_automoc(${orb_slam2_dense_SRCS})
add_executable(orb_slam2_dense ${orb_slam2_dense_SRCS})
target_link_libraries(orb_slam2_dense ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
install(TARGETS orb_slam2_dense RUNTIME DESTINATION bin)
