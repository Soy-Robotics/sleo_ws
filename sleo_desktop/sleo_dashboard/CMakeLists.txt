cmake_minimum_required(VERSION 2.8.3)
project(sleo_dashboard)

find_package(catkin REQUIRED COMPONENTS  sleo_msgs  roscpp  roslint  rqt_gui  rqt_gui_cpp  std_msgs)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

set(sleo_dashboard_SRCS
  src/sleo_dashboard/sleo_dashboard.cpp
)

set(sleo_dashboard_HDRS
  include/sleo_dashboard/sleo_dashboard.h
)

set(sleo_dashboard_UIS
  src/sleo_dashboard/sleo_dashboard.ui
)

set(sleo_dashboard_INCLUDE_DIRECTORIES
  include
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}"
)
if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
endif()


catkin_package(
  INCLUDE_DIRS ${sleo_dashboard_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS sleo_msgs  roscpp  roslint  rqt_gui  rqt_gui_cpp  std_msgs
)
catkin_python_setup()

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(sleo_dashboard_MOCS ${sleo_dashboard_HDRS})
else()
  qt4_wrap_cpp(sleo_dashboard_MOCS ${sleo_dashboard_HDRS})
endif()
# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_ui(sleo_dashboard_UIS_H ${sleo_dashboard_UIS})
else()
  qt4_wrap_ui(sleo_dashboard_UIS_H ${sleo_dashboard_UIS})
endif()

set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

include_directories(${sleo_dashboard_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${sleo_dashboard_SRCS} ${sleo_dashboard_MOCS} ${sleo_dashboard_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(DIRECTORY plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/sleo_dashboard
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/sleo_dashboard
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/sleo_dashboard
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/ 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

