cmake_minimum_required(VERSION 3.0.2)
project(grid_maze_costmap)

# Поиск зависимых пакетов
find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  sensor_msgs
  tf2
  tf2_ros
  dynamic_reconfigure
  nav_msgs
  geometry_msgs
  pluginlib
  visualization_msgs
  cv_bridge
  image_transport
)

# Настройка компиляции
catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    grid_maze_costmap
  CATKIN_DEPENDS 
    roscpp 
    costmap_2d 
    sensor_msgs 
    tf2 
    tf2_ros 
    dynamic_reconfigure 
    nav_msgs 
    geometry_msgs 
    pluginlib
    visualization_msgs
    cv_bridge
    image_transport
  DEPENDS 
    Boost
)

# Установка флагов компиляции
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Включение директорий
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Создание библиотеки
add_library(grid_maze_costmap
  src/layer.cpp
)

# Настройка зависимостей
add_dependencies(grid_maze_costmap 
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

# Линковка библиотек
target_link_libraries(grid_maze_costmap
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Установка плагина для costmap_2d
install(TARGETS grid_maze_costmap
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Установка заголовочных файлов
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

# Установка launch файлов
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

# Установка конфигурационных файлов
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)

# Установка плагина для pluginlib
install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)