cmake_minimum_required(VERSION 3.0.2)
project(rqt_simple_launcher)

find_package(catkin REQUIRED COMPONENTS
  rqt_gui
  rqt_gui_py
)

catkin_python_setup()

catkin_package()
