# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;controller_interface;hardware_interface;pluginlib;control_toolbox;dynamic_reconfigure;std_msgs;realtime_tools;geometry_msgs;tf".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lwindmill_controller".split(';') if "-lwindmill_controller" != "" else []
PROJECT_NAME = "windmill_controller"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
