### You should not need to change anything below.
LINUX=1
MACOSX=0

# Compilers
CC=/usr/bin/gcc
CXX=/usr/bin/c++

# Paths
MAPPING_ROOT=/home/robotics/ros_workspace/hmc-robot-code/mygmapping/build/gmapping_export
LIBDIR=/home/robotics/ros_workspace/hmc-robot-code/mygmapping/build/gmapping_export/lib
BINDIR=/home/robotics/ros_workspace/hmc-robot-code/mygmapping/build/gmapping_export/bin

# Build tools
PRETTY=/home/robotics/ros_workspace/hmc-robot-code/mygmapping/build/gmapping_export/build_tools/pretty_compiler
MESSAGE=/home/robotics/ros_workspace/hmc-robot-code/mygmapping/build/gmapping_export/build_tools/message
TESTLIB=/home/robotics/ros_workspace/hmc-robot-code/mygmapping/build/gmapping_export/build_tools/testlib

# QT support
MOC=moc-qt3
QT_LIB=-lqt-mt
QT_INCLUDE=-I/usr/include/qt3

# ARIA support
ARIA_LIB=
ARIA_INCLUDE=


# # KDE support
# KDE_LIB=
# KDE_INCLUDE=
# UIC=

# Generic makefiles
MAKEFILE_GENERIC=/home/robotics/ros_workspace/hmc-robot-code/mygmapping/build/gmapping_export/build_tools/Makefile.generic-shared-object
MAKEFILE_APP=/home/robotics/ros_workspace/hmc-robot-code/mygmapping/build/gmapping_export/build_tools/Makefile.app
MAKEFILE_SUBDIRS=/home/robotics/ros_workspace/hmc-robot-code/mygmapping/build/gmapping_export/build_tools/Makefile.subdirs


# Flags
CPPFLAGS+=-DLINUX -I/home/robotics/ros_workspace/hmc-robot-code/mygmapping/build/gmapping_export 
CXXFLAGS+=
LDFLAGS+= -Xlinker -rpath /home/robotics/ros_workspace/hmc-robot-code/mygmapping/build/gmapping_export/lib
CARMENSUPPORT=0
ARIASUPPORT=0



include /home/robotics/ros_workspace/hmc-robot-code/mygmapping/build/gmapping_export/manual.mk

