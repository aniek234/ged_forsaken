# OGRE Base Project
#
# 2022-2023
#
# Originally based on the Official Ogre Tutorials 
# (Tutorial Framework) http://www.ogre3d.org/tikiwiki/ 
# heavily modified for CMU students - CIS5012. 

# Set the Executable name. 
set(APP "gameEngineAniek")

project(${APP})

cmake_minimum_required(VERSION 3.4)

set(CMAKE_CXX_STANDARD 11)

find_package(Threads)


set(HDRS
	./src/Game.h
	./src/Player.h
	./src/NPC.h
)

set(SRCS
	./src/Game.cpp
	./src/Player.cpp
	./src/NPC.cpp
	./src/main.cpp
)

include("tools/cmake/common.cmake")

# As of Ogre 1.13 Bullet is a plugin, so as long as
# its found by CMake we've nothing else to do. 

include("tools/cmake/bulletBase.cmake")

include("tools/cmake/ogreBase.cmake")


