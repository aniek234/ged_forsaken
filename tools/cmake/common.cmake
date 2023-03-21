#/*
#-----------------------------------------------------------------------------
# Filename:    common.cmake
#-----------------------------------------------------------------------------
#
# This source file is based on
#   ___                 __    __ _ _    _
#  /___\__ _ _ __ ___  / / /\ \ (_) | _(_)
# //  // _` | '__/ _ \ \ \/  \/ / | |/ / |
#/ \_// (_| | | |  __/  \  /\  /| |   <| |
#\___/ \__, |_|  \___|   \/  \/ |_|_|\_\_|
#      |___/
#      Tutorial Framework
#      http://www.ogre3d.org/tikiwiki/
#
# Modified for CMU students.
#-----------------------------------------------------------------------------
#*/

if (CMAKE_BUILD_TYPE STREQUAL "")
  # CMake defaults to leaving CMAKE_BUILD_TYPE empty. This screws up
  # differentiation between debug and release builds.
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build, options are: None (CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif ()

set(CMAKE_DEBUG_POSTFIX "_d")

set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/dist")

if(WIN32)
  set(CMAKE_WIN32_EXECUTABLE ON)
elseif(APPLE)
  set(CMAKE_MACOSX_BUNDLE ON)
endif()

add_executable(${APP} ${HDRS} ${SRCS})

set_target_properties(${APP} PROPERTIES DEBUG_POSTFIX _d)

#target_link_libraries(${APP} OgreBites OgreRTShaderSystem)
#target_link_libraries(${APP} ${OGRE_LIBRARIES} ${OGRE_Overlay_LIBRARIES} )

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/dist/bin)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/dist/media)

# post-build copy for win32
if(WIN32 AND NOT MINGW)
	add_custom_command( TARGET ${APP} PRE_BUILD
		COMMAND if not exist .\\dist\\bin mkdir .\\dist\\bin )
	add_custom_command( TARGET ${APP} POST_BUILD
		COMMAND copy \"$(TargetPath)\" .\\dist\\bin )
endif(WIN32 AND NOT MINGW)

if(MINGW OR UNIX)
	set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/dist/bin)
endif(MINGW OR UNIX)

if(WIN32)
   set_target_properties(${APP} PROPERTIES LINK_FLAGS_DEBUG "/SUBSYSTEM:CONSOLE")
   set_target_properties(${APP} PROPERTIES COMPILE_DEFINITIONS_DEBUG "_CONSOLE")
   set_target_properties(${APP} PROPERTIES LINK_FLAGS_RELWITHDEBINFO "/SUBSYSTEM:CONSOLE")
   set_target_properties(${APP} PROPERTIES COMPILE_DEFINITIONS_RELWITHDEBINFO "_CONSOLE")
   set_target_properties(${APP} PROPERTIES LINK_FLAGS_RELEASE "/SUBSYSTEM:WINDOWS  /ENTRY:mainCRTStartup")
   set_target_properties(${APP} PROPERTIES LINK_FLAGS_MINSIZEREL "/SUBSYSTEM:WINDOWS  /ENTRY:mainCRTStartup")
endif(WIN32)


if(WIN32)
	install(TARGETS ${APP}
		RUNTIME DESTINATION bin
		CONFIGURATIONS All)

	# https://stackoverflow.com/questions/23950887/does-cmake-offer-a-method-to-set-the-working-directory-for-a-given-build-system
	set_target_properties(${APP} PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/dist/bin")
endif(WIN32)
	
if(UNIX)
	install(TARGETS ${APP}
		RUNTIME DESTINATION bin
		CONFIGURATIONS All)
endif(UNIX)
