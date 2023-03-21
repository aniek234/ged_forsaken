#/*
#-----------------------------------------------------------------------------
# Filename:    ogreBase.cmake
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


# Ask CMake to find OGRE and identify it's properties. 
find_package(OGRE 13.4 REQUIRED COMPONENTS Bites RTShaderSystem Bullet)


target_link_libraries(${APP} OgreBites OgreRTShaderSystem OgreBullet)
target_link_libraries(${APP} ${OGRE_LIBRARIES} ${OGRE_Overlay_LIBRARIES} )

# Install Target
# Directories created by the common script, this one just
# copies over the ogre stuff. 

if(WIN32)


	install(DIRECTORY ${CMAKE_SOURCE_DIR}/dist/media
		DESTINATION ./
		CONFIGURATIONS Release RelWithDebInfo Debug
	)

	install(FILES
		${CMAKE_SOURCE_DIR}/dist/bin/resources.cfg
		DESTINATION bin
		CONFIGURATIONS Release RelWithDebInfo
	)

	install(FILES
		${CMAKE_SOURCE_DIR}/dist/bin/resources_d.cfg
		DESTINATION bin
		CONFIGURATIONS Debug
		RENAME resources.cfg
	)

	# Got lazy - going stuff all the dlls in for the release. 
	file(GLOB_RECURSE OG_DLL_FILES  ${OGRE_CONFIG_DIR}/*.dll)
	
	install(FILES ${OG_DLL_FILES}
		DESTINATION bin
		CONFIGURATIONS Release RelWithDebInfo Debug
	)
	
   install(FILES ${Boost_DATE_TIME_LIBRARY_DEBUG}
      ${Boost_THREAD_LIBRARY_DEBUG}
      DESTINATION bin
      CONFIGURATIONS Debug
   )
endif(WIN32)

if(UNIX)

	install(DIRECTORY ${CMAKE_SOURCE_DIR}/dist/media
		DESTINATION ./
		CONFIGURATIONS Release RelWithDebInfo Debug
	)

	install(FILES
		${CMAKE_SOURCE_DIR}/dist/bin/resources.cfg
		DESTINATION bin
		CONFIGURATIONS Release RelWithDebInfo
	)

    install(FILES
		${CMAKE_SOURCE_DIR}/dist/bin/resources_d.cfg
		DESTINATION bin
		CONFIGURATIONS Debug
        RENAME resources.cfg
	)

endif(UNIX)
