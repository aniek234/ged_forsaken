# OGRE / Bullet Base Project /*
# Testing some ideas, may no longer need this as there is now an Ogre addon for bullet physics. 
#*/


# For later Bullet3 tutorials
find_package(Bullet REQUIRED)

include_directories(${BULLET_INCLUDE_DIRS})

target_link_libraries(${APP} ${BULLET_LIBRARIES})

# Don't need these, Bullet3 is static!
#link_directories(${BULLET_LIBRARIES})
#target_link_libraries(${APP} BulletDynamics BulletCollision LinearMath)
