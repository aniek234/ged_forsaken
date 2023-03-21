#pragma once

#include "Ogre.h"
#include "OgreApplicationContext.h"
#include "OgreInput.h"
#include "OgreRTShaderSystem.h"
#include "OgreApplicationContext.h"
#include "OgreCameraMan.h"

/* Bullet3 Physics */
#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"

using namespace Ogre;
using namespace OgreBites;

/* Forward declaration for the box class */
class Player;
class NPC;

/** Example Games class.
* Based (very heavily) on the Ogre3d examples.  Even uses OgreBytes (which I'd like to remove).
* Updated comments to Doxygen format. 
*/
class Game : public ApplicationContext, public InputListener
{
private:
    /**
    * Ogre Scene Manager.
    */
    SceneManager* scnMgr;

   /**
    * Collision configuration.
    */
    btDefaultCollisionConfiguration* collisionConfiguration;

    /**
    * The default collision dispatcher. For parallel processing you can use a different dispatcher (see Extras/BulletMultiThreaded)
    */
    btCollisionDispatcher* dispatcher ;

    /**
    * btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
    */
    btBroadphaseInterface* overlappingPairCache;

    /**
    * The default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    */
    btSequentialImpulseConstraintSolver* solver;

    /**
    * The dynamics world (physics/collision world).
    */
    btDiscreteDynamicsWorld* dynamicsWorld;

    /**
    * Keep track of the shapes, we release memory at exit.
    * make sure to re-use collision shapes among rigid bodies whenever possible!
    */
    btAlignedObjectArray<btCollisionShape*> collisionShapes;

    /** 
     * The Player object
     */
    Player *player;

    /**
     * The NPC object 
     */
    NPC* npc;

    /**
     * Key States
     * 
     */
    bool aDown, wDown, fDown, jDown, dDown;

public:
  /** 
  * Creates the object, sets all pointers to nullptr.
  */
	Game();

  /**
  * Destructor (virtual), as this is virtual that of the sub class will also be called.
  */
	virtual ~Game();

  /**
  * Carries out all setup, includes lighting, scene objects.
  */
  void setup();

  /**
  * Sets up the camera
  */
	void setupCamera();

  /**
  * Quick and dirty box mesh, essentially this is a mix of the Ogre code to setup a box - from example.
  * Added to this is the setup for the bullet3 collision box and rigid body.
  */
	void setupBoxMesh();

  /**
  * Player setup
  */
  void setupPlayer();

  /**
  * NPC setup
  */
  void setupNPC();

  /**
  * Turns on on the coffee machine.
  */
  void setupFloor();

  /**
  * Creates, lights and adds them to the scene.  All based on the sample code, needs moving out into a level class.
  */
	void setupLights();

  /**
  * Overload of the keyPressed method.
  * @param evt, a KeyboardEvent
  */
	bool keyPressed(const KeyboardEvent& evt);

  /**
  * Overload of the keyReleased method.
  * @param evt, a KeyboardEvent
  */
	bool keyReleased(const KeyboardEvent& evt);

  /**
  * Overload of the mouseMoved method.
  * @param evt, a MouseEvent
  */
	bool mouseMoved(const MouseMotionEvent& evt);

  /**
  * Ogre wraps the game loop, but we've registered as being interested in FrameEvents (through inheritance).
  * This method is called by the framework before rendering the frame.
  * @param evt, FrameEvent.
  */
	bool frameStarted (const FrameEvent &evt);

  /**
  * Ogre wraps the game loop, but we've registered as being interested in FrameEvents (through inheritance).
  * This method is called by the framework after rendering the frame.
  * @param evt, FrameEvent.
  */
	bool frameEnded(const FrameEvent &evt);

  /**
  * Sets up the bullet environment
  */
  void bulletInit();

};
