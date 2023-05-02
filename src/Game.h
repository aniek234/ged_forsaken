#pragma once

#include "Ogre.h"
#include "OgreApplicationContext.h"
#include "OgreInput.h"
#include "OgreRTShaderSystem.h"
#include "OgreCameraMan.h"
#include "../build/GameObject.h"
#include "../build/InputHelper.h"

#include "../build/InputComponent.h"
#include "../build/PlayerInputComponent.h"

using namespace Ogre;
using namespace OgreBites;

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


        std::vector<GameObject> gameObjects;

        /** 
         * The Player object
         */
        GameObject *player;

        /**
         * The NPC object 
         */
        GameObject* npc;

        /**
         * Input helper 
         */
        InputHelper* inputHelper;

        /**
        * Keep track of the shapes, we release memory at exit.
        * make sure to re-use collision shapes among rigid bodies whenever possible!
        */
        btAlignedObjectArray<btCollisionShape*> collisionShapes;


        // Set all the Bullet stuff to nullptr - trap uninitialised pointer errors.
        btDefaultCollisionConfiguration* collisionConfiguration = nullptr;
        btCollisionDispatcher* dispatcher = nullptr;
        btBroadphaseInterface* overlappingPairCache = nullptr;
        btSequentialImpulseConstraintSolver* solver = nullptr;
        btDiscreteDynamicsWorld* dynamicsWorld = nullptr;

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

        void bulletInit();

        /**
        * Player setup
        */
        void setupPlayer();

        /**
        * NPC setup
        */
        void setupNPC();

        /**
        * Floor setup
        */
        void setupFloor();

        /**
        * Creates, lights and adds them to the scene.  All based on the sample code, needs moving out into a level class.
        */
        void setupLights();


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


};
