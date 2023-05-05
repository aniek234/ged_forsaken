/*-------------------------------------------------------------------------
Significant portions of this project are based on the Ogre Tutorials
- https://ogrecave.github.io/ogre/api/1.10/tutorials.html
Copyright (c) 2000-2013 Torus Knot Software Ltd

Manual generation of meshes from here:
- http://wiki.ogre3d.org/Generating+A+Mesh

Ogre / Bullet connectivity from here:
 https://oramind.com/ogre-bullet-a-beginners-basic-guide/

*/

#include <exception>
#include <iostream>
#include "Game.h"
#include "OgreBullet.h"


Game::Game() : ApplicationContext("OgreTutorialApp")
{
    // Set all the Ogre stuff to nullptr - trap uninitialised pointer errors.
    scnMgr = nullptr;

    // array of gameobjects
    gameObjects;

    // Same for bullet
    /*btDefaultCollisionConfiguration *collisionConfiguration = nullptr;
    btCollisionDispatcher *dispatcher = nullptr;
    btBroadphaseInterface *overlappingPairCache = nullptr;
    btSequentialImpulseConstraintSolver *solver = nullptr;
    btDiscreteDynamicsWorld *dynamicsWorld = nullptr;*/

    player = nullptr;

    npc = nullptr;

    inputHelper = nullptr;
}

Game::~Game()
{
    // cleanup in the reverse order of creation/initialization
    // remove the rigidbodies from the dynamics world and delete them
    for (int i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
    {
        btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);

        if (body && body->getMotionState())
        {
            delete body->getMotionState();
        }

        dynamicsWorld->removeCollisionObject(obj);
        delete obj;
    }

    // delete collision shapes
    for (int j = 0; j < collisionShapes.size(); j++)
    {
        btCollisionShape* shape = collisionShapes[j];
        collisionShapes[j] = 0;
        delete shape;
    }

    // delete dynamics world
    delete dynamicsWorld;
    dynamicsWorld = nullptr;

    // delete solver
    delete solver;
    solver = nullptr;

    // delete broadphase
    delete overlappingPairCache;
    overlappingPairCache = nullptr;

    // delete dispatcher
    delete dispatcher;
    dispatcher = nullptr;

    delete collisionConfiguration;
    collisionConfiguration = nullptr;

    // next line is optional: it will be cleared by the destructor when the array goes out of scope
    collisionShapes.clear();
}

void Game::setup()
{
    // do not forget to call the base first
    ApplicationContext::setup();

    inputHelper = new InputHelper();

    addInputListener(inputHelper);

    // get a pointer to the already created root
    Root *root = getRoot();
    scnMgr = root->createSceneManager();

    // register our scene with the RTSS
    RTShader::ShaderGenerator *shadergen = RTShader::ShaderGenerator::getSingletonPtr();
    shadergen->addSceneManager(scnMgr);

    // initialize bullet
    bulletInit();

    // setup game objects and camera/lights
    setupPlayer();
    setupNPC(-20.0f, 20.0f, 100.0f);
    setupCamera();
    setupLights();

    // setup tunnel
    setupFloor();
    setupCeiling();
    setupWall1();
    setupWall2();
    setupWall3();
    setupWall4();
}

// to initialize Bullet
void Game::bulletInit()
{
    /// collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
    collisionConfiguration = new btDefaultCollisionConfiguration();

    /// use the default collision dispatcher. For parallel processing you can use a different dispatcher (see Extras/BulletMultiThreaded)
    dispatcher = new btCollisionDispatcher(collisionConfiguration);

    /// btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
    overlappingPairCache = new btDbvtBroadphase();

    /// the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    solver = new btSequentialImpulseConstraintSolver;

    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);

    dynamicsWorld->setGravity(btVector3(0, -10, 0));
}

void Game::setupCamera()
{
    // Create Camera
    Camera *cam = scnMgr->createCamera("myCam");

    // Setup Camera
    cam->setNearClipDistance(5);

    // Position Camera - to do this it must be attached to a scene graph and added to the scene.
    SceneNode* camNode = player->getPlayerNode()->createChildSceneNode();
    camNode->setPosition(0, 100, 600);
    camNode->lookAt(Vector3(0, 0, 0), Node::TransformSpace::TS_WORLD);
    camNode->attachObject(cam);

    // Setup viewport for the camera.
    Viewport *vp = getRenderWindow()->addViewport(cam);
    vp->setBackgroundColour(ColourValue(0, 0, 0));

    // link the camera and view port.
    cam->setAspectRatio(Real(vp->getActualWidth()) / Real(vp->getActualHeight()));
}

void Game::setupLights()
{
    // Setup Ambient light
    scnMgr->setAmbientLight(ColourValue(0, 0, 0));
    scnMgr->setShadowTechnique(ShadowTechnique::SHADOWTYPE_STENCIL_MODULATIVE);

    // Add a spotlight
    Light* spotLight = scnMgr->createLight("SpotLight");

    // Configure
    spotLight->setDiffuseColour(0, 0, 1.0);
    spotLight->setSpecularColour(0, 0, 1.0);
    spotLight->setType(Light::LT_SPOTLIGHT);
    spotLight->setSpotlightRange(Degree(35), Degree(50));

    // Create a scene node for the spotlight
    SceneNode* spotLightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    spotLightNode->setDirection(-1, -1, 0);
    spotLightNode->setPosition(Vector3(200, 200, 0));

    // Add spotlight to the scene node.
    spotLightNode->attachObject(spotLight);

    // Create directional light
    Light* directionalLight = scnMgr->createLight("DirectionalLight");

    // Configure the light
    directionalLight->setType(Light::LT_DIRECTIONAL);
    directionalLight->setDiffuseColour(ColourValue(0.4, 0, 0));
    directionalLight->setSpecularColour(ColourValue(0.4, 0, 0));

    // Setup a scene node for the directional lightnode.
    SceneNode* directionalLightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    directionalLightNode->attachObject(directionalLight);
    directionalLightNode->setDirection(Vector3(0, -1, 1));

    // Create a point light
    Light* pointLight = scnMgr->createLight("PointLight");

    // Configure the light
    pointLight->setType(Light::LT_POINT);
    pointLight->setDiffuseColour(0.3, 0.3, 0.3);
    pointLight->setSpecularColour(0.3, 0.3, 0.3);

    // setup the scene node for the point light
    SceneNode* pointLightNode = scnMgr->getRootSceneNode()->createChildSceneNode();

    // Configure the light
    pointLightNode->setPosition(Vector3(0, 150, 250));

    // Add the light to the scene.
    pointLightNode->attachObject(pointLight);
}

/**
* @brief Create a Player using the GameObject class.
*/
void Game::setupPlayer()
{
    // box mass.
    float mass = 1.0f;

    // Axis
    Vector3 axis(10.0, 0.0, 10.0);
    axis.normalise();

    // angle
    Radian rads(Degree(40.0));

    player = new GameObject();
    player->setup(scnMgr, dynamicsWorld, mass);
    // TODO: give components
    player->setGraphicsComponent(new GraphicsComponent());
    player->setPhysicsComponent(new PhysicsComponent(dynamicsWorld, mass, player->colShape));
    player->setInputComponent(new PlayerInputComponent());

    player->setRotation(axis, rads);
    player->physicsComponent->setPosition(0.0f, 20.0f, 200.0f);
    player->syncSceneNode();

    gameObjects.push_back(*player);

    collisionShapes.push_back(player->getCollisionShape());
    dynamicsWorld->addRigidBody(player->getRigidBody());
}

/**
* @brief Create an NPC using the GameObject class.
*/
void Game::setupNPC(float pos_x, float pos_y, float pos_z)
{
    // box mass.
    float mass = 1.0f;

    // Axis
    Vector3 axis(1.0, 0.0, 0.0);
    axis.normalise();

    // angle
    Radian rads(Degree(0.0));

    npc = new GameObject();
    npc->setup(scnMgr, dynamicsWorld, mass);
    npc->setGraphicsComponent(new GraphicsComponent());
    npc->setPhysicsComponent(new PhysicsComponent(dynamicsWorld, mass, npc->colShape));

    npc->setRotation(axis, rads);
    npc->physicsComponent->setPosition(pos_x, pos_y, pos_z);

    npc->AddWayPoint(btVector3(200.0f, 20.0f, -100.0f));
    npc->AddWayPoint(btVector3(-200.0f, 20.0f, 100.0f));
    /*npc->AddWayPoint(btVector3(5.0f, 20.0f, -300.0f));
    npc->AddWayPoint(btVector3(300.0f, 20.0f, 0.0f));*/

    gameObjects.push_back(*npc);

    collisionShapes.push_back(npc->getCollisionShape());
    dynamicsWorld->addRigidBody(npc->getRigidBody());
}

/**
* @brief Create the floor
*/
void Game::setupFloor()
{
    // Create a plane
    Plane plane(Vector3::UNIT_Y, 0);

    // Define the plane mesh
    MeshManager::getSingleton().createPlane(
        "ground", RGN_DEFAULT,
        plane,
        1500, 15000, 20, 20,
        true,
        1, 5, 5,
        Vector3::UNIT_Z);

    // Create an entity for the ground
    Entity *groundEntity = scnMgr->createEntity("ground");

    // Setup ground entity
    //  Shadows off
    groundEntity->setCastShadows(false);

    // Material - Examples is the resources file,
    // Rockwall (texture/properties) is defined inside it.
    groundEntity->setMaterialName("Examples/Rockwall");

    // Create a scene node to add the mesh too.
    SceneNode *thisSceneNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    thisSceneNode->attachObject(groundEntity);

    // the ground is a cube of side 100 at position y = 0.
    // the sphere will hit it at y = -6, with center at -5
    btCollisionShape *groundShape = new btBoxShape(btVector3(btScalar(1500.0), btScalar(50.0), btScalar(15000.0)));

    collisionShapes.push_back(groundShape);

    btTransform groundTransform;
    groundTransform.setIdentity();

    Vector3 pos = thisSceneNode->_getDerivedPosition();

    // Box is 100 deep (dimensions are 1/2 heights)
    // but the plane position is flat.
    groundTransform.setOrigin(btVector3(pos.x, pos.y - 50.0, pos.z));

    Quaternion quat2 = thisSceneNode->_getDerivedOrientation();
    groundTransform.setRotation(btQuaternion(quat2.x, quat2.y, quat2.z, quat2.w));

    btScalar mass(0.);

    // rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
        groundShape->calculateLocalInertia(mass, localInertia);

    // using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState *myMotionState = new btDefaultMotionState(groundTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
    btRigidBody *body = new btRigidBody(rbInfo);

    //   body->setRestitution(0.0);

    // add the body to the dynamics world
    dynamicsWorld->addRigidBody(body);
}
/**
* @brief Creates the ceiling
*/
void Game::setupCeiling()
{
    // Create a plane
    Plane plane(Vector3::NEGATIVE_UNIT_Y, -750);

    // Define the plane mesh
    MeshManager::getSingleton().createPlane(
        "ceiling", RGN_DEFAULT,
        plane,
        1500, 15000, 20, 20,
        true,
        1, 5, 5,
        Vector3::UNIT_Z);

    // Create an entity for the ceiling
    Entity* ceilingEntity = scnMgr->createEntity("ceiling");

    // Setup ceiling entity
    //  Shadows off
    ceilingEntity->setCastShadows(false);

    // Material - Examples is the resources file,
    // Rockwall (texture/properties) is defined inside it.
    ceilingEntity->setMaterialName("Examples/Rockwall");

    // Create a scene node to add the mesh too.
    SceneNode* thisSceneNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    thisSceneNode->attachObject(ceilingEntity);

    // the ceiling is a cube of side 100 at position y = 0.
    // the sphere will hit it at y = -6, with center at -5
    btCollisionShape* ceilingShape = new btBoxShape(btVector3(btScalar(750.), btScalar(50.), btScalar(15000.)));

    collisionShapes.push_back(ceilingShape);

    btTransform ceilingTransform;
    ceilingTransform.setIdentity();

    Vector3 pos = thisSceneNode->_getDerivedPosition();

    // Box is 100 deep (dimensions are 1/2 heights)
    // but the plane position is flat.
    ceilingTransform.setOrigin(btVector3(pos.x, pos.y + 650, pos.z));

    Quaternion quat2 = thisSceneNode->_getDerivedOrientation();
    ceilingTransform.setRotation(btQuaternion(quat2.x, quat2.y, quat2.z, quat2.w));

    btScalar mass(0.);

    // rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
        ceilingShape->calculateLocalInertia(mass, localInertia);

    // using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* myMotionState = new btDefaultMotionState(ceilingTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, ceilingShape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);

    //   body->setRestitution(0.0);

    // add the body to the dynamics world
    dynamicsWorld->addRigidBody(body);
}
/**
* @brief Creates the walls
*/
void Game::setupWall1()
{
    // Create a plane
    Plane wallPlane(Vector3::UNIT_X, -750);

    // Define the plane mesh
    MeshManager::getSingleton().createPlane(
        "wall1", RGN_DEFAULT,
        wallPlane,
        1500, 15000, 20, 20,
        true,
        1, 5, 5,
        Vector3::UNIT_Z);

    // Create an entity for the wall
    Entity* wallEntity = scnMgr->createEntity("wall1");

    // Setup ground entity
    //  Shadows off
    wallEntity->setCastShadows(false);

    // Material - Examples is the resources file,
    // Rockwall (texture/properties) is defined inside it.
    wallEntity->setMaterialName("Examples/Rockwall");

    // Create a scene node to add the mesh too.
    SceneNode* thisSceneNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    thisSceneNode->attachObject(wallEntity);

    // the wall is a cube of side 100 at position y = 0.
    // the sphere will hit it at y = -6, with center at -5
    btCollisionShape* wallShape = new btBoxShape(btVector3(btScalar(100.), btScalar(15000.), btScalar(15000.)));

    collisionShapes.push_back(wallShape);

    btTransform wallTransform;
    wallTransform.setIdentity();

    Vector3 wallPos = thisSceneNode->_getDerivedPosition();

    // Box is 100 deep (dimensions are 1/2 heights)
    // but the plane position is flat.
    wallTransform.setOrigin(btVector3(wallPos.x - 850.0, wallPos.y, wallPos.z));

    Quaternion quat2 = thisSceneNode->_getDerivedOrientation();
    wallTransform.setRotation(btQuaternion(quat2.x, quat2.y, quat2.z, quat2.w));

    btScalar mass(0.);

    // rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
        wallShape->calculateLocalInertia(mass, localInertia);

    // using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* myMotionState = new btDefaultMotionState(wallTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, wallShape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);

    //   body->setRestitution(0.0);

    // add the body to the dynamics world
    dynamicsWorld->addRigidBody(body);
}
void Game::setupWall2()
{
    // Create a plane
    Plane wallPlane(Vector3::NEGATIVE_UNIT_X, -750);

    // Define the plane mesh
    MeshManager::getSingleton().createPlane(
        "wall2", RGN_DEFAULT,
        wallPlane,
        1500, 15000, 20, 20,
        true,
        1, 5, 5,
        Vector3::UNIT_Z);

    // Create an entity for the wall
    Entity* wallEntity = scnMgr->createEntity("wall2");

    // Setup ground entity
    //  Shadows off
    wallEntity->setCastShadows(false);

    // Material - Examples is the resources file,
    // Rockwall (texture/properties) is defined inside it.
    wallEntity->setMaterialName("Examples/Rockwall");

    // Create a scene node to add the mesh too.
    SceneNode* thisSceneNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    thisSceneNode->attachObject(wallEntity);

    // the wall is a cube of side 100 at position y = 0.
    // the sphere will hit it at y = -6, with center at -5
    btCollisionShape* wallShape = new btBoxShape(btVector3(btVector3(btScalar(100.0), btScalar(15000.0), btScalar(15000.0))));

    collisionShapes.push_back(wallShape);

    btTransform wallTransform;
    wallTransform.setIdentity();

    Vector3 wallPos = thisSceneNode->_getDerivedPosition();

    // Box is 100 deep (dimensions are 1/2 heights)
    // but the plane position is flat.
    wallTransform.setOrigin(btVector3(wallPos.x + 850, wallPos.y, wallPos.z));

    Quaternion quat2 = thisSceneNode->_getDerivedOrientation();
    wallTransform.setRotation(btQuaternion(quat2.x, quat2.y, quat2.z, quat2.w));

    btScalar mass(0.);

    // rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
        wallShape->calculateLocalInertia(mass, localInertia);

    // using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* myMotionState = new btDefaultMotionState(wallTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, wallShape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);

    //   body->setRestitution(0.0);

    // add the body to the dynamics world
    dynamicsWorld->addRigidBody(body);
}
void Game::setupWall3()
{
    // Create a plane
    Plane wallPlane(Vector3::UNIT_Z, -7500);

    // Define the plane mesh
    MeshManager::getSingleton().createPlane(
        "wall3", RGN_DEFAULT,
        wallPlane,
        1500, 15000, 20, 20,
        true,
        1, 5, 5,
        Vector3::UNIT_X);

    // Create an entity for the wall
    Entity* wallEntity = scnMgr->createEntity("wall3");

    // Setup ground entity
    //  Shadows off
    wallEntity->setCastShadows(false);

    // Material - Examples is the resources file,
    // Rockwall (texture/properties) is defined inside it.
    wallEntity->setMaterialName("Examples/Rockwall");

    // Create a scene node to add the mesh too.
    SceneNode* thisSceneNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    thisSceneNode->attachObject(wallEntity);

    // the wall is a cube of side 100 at position y = 0.
    // the sphere will hit it at y = -6, with center at -5
    btCollisionShape* wallShape = new btBoxShape(btVector3(btVector3(btScalar(15000.), btScalar(15000.), btScalar(100.))));

    collisionShapes.push_back(wallShape);

    btTransform wallTransform;
    wallTransform.setIdentity();

    Vector3 wallPos = thisSceneNode->_getDerivedPosition();

    // Box is 100 deep (dimensions are 1/2 heights)
    // but the plane position is flat.
    wallTransform.setOrigin(btVector3(wallPos.x, wallPos.y, wallPos.z - 7600));

    Quaternion quat2 = thisSceneNode->_getDerivedOrientation();
    wallTransform.setRotation(btQuaternion(quat2.x, quat2.y, quat2.z, quat2.w));

    btScalar mass(0.);

    // rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
        wallShape->calculateLocalInertia(mass, localInertia);

    // using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* myMotionState = new btDefaultMotionState(wallTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, wallShape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);

    //   body->setRestitution(0.0);

    // add the body to the dynamics world
    dynamicsWorld->addRigidBody(body);
}
void Game::setupWall4()
{
    // Create a plane
    Plane wallPlane(Vector3::NEGATIVE_UNIT_Z, -7500);

    // Define the plane mesh
    MeshManager::getSingleton().createPlane(
        "wall4", RGN_DEFAULT,
        wallPlane,
        1500, 15000, 20, 20,
        true,
        1, 5, 5,
        Vector3::UNIT_X);

    // Create an entity for the wall
    Entity* wallEntity = scnMgr->createEntity("wall4");

    // Setup ground entity
    //  Shadows off
    wallEntity->setCastShadows(false);

    // Material - Examples is the resources file,
    // Rockwall (texture/properties) is defined inside it.
    wallEntity->setMaterialName("Examples/Rockwall");

    // Create a scene node to add the mesh too.
    SceneNode* thisSceneNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    thisSceneNode->attachObject(wallEntity);

    // the wall is a cube of side 100 at position y = 0.
    // the sphere will hit it at y = -6, with center at -5
    btCollisionShape* wallShape = new btBoxShape(btVector3((15000.0), btScalar(15000.0), btScalar(100.0)));

    collisionShapes.push_back(wallShape);

    btTransform wallTransform;
    wallTransform.setIdentity();

    Vector3 wallPos = thisSceneNode->_getDerivedPosition();

    // Box is 100 deep (dimensions are 1/2 heights)
    // but the plane position is flat.
    wallTransform.setOrigin(btVector3(wallPos.x, wallPos.y, wallPos.z + 7600));

    Quaternion quat2 = thisSceneNode->_getDerivedOrientation();
    wallTransform.setRotation(btQuaternion(quat2.x, quat2.y, quat2.z, quat2.w));

    btScalar mass(0.);

    // rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
        wallShape->calculateLocalInertia(mass, localInertia);

    // using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* myMotionState = new btDefaultMotionState(wallTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, wallShape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);

    //   body->setRestitution(0.0);

    // add the body to the dynamics world
    dynamicsWorld->addRigidBody(body);
}

// game loop
bool Game::frameStarted(const Ogre::FrameEvent &evt)
{
    // Be sure to call base class - otherwise events are not polled.
    ApplicationContext::frameStarted(evt);

    if (this->dynamicsWorld != NULL)
    {
        // check if escape is pressed        
        if (inputHelper->esc)
        {
            getRoot()->queueEndRendering();
        }

        // loop through game objects and handleInput()
        for (int i = 0; i < gameObjects.size(); i++)
        {
            gameObjects[i].handleInput(*inputHelper);
        }

        // Bullet can work with a fixed timestep
        // dynamicsWorld->stepSimulation(1.f / 60.f, 10);

        // Or a variable one, however, under the hood it uses a fixed timestep
        // then interpolates between them.

        // Apply forces based on input.
        // update player positions

        dynamicsWorld->stepSimulation((float)evt.timeSinceLastFrame, 10);

        // update positions of all objects
        for (int j = dynamicsWorld->getNumCollisionObjects() - 1; j >= 0; j--)
        {
            btCollisionObject *obj = dynamicsWorld->getCollisionObjectArray()[j];
            btRigidBody *body = btRigidBody::upcast(obj);
            btTransform trans;

            if (body && body->getMotionState())
            {
                body->getMotionState()->getWorldTransform(trans);

                // Bullet has updated the rididbody, we now need to update the ogre scene node (i.e. the model on screen).
                void *userPointer = body->getUserPointer();

                // This is a horrific hack!!!!!!
                // Need to change this so everything in the game (including the floor) uses 
                // the same method of updating its physics / graphics. 
                if (userPointer && userPointer != player && userPointer != npc)
                {
                    /*btQuaternion orientation = trans.getRotation();
                    Ogre::SceneNode *sceneNode = static_cast<Ogre::SceneNode *>(userPointer);
                    sceneNode->setPosition(Ogre::Vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()));
                    Ogre::Quaternion q = Ogre::Quaternion(orientation.getW(), orientation.getX(), orientation.getY(), orientation.getZ());
                    sceneNode->setOrientation(Ogre::Quaternion(orientation.getW(), orientation.getX(), orientation.getY(), orientation.getZ()));*/
                }
                else
                {
                     //std::cout << "Player update" << std::endl;
                     player->update();

                     // AI here
                     npc->arrive();
                }
            }
            else
            {
                trans = obj->getWorldTransform();
            }
        }

        // always update the npc, it has no player input to wake it back up!
        npc->setTarget(player->getRigidBody()->getWorldTransform());

        npc->update();
    }

    return true;
}

bool Game::frameEnded(const Ogre::FrameEvent &evt)
{
    if (this->dynamicsWorld != NULL)
    {
        // Bullet can work with a fixed timestep
        // dynamicsWorld->stepSimulation(1.f / 60.f, 10);

        // Or a variable one, however, under the hood it uses a fixed timestep
        // then interpolates between them.

        dynamicsWorld->stepSimulation((float)evt.timeSinceLastFrame, 10);
    }
    return true;
}

