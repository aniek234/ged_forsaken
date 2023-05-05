#include "GameObject.h"
#include <iostream>
#include <OgreBullet.h>
#include "InputComponent.h"
#include "PlayerInputComponent.h"
#include "PhysicsComponent.h"
#include "GraphicsComponent.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <math.h>

GameObject::GameObject()
{
    graphicsComponent = nullptr;
    physicsComponent = nullptr;
    inputComponent = nullptr;

    AddWayPoint(btVector3(200.0f, 20.0f, -50.0f));
}

GameObject::~GameObject(){}

// set methods
void GameObject::setGraphicsComponent(GraphicsComponent* gc) { this->graphicsComponent = gc; }

void GameObject::setPhysicsComponent(PhysicsComponent* pc)
{ 
    if (!this->colShape)
    {
        // throw error
        throw std::runtime_error(std::string("Failed to set Physics Component because there is no Collision shape"));
    }
    else 
    {
        this->physicsComponent = pc;
        // TODO: heeft c++ exit function? program termination
    }
}

void GameObject::setInputComponent(InputComponent* ic) { this->inputComponent = ic; }

void GameObject::handleInput(InputHelper inputHelper)
{
    if (inputComponent)
    {
        inputComponent->handleInput(inputHelper, this);
    }
}


/* Note: this attaches to the root, probably a bit limiting? */
void GameObject::setup(SceneManager* scnMgr, btDiscreteDynamicsWorld* world, float mass)
{
    dynamicsWorld = world;

    createMesh(scnMgr);
    attachToNode(scnMgr->getRootSceneNode());

    createCollisionShape();
}

void GameObject::AddWayPoint(btVector3 newPoint)
{
    btTransform localTrans;
    localTrans.setOrigin(newPoint);
    m_patrolWayPoints.push_back(localTrans);
    setTarget(localTrans);
}

void GameObject::createMesh(SceneManager* scnMgr)
{
    box = scnMgr->createEntity("cube.mesh");
}

void GameObject::attachToNode(SceneNode* parent)
{
    boxSceneNode = parent->createChildSceneNode();
    boxSceneNode->attachObject(box);
    boxSceneNode->setScale(0.5f, 0.5f, 0.5f);
}

void GameObject::setScale(float x, float y, float z)
{
    boxSceneNode->setScale(x, y, z);
}

/* Not as simple as creating the body in the right place. This is
   based on this stack overflow post https://stackoverflow.com/questions/12251199/re-positioning-a-rigid-body-in-bullet-physics
*/
void GameObject::setRotation(Vector3 axis, Radian rads)
{
    btRigidBody* rb = physicsComponent->rigidBody;

    //quat from axis angle
    Quaternion quat(rads, axis);
    //boxSceneNode->setOrientation(quat);

    btTransform trans;

    // get current transformation
    rb->getMotionState()->getWorldTransform(trans);

    btQuaternion rotation = Ogre::Bullet::convert(quat);

    // replace rotation
    trans.setRotation(rotation);

    // put it back
    rb->setWorldTransform(trans);
    rb->getMotionState()->setWorldTransform(trans);

    // sync scene node
    syncSceneNode();
}

void GameObject::createCollisionShape()
{
    colShape = Ogre::Bullet::createBoxCollider(box);
}

btCollisionShape* GameObject::getCollisionShape()
{
    return colShape;
}

btRigidBody* GameObject::getRigidBody()
{
    return physicsComponent->rigidBody;
}

void GameObject::update()
{
    if (physicsComponent)
    {
        syncSceneNode();
    }
}

SceneNode* GameObject::getPlayerNode()
{
    return boxSceneNode;
}

void GameObject::syncSceneNode()
{
    btTransform trans;
    physicsComponent->getWorldTransform(trans);
    btQuaternion orientation = trans.getRotation();

    boxSceneNode->setPosition(Ogre::Vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()));
    boxSceneNode->setOrientation(Ogre::Quaternion(orientation.getW(), orientation.getX(), orientation.getY(), orientation.getZ()));
}


void GameObject::arrive()
{
    // Slowing radius
    float slowingRad = 500.0f;
    // Stopping radius
    float stoppingRad = 105.0f; // note cube is 100 square, don't set this below 100!

    if (targetInRange(stoppingRad))
    {
        currentWaypoint++;
        if (currentWaypoint >= m_patrolWayPoints.size())
        {
            //std::cout << "currentWayPoint: " + currentWaypoint;
            currentWaypoint = 0;
        }
        setTarget(m_patrolWayPoints.at(currentWaypoint));
        // stop the npc!
    }
    else
    {
        if (lookWhereYouAreGoing())
        {
            // point at the player and ram, could make a fun sumo game out of this!
            //forward();
            physicsComponent->forward();

            // Aim to slow to a stop at the position
            if (targetInRange(slowingRad))
            {
                // slow to target
            }
        }
    }
}

void GameObject::setTarget(btTransform target)
{
    this->target = target;
}

bool GameObject::targetInRange(float range)
{
    btTransform trans;
    float range2 = range * range; // faster than working out the root of the lenght?

    // Get current transform
    physicsComponent->getWorldTransform(trans);

    btVector3 vecToTarget = target.getOrigin() - trans.getOrigin();

    std::cout << "Distance to target " << Math::Sqrt(vecToTarget.length2()) << std::endl;

    if (vecToTarget.length2() < range2)
        return true;
    else
        return false;
}

/*
This could be tricky, I'm going to cheat and apply a turning force based
on a vector to the player.
*/
bool GameObject::lookWhereYouAreGoing()
{
    // Current Orientation
    btTransform trans;
    physicsComponent->getWorldTransform(trans);
    btQuaternion orientation = trans.getRotation();

    // Desired force
    btVector3 forward(0.0f, 0.0f, 1.0f);

    //rotate the local forward, into the global space.
    //i.e. push in down the local z.
    forward = quatRotate(orientation, forward);
    forward.normalize();

    btVector3 vecToTarget = target.getOrigin() - trans.getOrigin();
    vecToTarget.normalize();

    float dotProd = btDot(forward, vecToTarget); // dotprod
    float angle = acos(dotProd);

    btVector3 crossProd = btCross(forward, vecToTarget); // cross prod

    std::cout << "Angle is:" << angle << std::endl;
    std::cout << "cross is" << "[" << crossProd.x() << "," << crossProd.y() << "," << crossProd.z() << "]" << std::endl;

    if (angle < 0.3f) // dead ahead! - or close enough
        return true;
    else
    {
        if (crossProd.y() > 0.0f)
        {
            std::cout << "Turning Right" << std::endl;
            physicsComponent->turnRightScaled(angle / M_PI);
        }
        else
        {
            std::cout << "Turning Left" << std::endl;
            physicsComponent->turnLeftScaled(angle / M_PI);
        }
    }

    return false;
}
