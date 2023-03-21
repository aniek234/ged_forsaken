#include "Player.h"

#include "OgreBullet.h"
#include <iostream>

Player::Player()
{
  boxSceneNode = nullptr;
  box = nullptr;
  body = nullptr;
  colShape = nullptr;

  /* Note: These are hardcoded in player.  Should probably be read in from a
  * config file or similar.
  */

  forwardForce = 5.0f;
  turningForce = 50.0f;
  jumpForce = 5.0f;

  linearDamping = 0.6f;
  angularDamping = 0.1f;
}

Player::~Player()
{

}

/* Note: this attaches to the root, probably a bit limiting? */
void Player::setup(SceneManager* scnMgr, btDiscreteDynamicsWorld* world, float mass)
{
    dynamicsWorld = world;
    
    createMesh(scnMgr);
    attachToNode(scnMgr->getRootSceneNode());

    createCollisionShape();
    createRigidBody(mass);
}

void Player::createMesh(SceneManager* scnMgr)
{
  box = scnMgr->createEntity("cube.mesh");
}

void Player::attachToNode(SceneNode* parent)
{
  boxSceneNode = parent->createChildSceneNode();
  boxSceneNode->attachObject(box);
  boxSceneNode->setScale(1.0f,1.0f,1.0f);

}

void Player::setScale(float x, float y, float z)
{
    boxSceneNode->setScale(x,y,z);
}

/* Not as simple as creating the body in the right place. This is 
   based on this stack overflow post https://stackoverflow.com/questions/12251199/re-positioning-a-rigid-body-in-bullet-physics 
*/
void Player::setRotation(Vector3 axis, Radian rads)
{
  //quat from axis angle
  Quaternion quat(rads, axis);
  //boxSceneNode->setOrientation(quat);

  btTransform trans;
  
  // get current transformation
  body->getMotionState()->getWorldTransform(trans);

  btQuaternion rotation = Ogre::Bullet::convert(quat);

  // replace rotation
  trans.setRotation(rotation);

  // put it back
  body->setWorldTransform(trans);
  body->getMotionState()->setWorldTransform(trans);

  // sync scene node
  syncSceneNode();
}

/* Not as simple as creating the body in the right place. This is 
   based on this stack overflow post https://stackoverflow.com/questions/12251199/re-positioning-a-rigid-body-in-bullet-physics 
*/
void Player::setPosition(float x, float y, float z)
{
    // create a new position from the x,y,z
    btVector3 pos(x,y,z);

    btTransform trans;
    
    // get current transformation
    body->getMotionState()->getWorldTransform(trans);

    // replace rotation
    trans.setOrigin(pos);

    // put it back
    body->setWorldTransform(trans);
    body->getMotionState()->setWorldTransform(trans);

    // sync scene node
    syncSceneNode();
}

void Player::createCollisionShape() 
{
  colShape = Ogre::Bullet::createBoxCollider(box);
}

void Player::createRigidBody(float bodyMass)
{
  /// Create Dynamic Objects
  btTransform startTransform;
  startTransform.setIdentity();

  btScalar mass(bodyMass);

  //rigidbody is dynamic if and only if mass is non zero, otherwise static
  bool isDynamic = (mass != 0.f);

  btVector3 localInertia(0, 0, 0);
  if (isDynamic)
  {
      // Debugging
      //std::cout << "I see the cube is dynamic" << std::endl;
      colShape->calculateLocalInertia(mass, localInertia);
  }

  //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
  btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
  btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
  body = new btRigidBody(rbInfo);

  // aid the control of this body by adding linear and angular drag!
  // If we wanted to have different drag / damping for each dimension axis, 
  // we need to implement this ourselves - https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=11430
  // Constraints are probably easier!
  body->setDamping(linearDamping,angularDamping);

  // Just some testing
  restrictAxisTest();

  //Set the user pointer to this object.
  body->setUserPointer((void*)this);
}

void Player::restrictAxisTest() 
{
    static btRigidBody staticBody(0,NULL,NULL);
    staticBody.setWorldTransform(btTransform::getIdentity());//No need to add it to the world
    btTransform localA,localB;
    localA.setIdentity(); localB.setIdentity();
    // Now you should set their basis so that your plane normal ends on the X-axis (or on the Z-axis).

    // The following should work if your plane normal is the Y axis (and we move it to the X axis): change it accordingly:
    btQuaternion q(btVector3(0,0,1),SIMD_HALF_PI);	// If we need a full 360 rotation around Y, Y is limited in [-PI/2,PI/2] AFAIK, so we swap Y with X ( newY = X; newX = -Y )

    localA.getBasis().setRotation(q);
    localB.getBasis()=localA.getBasis();
                
    btGeneric6DofConstraint *joint6DOF =  new btGeneric6DofConstraint (staticBody, *body, localA, localB, true);

    // Now the limits depend on what you want to do (convention from docks):
    // Lowerlimit == Upperlimit -> axis is locked.
    // Lowerlimit > Upperlimit -> axis is free
    // Lowerlimit < Upperlimit -> axis it limited in that range 

    // Free translation allowed in this case:
    joint6DOF->setLinearLowerLimit(btVector3(1,1,1));	
    joint6DOF->setLinearUpperLimit(btVector3(-1,-1,-1));

    // This will force the body to be aligned with the Y axis (or your plane normal axis) like setAngularFactor(0,1,0):
    joint6DOF->setAngularLowerLimit(btVector3(1,0,0));	// Axis 0 = Y axis now
    joint6DOF->setAngularUpperLimit(btVector3(0,0,0));

    dynamicsWorld->addConstraint(joint6DOF,true);
}

btCollisionShape* Player::getCollisionShape() 
{ 
  return colShape;
}

btRigidBody* Player::getRigidBody()
{
  return body;
}

void Player::update()
{
  if (body && body->getMotionState())
  {
    syncSceneNode();

  }

}

void Player::syncSceneNode() 
{
  btTransform trans;
  body->getMotionState()->getWorldTransform(trans);
  btQuaternion orientation = trans.getRotation();

  boxSceneNode->setPosition(Ogre::Vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()));
  boxSceneNode->setOrientation(Ogre::Quaternion(orientation.getW(), orientation.getX(), orientation.getY(), orientation.getZ()));
}

void Player::forward()
{
    //Create a vector in local coordinates
    //pointing down z.
    btVector3 fwd(0.0f,0.0f,forwardForce);
    btVector3 push;

    btTransform trans;

    if (body && body->getMotionState())
    {
        //get the orientation of the rigid body in world space.
        body->getMotionState()->getWorldTransform(trans);
        btQuaternion orientation = trans.getRotation();

        //rotate the local force, into the global space.
        //i.e. push in down the local z.
        push = quatRotate(orientation, fwd);

        //activate the body, this is essential if the body
        //has gone to sleep (i.e. stopped moving/colliding).
        body->activate();

        //apply a force to the center of the body
        body->applyCentralImpulse(push);
    }
}

void Player::turnRight()
{
    //Apply a turning force to the front of the body.
    btVector3 right(0.0f,turningForce,0.0f);
    btVector3 turn;

    btTransform trans;

    if (body && body->getMotionState())
    {
        //better - only turn if we're moving.
        //not ideal, if sliding sideways will keep turning.
        //if(body->getLinearVelocity().length() > 0.0f)
            body->applyTorqueImpulse(right);
    }
}

void Player::turnLeft()
{
    //Apply a turning force to the front of the body.
    btVector3 left(0.0f,-turningForce,0.0f);
    btVector3 turn;

    btTransform trans;

    if (body && body->getMotionState())
    {
        //better - only turn if we're moving.
        //not ideal, if sliding sideways will keep turning.
       // if(body->getLinearVelocity().length() > 0.0f)
            body->applyTorqueImpulse(left);
    }
}

/* Based on the code from the earlier ray casting example */
bool Player::isGrounded() 
{
    btTransform trans;
    body->getMotionState()->getWorldTransform(trans);

    // Cast a vector down - 100 from the center down 50 units below the object.
    // took into account the bottom of the object.
    // The coordinates are in world space.
    btVector3 start(trans.getOrigin());
    btVector3 end(start.x(), start.y() - 100, start.z());

    btCollisionWorld::ClosestRayResultCallback closesRayCallback(start, end);
    dynamicsWorld->rayTest(start, end, closesRayCallback);

    if (closesRayCallback.hasHit())
    {
        // Reset linear damping after fall. 
        body->setDamping(linearDamping,angularDamping);

        return true;
    }
    else
    {
       return false;
    }
}

void Player::jump()
{
    //Create a vector in local coordinates
    //pointing up.
    btVector3 up(0.0f,jumpForce,0.0f);
    btVector3 push;

    btTransform trans;

    if (body && body->getMotionState() && isGrounded())
    {
        // Turn off linear damping or we float back down.
        body->setDamping(0.0f,angularDamping);


        //activate the body, this is essential if the body
        //has gone to sleep (i.e. stopped moving/colliding).
        body->activate();

        //apply a force to the center of the body
        body->applyCentralImpulse(up);
    }
}

void Player::fly()
{
    //Create a vector in local coordinates
    //pointing up.
    btVector3 up(0.0f,jumpForce,0.0f);
    btVector3 push;

    btTransform trans;

    if (body && body->getMotionState())
    {

        //activate the body, this is essential if the body
        //has gone to sleep (i.e. stopped moving/colliding).
        body->activate();

        //apply a force to the center of the body
        body->applyCentralImpulse(up);
    }
}
