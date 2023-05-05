#include "PhysicsComponent.h"
#include "GameObject.h"


PhysicsComponent::PhysicsComponent(btDiscreteDynamicsWorld* world, float bodyMass, btCollisionShape* colShape, 
    float forwardForce_, float turningForce_, float jumpForce_, float linearDamping_, 
    float angularDamping_)
{
	// Set all the Bullet stuff to nullptr - trap uninitialised pointer errors.
	/*collisionConfiguration = nullptr;
	dispatcher = nullptr;
	overlappingPairCache = nullptr;
	solver = nullptr;*/
	dynamicsWorld = world;

    createRigidBody(bodyMass, colShape);

    forwardForce = forwardForce_;
    turningForce = turningForce_;
    jumpForce = jumpForce_;
    linearDamping = linearDamping_;
    angularDamping = angularDamping_;
}

PhysicsComponent::~PhysicsComponent()
{

}


void PhysicsComponent::createRigidBody(float bodyMass, btCollisionShape* colShape)
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
    rigidBody = new btRigidBody(rbInfo);

    // aid the control of this body by adding linear and angular drag!
    // If we wanted to have different drag / damping for each dimension axis, 
    // we need to implement this ourselves - https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=11430
    // Constraints are probably easier!
    // TODO!!
    rigidBody->setDamping(linearDamping, angularDamping);

    // Just some testing
    restrictAxisTest();

    //Set the user pointer to this object.
    rigidBody->setUserPointer((void*)this);
}

/* Not as simple as creating the body in the right place. This is
   based on this stack overflow post https://stackoverflow.com/questions/12251199/re-positioning-a-rigid-body-in-bullet-physics
*/
void PhysicsComponent::setPosition(float x, float y, float z)
{
    // create a new position from the x,y,z
    btVector3 pos(x, y, z);

    btTransform trans;

    // get current transformation
    rigidBody->getMotionState()->getWorldTransform(trans);

    // replace rotation
    trans.setOrigin(pos);

    // put it back
    rigidBody->setWorldTransform(trans);
    rigidBody->getMotionState()->setWorldTransform(trans);
}

void PhysicsComponent::getWorldTransform(btTransform trans)
{
    rigidBody->getMotionState()->getWorldTransform(trans);
}


void PhysicsComponent::forward()
{
    //std::cout << "forward() is called ";
    //Create a vector in local coordinates
    //pointing down z.
    btVector3 fwd(0.0f, 0.0f, -forwardForce);
    btVector3 push;

    btTransform trans;

    // TODO: rigidbody check is overbodig
    if (rigidBody && rigidBody->getMotionState())
    {
        //get the orientation of the rigid body in world space.
        rigidBody->getMotionState()->getWorldTransform(trans);
        btQuaternion orientation = trans.getRotation();

        //rotate the local force, into the global space.
        //i.e. push in down the local z.
        push = quatRotate(orientation, fwd);

        //activate the body, this is essential if the body
        //has gone to sleep (i.e. stopped moving/colliding).
        rigidBody->activate();

        //apply a force to the center of the body
        rigidBody->applyCentralImpulse(push);
    }
}

void PhysicsComponent::backward()
{
    //Create a vector in local coordinates
    //pointing down z.
    btVector3 fwd(0.0f, 0.0f, forwardForce);
    btVector3 push;

    btTransform trans;

    if (rigidBody && rigidBody->getMotionState())
    {
        //get the orientation of the rigid body in world space.
        rigidBody->getMotionState()->getWorldTransform(trans);
        btQuaternion orientation = trans.getRotation();

        //rotate the local force, into the global space.
        //i.e. push in down the local z.
        push = quatRotate(orientation, fwd);

        //activate the body, this is essential if the body
        //has gone to sleep (i.e. stopped moving/colliding).
        rigidBody->activate();

        //apply a force to the center of the body
        rigidBody->applyCentralImpulse(push);
    }
}

void PhysicsComponent::turnRight()
{
    //Apply a turning force to the front of the body.
    btVector3 right(0.0f, -turningForce, 0.0f);
    btVector3 turn;

    btTransform trans;

    if (rigidBody && rigidBody->getMotionState())
    {
        //better - only turn if we're moving.
        //not ideal, if sliding sideways will keep turning.
        if (rigidBody->getLinearVelocity().length() > 0.0f)
            rigidBody->applyTorqueImpulse(right);
    }
}

void PhysicsComponent::turnLeft()
{
    //Apply a turning force to the front of the body.
    btVector3 left(0.0f, turningForce, 0.0f);
    btVector3 turn;

    btTransform trans;

    if (rigidBody && rigidBody->getMotionState())
    {
        //better - only turn if we're moving.
        //not ideal, if sliding sideways will keep turning.
       // if(body->getLinearVelocity().length() > 0.0f)
        rigidBody->applyTorqueImpulse(left);
    }
}

void PhysicsComponent::turnLeftScaled(float scalar)
{
    //Apply a turning force to the front of the body.
    float tf = -turningForce;
    tf *= scalar;
    btVector3 left(0.0f, tf, 0.0f);
    btVector3 turn;

    btTransform trans;

    if (rigidBody && rigidBody->getMotionState())
    {
        //activate the body, this is essential if the body
        //has gone to sleep (i.e. stopped moving/colliding).
        rigidBody->activate();

        //better - only turn if we're moving.
        //not ideal, if sliding sideways will keep turning.
       // if(body->getLinearVelocity().length() > 0.0f)
        rigidBody->applyTorqueImpulse(left);
    }
}

void PhysicsComponent::turnRightScaled(float scalar)
{
    //Apply a turning force to the front of the body.
    float tf = turningForce;
    tf *= scalar;
    btVector3 right(0.0f, tf, 0.0f);
    btVector3 turn;

    btTransform trans;

    if (rigidBody && rigidBody->getMotionState())
    {
        //activate the body, this is essential if the body
        //has gone to sleep (i.e. stopped moving/colliding).
        rigidBody->activate();

        //better - only turn if we're moving.
        //not ideal, if sliding sideways will keep turning.
       // if(body->getLinearVelocity().length() > 0.0f)
        rigidBody->applyTorqueImpulse(right);
    }
}


void PhysicsComponent::jump()
{
    //Create a vector in local coordinates
    //pointing up.
    btVector3 up(0.0f, jumpForce, 0.0f);
    btVector3 push;

    btTransform trans;

    if (rigidBody && rigidBody->getMotionState() && isGrounded(rigidBody, linearDamping, angularDamping))
    {
        // Turn off linear damping or we float back down.
        rigidBody->setDamping(0.0f, angularDamping);


        //activate the body, this is essential if the body
        //has gone to sleep (i.e. stopped moving/colliding).
        rigidBody->activate();

        //apply a force to the center of the body
        rigidBody->applyCentralImpulse(up);
    }
}

void PhysicsComponent::fly()
{
    //Create a vector in local coordinates
    //pointing up.
    btVector3 up(0.0f, jumpForce, 0.0f);
    btVector3 push;

    btTransform trans;

    if (rigidBody && rigidBody->getMotionState())
    {
        //activate the body, this is essential if the body
        //has gone to sleep (i.e. stopped moving/colliding).
        rigidBody->activate();

        //apply a force to the center of the body
        rigidBody->applyCentralImpulse(up);
    }
}

void PhysicsComponent::restrictAxisTest()
{
    static btRigidBody staticBody(0, NULL, NULL);
    staticBody.setWorldTransform(btTransform::getIdentity());//No need to add it to the world
    btTransform localA, localB;
    localA.setIdentity(); localB.setIdentity();
    // Now you should set their basis so that your plane normal ends on the X-axis (or on the Z-axis).

    // The following should work if your plane normal is the Y axis (and we move it to the X axis): change it accordingly:
    btQuaternion q(btVector3(0, 0, 1), SIMD_HALF_PI);	// If we need a full 360 rotation around Y, Y is limited in [-PI/2,PI/2] AFAIK, so we swap Y with X ( newY = X; newX = -Y )

    localA.getBasis().setRotation(q);
    localB.getBasis() = localA.getBasis();

    btGeneric6DofConstraint* joint6DOF = new btGeneric6DofConstraint(staticBody, *rigidBody, localA, localB, true);

    // Now the limits depend on what you want to do (convention from docks):
    // Lowerlimit == Upperlimit -> axis is locked.
    // Lowerlimit > Upperlimit -> axis is free
    // Lowerlimit < Upperlimit -> axis it limited in that range 

    // Free translation allowed in this case:
    joint6DOF->setLinearLowerLimit(btVector3(1, 1, 1));
    joint6DOF->setLinearUpperLimit(btVector3(-1, -1, -1));

    // This will force the body to be aligned with the Y axis (or your plane normal axis) like setAngularFactor(0,1,0):
    joint6DOF->setAngularLowerLimit(btVector3(1, 0, 0));	// Axis 0 = Y axis now
    joint6DOF->setAngularUpperLimit(btVector3(0, 0, 0));

    dynamicsWorld->addConstraint(joint6DOF, true);
}

/* Based on the code from the earlier ray casting example */
bool PhysicsComponent::isGrounded(btRigidBody* body, float linearDamping, float angularDamping)
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
        body->setDamping(linearDamping, angularDamping);

        return true;
    }
    else
    {
        return false;
    }
}