#pragma once

/* Bullet3 Physics */
#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"
#include "GameObject.h"

class PhysicsComponent
{
private:
    /**
    * The dynamics world (physics/collision world).
    */
    btDiscreteDynamicsWorld* dynamicsWorld;

    /**
    * Keep track of the shapes, we release memory at exit.
    * make sure to re-use collision shapes among rigid bodies whenever possible!
    */
    btAlignedObjectArray<btCollisionShape*> collisionShapes;

    // the game object with this component
    GameObject* gameObject;

    float forwardForce;
    float turningForce;
    float jumpForce;
    float linearDamping;
    float angularDamping;

public:
    btRigidBody* rigidBody;

    /**
    * Creates the object, sets all pointers to nullptr.
    */
    PhysicsComponent(btDiscreteDynamicsWorld* world, float bodyMass, btCollisionShape* colShape, float forwardForce = 5.0f, float turningForce = 25.0f, float jumpForce = 2.0f, float linearDamping = 0.6f,
        float angularDamping = 0.8f);

    /**
    * Destructor (virtual), as this is virtual that of the sub class will also be called.
    */
    virtual ~PhysicsComponent();

    /**
    * Creates a new rigid body of the given mass.
    * @param mass
    */
    void createRigidBody(float mass, btCollisionShape* colShape);

    /**
    * Sets the position.
    * @param x, position on the x axis.
    * @param y, position on the y axis.
    * @param z, position on the z axis.
    */
    void setPosition(float x, float y, float z);

    void getWorldTransform(btTransform transform);

    /**
    * Moves the player forward with maximum acceleration.
    */
    void forward();

    /**
    * Moves the player backward with maximum acceleration.
    */
    void backward();

    /**
    * Just apply a turning force.
    */
    void turnRight();

    /**
    * Just apply a turning force.
    */
    void turnLeft();

    void turnLeftScaled(float scalar);

    void turnRightScaled(float scalar);

    /**
    *  Jump
    */
    void jump();

    /**
    * Fly
    */
    void fly();


    /**
    * Testing out 6DOF to restrict rotation axis.
    *
    */
    void restrictAxisTest();

    /**
    * Check to see the player is on the floor (to prevent jumping on the ground)
    * Uses the code from the earlier ray casting example.
    *
    */
    bool isGrounded(btRigidBody* body, float linearDamping, float angularDamping);
};

