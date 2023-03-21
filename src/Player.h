#ifndef PLAYER_H_
#define PLAYER_H_

/* Ogre3d Graphics*/
#include "Ogre.h"

/* Bullet3 Physics */
#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"

using namespace Ogre;

/** @brief More sophisticated player class. 
*/
class Player
{
private:
  SceneNode* boxSceneNode;     /**< Scene graph node */
  Entity* box;                 /**< Mesh entity */

  btCollisionShape* colShape;  /**< Collision shape, describes the collision boundary */
  btRigidBody* body;           /**< Rigid Body */

  /**
   * The dynamics world (physics/collision world). Need this to raycast to the floor for jumping. 
   */
    btDiscreteDynamicsWorld* dynamicsWorld;


  float forwardForce; /**< Force of the engine/thrusters moving the player forward */
  float turningForce; /**< Force exerted by the turning of the wheels or thrusters */
  float jumpForce;  /**< Force exerted by jumping or upward thrusters */
  btScalar linearDamping; /**< Damping force on the linear motion of the body, kind of air/friction */
  btScalar angularDamping; /**< Damping force on the angular motion of the body, kind of air/friction */

 
   /**
  * Creates a new rigid body of the given mass.
  * @param mass
  */
  void createRigidBody(float mass);

  /**
  * Creates the mesh.
  * @param scnMgr the Ogre SceneManager.
  */
  void createMesh(SceneManager* scnMgr);
  /**
  * Creates a new child of the given parent node, adds the mesh to it.
  * @param parent, the parent (in the scene graph) of the node the player will be attached to.
  */
  void attachToNode(SceneNode* parent);

public:
  Player();
  ~Player();

  /** Setup the object 
   * @param mass
   */
  void setup(SceneManager* scnMgr, btDiscreteDynamicsWorld* world, float mass);

  /**
  * Sets the scale.
  * @param x, scale on the x axis.
  * @param y, scale on the y axis.
  * @param z, scale on the z axis.
  * 
  * Note: This has not been fully implemented, need to replace the 
  * collision mesh on resize. 
  * 
  */
  void setScale(float x, float y, float z);
  /**
  * Sets the orientation.
  * @param axis, vector about which the orientation takes place.
  * @param angle, angle (in radians).
  */
  void setRotation(Vector3 axis, Radian angle);

  /**
  * Sets the position.
  * @param x, position on the x axis.
  * @param y, position on the y axis.
  * @param z, position on the z axis.
  */  
  void setPosition(float x, float y, float z);

  /**
  * Update, synchronise rigid body and scene node (which holds mesh)
  */
  void update();

  /**
   * @brief Create the collision shape, note this uses the Ogre::Bullet plugin.
   * 
   */
  void createCollisionShape();

  /**
   * @brief Getter for the collision shape
   * @retval btCollisionShape for this object. 
   * 
   */
  btCollisionShape* getCollisionShape();

  /**
   * @brief Getter for the rigidbody
   * @retval btRigidBody for this object. 
   * 
   */
  btRigidBody* getRigidBody();

  // Player Movement methods!!

  /**
  * Moves the player forward with maximum acceleration.
  */
  void forward();

  /**
  * Just apply a turning force.
  */
  void turnRight();

  /**
  * Just apply a turning force.
  */
  void turnLeft();

  /**
   * Check to see the player is on the floor (to prevent jumping on the ground)
   * Uses the code from the earlier ray casting example. 
   * 
   */
  bool isGrounded();

  /**
   *  Jump
   * 
   */
  void jump();

  /**
   * Fly
   * 
   */
  void fly();

  /**
   * Testing out 6DOF to restrict rotation axis.  
   * 
   */
  void restrictAxisTest();


  /**
  * Make sure the mesh is lined up with the ghost collision object. 
  * 
  */
  void syncSceneNode();

};


#endif
