#pragma once
#include "GraphicsComponent.h"
#include "InputHelper.h"
#include "btBulletDynamicsCommon.h"

class InputComponent;
class PlayerInputComponent;
class PhysicsComponent;
class GraphicsComponent;

class GameObject
{
private:
    GraphicsComponent* graphicsComponent;
	InputComponent* inputComponent;
	
	SceneNode* boxSceneNode;     /**< Scene graph node */
	Entity* box;                 /**< Mesh entity */

	/**
	* Keep track of the way points for the npc
	*/
	std::vector<btTransform> m_patrolWayPoints;
	int currentWaypoint;

	/**
	* The dynamics world (physics/collision world). Need this to raycast to the floor for jumping.
	*/
	btDiscreteDynamicsWorld* dynamicsWorld;

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
	PhysicsComponent* physicsComponent;
	btCollisionShape* colShape;  /**< Collision shape, describes the collision boundary */

	btTransform target;          /**< Making use of the transform here, might want to use the angle to ensure a particular orientation on arrival.  */

    /**
    * Creates the object, sets all pointers to nullptr.
    */
    GameObject();

    /**
    * Destructor (virtual), as this is virtual that of the sub class will also be called.
    */
    virtual ~GameObject();

	// setters
	void setGraphicsComponent(GraphicsComponent* gc);
	void setPhysicsComponent(PhysicsComponent* pc);
	void setInputComponent(InputComponent* ic);

	void handleInput(InputHelper inputHelper);

	/**
	* Update, synchronise rigid body and scene node (which holds mesh)
	*/
	void update();

	/** Setup the object
	* @param mass
	*/
	void setup(SceneManager* scnMgr, btDiscreteDynamicsWorld* world, float mass);

	void AddWayPoint(btVector3 newPoint);

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

	SceneNode* getPlayerNode();

	/**
	* @brief Arrive behaviour, heads towards a target held in the class.
	*
	*/
	void arrive();

	/**
	* @brief Check target is in range, allows re-use for slowing radius.
	* @param range distance to target.
	*
	*/
	bool targetInRange(float range);

	/**
	* @brief Target for arrive.
	*
	*/
	void setTarget(btTransform target);

	/**
	* @brief support method for arrive, look where you are going.
	* @return true if we're looking the right way, false otherwise (keep turning).
	*/
	bool lookWhereYouAreGoing();

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

	/**
	* Make sure the mesh is lined up with the ghost collision object.
	*
	*/
	void syncSceneNode();
};

