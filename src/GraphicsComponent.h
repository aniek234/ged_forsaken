#pragma once

#include "Ogre.h"

using namespace Ogre;

class GraphicsComponent
{
private:
    // the game object with this component
    //GameObject* gameObject;

    SceneNode* boxSceneNode;     /**< Scene graph node */
    Entity* box;                 /**< Mesh entity */

public:

    /**
    * Creates the object, sets all pointers to nullptr.
    */
    GraphicsComponent();

    /**
    * Destructor (virtual), as this is virtual that of the sub class will also be called.
    */
    virtual ~GraphicsComponent();

    /**
    * Creates the mesh.
    * @param scnMgr the Ogre SceneManager.
    */
    void createMesh(SceneManager* scnMgr);

};

