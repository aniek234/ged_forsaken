#pragma once

#include "OgreInput.h"
#include "OgreInput.h"

using namespace Ogre;
using namespace OgreBites;

class InputHelper : public InputListener
{
public:
    /**
    * Creates the object, sets all pointers to nullptr.
    */
    InputHelper();

    /**
    * Destructor (virtual), as this is virtual that of the sub class will also be called.
    */
    virtual ~InputHelper();
    /**
     * Key States
     */
    bool aDown, wDown, fDown, sDown, jDown, dDown, esc;

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
};

