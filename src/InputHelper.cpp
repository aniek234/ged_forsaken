#include "InputHelper.h"

#include "OgreBullet.h"
#include <iostream>

InputHelper::InputHelper()
{
    // keys
    aDown = dDown = wDown = sDown = fDown = jDown = esc = false;
}

InputHelper::~InputHelper()
{

}

/* Uses the OgreBites::InputListener, but can also use SDL2 */
bool InputHelper::keyPressed(const KeyboardEvent& evt)
{
    //std::cout << "Got key down event" << std::endl;
    if (evt.keysym.sym == SDLK_ESCAPE)
    {
        esc = true;
    }

    if (evt.keysym.sym == 'w')
    {
        wDown = true;
    }

    if (evt.keysym.sym == 'a')
    {
        aDown = true;
    }

    if (evt.keysym.sym == 's')
    {
        sDown = true;
    }

    if (evt.keysym.sym == 'd')
    {
        dDown = true;
    }

    if (evt.keysym.sym == 'j')
    {
        jDown = true;
    }

    if (evt.keysym.sym == 'f')
    {
        fDown = true;
    }

    return true;
}

bool InputHelper::keyReleased(const KeyboardEvent& evt)
{
    //std::cout << "Got key up event" << std::endl;

    if (evt.keysym.sym == 'w')
    {
        wDown = false;
    }

    if (evt.keysym.sym == 'a')
    {
        aDown = false;
    }

    if (evt.keysym.sym == 's')
    {
        sDown = false;
    }

    if (evt.keysym.sym == 'd')
    {
        dDown = false;
    }

    if (evt.keysym.sym == 'j')
    {
        jDown = false;
    }

    if (evt.keysym.sym == 'f')
    {
        fDown = false;
    }

    return true;
}

bool InputHelper::mouseMoved(const MouseMotionEvent& evt)
{
    //std::cout << "Got Mouse" << std::endl;
    return true;
}

