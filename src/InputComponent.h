#pragma once
#include "InputHelper.h"
#include "GameObject.h"

class InputComponent
{
public:
	virtual void handleInput(InputHelper inputHelper, GameObject* gameObject) = 0;
};
