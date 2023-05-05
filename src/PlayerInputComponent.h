#pragma once
#include "InputComponent.h"
#include "GameObject.h"

class PlayerInputComponent : public InputComponent
{
public:
	/**
	* Creates the object, sets all pointers to nullptr.
	*/
	PlayerInputComponent();

	/**
	* Destructor (virtual), as this is virtual that of the sub class will also be called.
	*/
	~PlayerInputComponent();

	/**
	* handles the input method from the interface
	*/
	void handleInput(InputHelper inputHelper, GameObject* gameObject) override;
};

