#include "PlayerInputComponent.h"
#include "GameObject.h"
#include <iostream>
#include "PhysicsComponent.h"


float forwardForce = 5.0f;
float turningForce = 25.0f;
float jumpForce = 2.0f;
float linearDamping = 20.8f;
float angularDamping = 12.8f;

PlayerInputComponent::PlayerInputComponent()
{

}

PlayerInputComponent::~PlayerInputComponent()
{

}

void PlayerInputComponent::handleInput(InputHelper inputHelper, GameObject* player)
{
    if (inputHelper.wDown)
        player->physicsComponent->forward();

    if (inputHelper.aDown)
        player->physicsComponent->turnLeft();

    if (inputHelper.sDown)
        player->physicsComponent->backward();

    if (inputHelper.dDown)
        player->physicsComponent->turnRight();

    if (inputHelper.jDown)
        player->physicsComponent->jump();

    if (inputHelper.fDown)
        player->physicsComponent->fly();
}

