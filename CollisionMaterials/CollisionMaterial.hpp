#pragma once
//#include "UASimLib.hpp"

class CollisionMaterial
{
    public:
    virtual void ProcessCollision(float delta, float time, ray& r, ray_collision& rc) = 0;
};
