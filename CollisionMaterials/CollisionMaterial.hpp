#pragma once

class CollisionMaterial
{
    public:
    std::any custom_data;
    virtual void ProcessCollision(float delta, float time, ray& r, ray_collision& rc) = 0;
};
