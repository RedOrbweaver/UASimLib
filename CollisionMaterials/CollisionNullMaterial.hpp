#pragma once

class CollisionNullMaterial : public CollisionMaterial
{
    public:
    virtual void ProcessCollision(float delta, float time, ray& r, ray_collision& rc) override
    {

    }
};
