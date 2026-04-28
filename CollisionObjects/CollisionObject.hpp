#pragma once

class CollisionObject
{
    public:
    class Surface
    {
        public:
        int index;
        shared_ptr<CollisionMaterial> material;
        std::vector<vec3f> vertices;
        std::vector<int> indices;
        std::vector<vec3f> vertices_raw;
        std::any custom_data;
    };
    std::any custom_data;
    protected:
    std::vector<shared_ptr<Surface>> surfaces;
    virtual void RayCollisionInternal(float delta, float time, ray& r, ray_collision& rc) = 0;
    public:
    virtual void GetAABB(Red::vec<float, 6>& aabb) = 0;
    virtual void RayCollision(float delta, float time, ray& r, ray_collision& rc)
    {
        RayCollisionInternal(delta, time, r, rc);
        if(rc.has_collided)
        {
            surfaces[rc.surface]->material->ProcessCollision(delta, time, r, rc);
        }
    }

    void GetSurfaces(shared_ptr<Surface>* surfaces, int n)
    {
        surfaces = &this->surfaces[0];
        n = this->surfaces.size();
    }
};