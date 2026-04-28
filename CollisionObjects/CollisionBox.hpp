#pragma once

class CollisionBox : public CollisionObject
{
public:
    vec3f bmin_local = {-0.5f, -0.5f, -0.5f};
    vec3f bmax_local = { 0.5f,  0.5f,  0.5f};

    vec3f position = {0,0,0};
    vec3f rotation = {0,0,0}; // Euler: X(pitch), Y(yaw), Z(roll)

protected:
    static vec3f RotateEuler(const vec3f &p, const vec3f &euler)
    {
        float sx = sinf(euler.x), cx = cosf(euler.x);
        vec3f v1 = { p.x, p.y*cx - p.z*sx, p.y*sx + p.z*cx };

        float sy = sinf(euler.y), cy = cosf(euler.y);
        vec3f v2 = { v1.x*cy + v1.z*sy, v1.y, -v1.x*sy + v1.z*cy };

        float sz = sinf(euler.z), cz = cosf(euler.z);
        vec3f v3 = { v2.x*cz - v2.y*sz, v2.x*sz + v2.y*cz, v2.z };

        return v3;
    }

    static vec3f RotateEulerInverse(const vec3f &p, const vec3f &euler)
    {
        float sz = sinf(-euler.z), cz = cosf(-euler.z);
        vec3f v1 = { p.x*cz - p.y*sz, p.x*sz + p.y*cz, p.z };

        float sy = sinf(-euler.y), cy = cosf(-euler.y);
        vec3f v2 = { v1.x*cy + v1.z*sy, v1.y, -v1.x*sy + v1.z*cy };

        float sx = sinf(-euler.x), cx = cosf(-euler.x);
        vec3f v3 = { v2.x, v2.y*cx - v2.z*sx, v2.y*sx + v2.z*cx };

        return v3;
    }

    static vec3f Reflect(const vec3f &I, const vec3f &N)
    {
        return I - N * (2.0f * I.dot(N));
    }

    virtual void RayCollisionInternal(float /*delta*/, float /*time*/, ray& r, ray_collision& rc) override
    {
        const float EPS = 1e-6f;

        vec3f localPos = RotateEulerInverse(r.position - position, rotation);
        vec3f localVel = RotateEulerInverse(r.velocity, rotation);

        if (fabsf(localVel.x) < EPS && fabsf(localVel.y) < EPS && fabsf(localVel.z) < EPS)
        {
            rc.has_collided = false;
            return;
        }

        float tmin = -FLT_MAX;
        float tmax = FLT_MAX;
        int hitFace = -1;

        // X slab
        if (fabsf(localVel.x) < EPS)
        {
            if (localPos.x < bmin_local.x || localPos.x > bmax_local.x) { rc.has_collided = false; return; }
        }
        else
        {
            float tx1 = (bmin_local.x - localPos.x) / localVel.x;
            float tx2 = (bmax_local.x - localPos.x) / localVel.x;
            if (tx1 > tx2) std::swap(tx1, tx2);
            if (tx1 > tmin) { tmin = tx1; hitFace = (localVel.x >= 0.0f) ? 1 : 0; }
            tmax = std::min(tmax, tx2);
            if (tmin > tmax) { rc.has_collided = false; return; }
        }

        // Y slab
        if (fabsf(localVel.y) < EPS)
        {
            if (localPos.y < bmin_local.y || localPos.y > bmax_local.y) { rc.has_collided = false; return; }
        }
        else
        {
            float ty1 = (bmin_local.y - localPos.y) / localVel.y;
            float ty2 = (bmax_local.y - localPos.y) / localVel.y;
            if (ty1 > ty2) std::swap(ty1, ty2);
            if (ty1 > tmin) { tmin = ty1; hitFace = (localVel.y >= 0.0f) ? 3 : 2; }
            tmax = std::min(tmax, ty2);
            if (tmin > tmax) { rc.has_collided = false; return; }
        }

        // Z slab
        if (fabsf(localVel.z) < EPS)
        {
            if (localPos.z < bmin_local.z || localPos.z > bmax_local.z) { rc.has_collided = false; return; }
        }
        else
        {
            float tz1 = (bmin_local.z - localPos.z) / localVel.z;
            float tz2 = (bmax_local.z - localPos.z) / localVel.z;
            if (tz1 > tz2) std::swap(tz1, tz2);
            if (tz1 > tmin) { tmin = tz1; hitFace = (localVel.z >= 0.0f) ? 5 : 4; }
            tmax = std::min(tmax, tz2);
            if (tmin > tmax) { rc.has_collided = false; return; }
        }

        if (tmax < 0.0f) { rc.has_collided = false; return; }

        float tHit = (tmin >= 0.0f) ? tmin : tmax;
        if (tHit < 0.0f || tHit < r.suppressUntilT) { rc.has_collided = false; return; }

        vec3f localColl = localPos + localVel * tHit;
        rc.collision_point = position + RotateEuler(localColl, rotation);

        vec3f localNormal;
        switch (hitFace)
        {
            case 0: localNormal = vec3f{ 1,0,0}; break;
            case 1: localNormal = vec3f{-1,0,0}; break;
            case 2: localNormal = vec3f{ 0,1,0}; break;
            case 3: localNormal = vec3f{ 0,-1,0}; break;
            case 4: localNormal = vec3f{ 0,0,1}; break;
            case 5: localNormal = vec3f{ 0,0,-1}; break;
            default: localNormal = vec3f{0,0,0}; break;
        }
        rc.collision_normal = RotateEuler(localNormal, rotation);

        rc.surface = hitFace >= 0 ? hitFace : 0;

        switch (hitFace)
        {
            case 0: case 1:
                rc.surface_coordinates.x = (localColl.y - bmin_local.y) / (bmax_local.y - bmin_local.y);
                rc.surface_coordinates.y = (localColl.z - bmin_local.z) / (bmax_local.z - bmin_local.z);
                break;
            case 2: case 3:
                rc.surface_coordinates.x = (localColl.x - bmin_local.x) / (bmax_local.x - bmin_local.x);
                rc.surface_coordinates.y = (localColl.z - bmin_local.z) / (bmax_local.z - bmin_local.z);
                break;
            case 4: case 5:
                rc.surface_coordinates.x = (localColl.x - bmin_local.x) / (bmax_local.x - bmin_local.x);
                rc.surface_coordinates.y = (localColl.y - bmin_local.y) / (bmax_local.y - bmin_local.y);
                break;
            default:
                rc.surface_coordinates = vec2f{0,0};
        }

        float vlen = r.velocity.dot(r.velocity) > 0.0f ? sqrtf(r.velocity.dot(r.velocity)) : 0.0f;
        vec3f inDirN = vlen > 1e-9f ? r.velocity / vlen : vec3f{0,0,0};
        vec3f refl = Reflect(inDirN, rc.collision_normal);
        rc.exit_vector = refl * vlen;
        rc.exit_position = rc.collision_point;
        rc.exit_energy = r.energy;
        rc.exit_delay = 0.0f;
        rc.has_collided = true;
        rc.has_delay = false;
    }

public:
    virtual void GetAABB(Red::vec<float, 6>& aabb) override
    {
        vec3f corners[8];
        for (int xi = 0; xi < 2; ++xi)
        for (int yi = 0; yi < 2; ++yi)
        for (int zi = 0; zi < 2; ++zi)
        {
            int i = (xi<<2) | (yi<<1) | zi;
            corners[i].x = xi ? bmax_local.x : bmin_local.x;
            corners[i].y = yi ? bmax_local.y : bmin_local.y;
            corners[i].z = zi ? bmax_local.z : bmin_local.z;
        }

        vec3f w = position + RotateEuler(corners[0], rotation);
        vec3f mn = w, mx = w;
        for (int i = 1; i < 8; ++i)
        {
            vec3f wc = position + RotateEuler(corners[i], rotation);
            mn.x = std::min(mn.x, wc.x); mn.y = std::min(mn.y, wc.y); mn.z = std::min(mn.z, wc.z);
            mx.x = std::max(mx.x, wc.x); mx.y = std::max(mx.y, wc.y); mx.z = std::max(mx.z, wc.z);
        }

        aabb[0] = mn.x; aabb[1] = mn.y; aabb[2] = mn.z;
        aabb[3] = mx.x; aabb[4] = mx.y; aabb[5] = mx.z;
    }

    CollisionBox() = default;
    CollisionBox(const vec3f& minp, const vec3f& maxp, const vec3f& pos = {0,0,0}, const vec3f& rot = {0,0,0}, std::array<shared_ptr<CollisionMaterial>, 6> materials = {})
        : bmin_local(minp), bmax_local(maxp), position(pos), rotation(rot) 
    {
        this->surfaces.reserve(6);
        for(int i = 0; i < 6; i++)
        {
            auto s = make_shared<Surface>();
            s->index = i;
            if(materials[i] != nullptr)
                s->material = materials[i];
            else
                s->material = make_shared<CollisionNullMaterial>();
            this->surfaces.push_back(s);
        }
    }
};
