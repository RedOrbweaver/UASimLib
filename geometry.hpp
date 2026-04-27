#pragma once
#include "UASimLib.hpp"

// POMOCNICZE
struct Edge
{
    int a, b;
}; // a<b
inline Edge make_edge(int u, int v)
{
    if (u > v)
        std::swap(u, v);
    return {u, v};
}
inline bool edge_less(const Edge &x, const Edge &y)
{
    return (x.a < y.a) || (x.a == y.a && x.b < y.b);
}
inline bool edge_eq(const Edge &x, const Edge &y)
{
    return x.a == y.a && x.b == y.b;
}
// unikalny klucz kraw�dzi (mniejszy indeks najpierw)
inline uint64_t edge_key(int a, int b)
{
    if (a > b)
        std::swap(a, b);
    return (uint64_t(a) << 32) | uint32_t(b);
}

// rzut na sfer� (promie� r)
inline vec3f normalize_to_radius(const vec3f &p, float r)
{
    float len = vec3f::length(p);
    if (len == 0.0f)
        return vec3f{r, 0, 0};
    return (p / len) * r;
}

// zwraca indeks �rodkowego wierzcho�ka (bez duplikatow)
inline int midpoint_index(int i0, int i1,
                          std::vector<vec3f> &verts,
                          std::unordered_map<uint64_t, int> &cache,
                          float radius)
{
    uint64_t key = edge_key(i0, i1);
    auto it = cache.find(key);
    if (it != cache.end())
        return it->second;

    vec3f m = (verts[i0] + verts[i1]) * 0.5f;
    m = normalize_to_radius(m, radius); // PROJEKCJA NA SFER�
    int idx = (int)verts.size();
    verts.push_back(m);
    cache.emplace(key, idx);
    return idx;
}

inline float computeAvgEdgeLen(const std::vector<vec3f> &V,
                               const std::vector<vec3i> &F)
{
    double sum = 0;
    size_t cnt = 0;
    for (auto &t : F)
    {
        const auto &a = V[t.x], &b = V[t.y], &c = V[t.z];
        sum += vec3f::length(a - b);
        ++cnt;
        sum += vec3f::length(b - c);
        ++cnt;
        sum += vec3f::length(c - a);
        ++cnt;
    }
    return cnt ? float(sum / cnt) : 0.0f;
}


inline float dist2PointSegment(const vec3f &p,
                               const vec3f &a,
                               const vec3f &b)
{
    vec3f ab = b - a;
    float ab2 = vec3f::dot(ab, ab);
    if (ab2 < 1e-12f)
    {

        vec3f d = p - a;
        return vec3f::dot(d, d);
    }

    float t = vec3f::dot(p - a, ab) / ab2;
    t = std::clamp(t, 0.0f, 1.0f);
    vec3f q = a + (ab * t);
    vec3f d = p - q;
    return vec3f::dot(d, d);
}

inline float dist2PointTriangle(const vec3f &p,
                                const vec3f &a,
                                const vec3f &b,
                                const vec3f &c)
{
    vec3f ab = b - a;
    vec3f ac = c - a;
    vec3f n = vec3f::cross(ab, ac);
    float n2 = vec3f::dot(n, n);

    if (n2 < 1e-12f)
    {
        float d2ab = dist2PointSegment(p, a, b);
        float d2bc = dist2PointSegment(p, b, c);
        float d2ca = dist2PointSegment(p, c, a);
        return std::min(d2ab, std::min(d2bc, d2ca));
    }

    vec3f nNorm = n / std::sqrt(n2);
    float distPlane = vec3f::dot(p - a, nNorm);
    vec3f proj = p - (nNorm * distPlane);

    vec3f v0 = ab;
    vec3f v1 = ac;
    vec3f v2 = proj - a;

    float d00 = vec3f::dot(v0, v0);
    float d01 = vec3f::dot(v0, v1);
    float d11 = vec3f::dot(v1, v1);
    float d20 = vec3f::dot(v2, v0);
    float d21 = vec3f::dot(v2, v1);
    float denom = d00 * d11 - d01 * d01;

    float u = -1.f, v = -1.f, w = -1.f;
    if (std::fabs(denom) > 1e-12f)
    {
        v = (d11 * d20 - d01 * d21) / denom;
        w = (d00 * d21 - d01 * d20) / denom;
        u = 1.0f - v - w;
    }

    if (u >= 0.0f && v >= 0.0f && w >= 0.0f)
    {
        return distPlane * distPlane;
    }

    float d2ab = dist2PointSegment(p, a, b);
    float d2bc = dist2PointSegment(p, b, c);
    float d2ca = dist2PointSegment(p, c, a);
    return std::min(d2ab, std::min(d2bc, d2ca));
}

inline float q5(float dt, float t)
{
    // double w srodku zmniejsza blad
    double buf = 1 / dt;
    return (float)(std::llround((double)t * buf) / buf);
}

inline float q5_inter(float inv_fp, float t)
{
    // double w srodku zmniejsza blad
    double buf = 1 / inv_fp;
    return (float)(std::llround((double)t * buf) / buf);
}

// Zwraca sfer� wok� (0,0,0) o promieniu 'radius' i g�sto�ci 'subdiv'
inline void buildIcosphereNodesTris(float radius, int subdiv,
                                    std::vector<ray> &out_nodes,
                                    std::vector<Triangle> &out_tris)
{
    // 12 wierzcho�k�w fali
    const float t = (1.0f + std::sqrt(5.0f)) * 0.5f;
    std::vector<vec3f> verts = 
    {
        vec3f::normalized(vec3f{-1, t, 0}),
        vec3f::normalized(vec3f{1, t, 0}),
        vec3f::normalized(vec3f{-1, -t, }),
        vec3f::normalized(vec3f{1, -t, 0}),
        
        vec3f::normalized(vec3f{0, -1, t}),
        vec3f::normalized(vec3f{0, 1, t}),
        vec3f::normalized(vec3f{0, -1, -t}),
        vec3f::normalized(vec3f{0, 1, -t}),
        
        vec3f::normalized(vec3f{t, 0, -1}),
        vec3f::normalized(vec3f{t, 0, 1}),
        vec3f::normalized(vec3f{-t, 0, -1}),
        vec3f::normalized(vec3f{-t, 0, 1}),
    };
    for (auto &v : verts)
        v *= radius;

    std::vector<vec3i> faces = {
        {0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11}, {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8}, {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9}, {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}};

    // podzialy z krawedzi
    for (int s = 0; s < subdiv; ++s)
    {
        std::unordered_map<uint64_t, int> cache;
        std::vector<vec3i> new_faces;
        new_faces.reserve(faces.size() * 4);

        for (const auto &f : faces)
        {
            int i0 = f.x, i1 = f.y, i2 = f.z;
            int a = midpoint_index(i0, i1, verts, cache, radius);
            int b = midpoint_index(i1, i2, verts, cache, radius);
            int c = midpoint_index(i2, i0, verts, cache, radius);

            new_faces.push_back({i0, a, c});
            new_faces.push_back({i1, b, a});
            new_faces.push_back({i2, c, b});
            new_faces.push_back({a, b, c});
        }
        faces.swap(new_faces);
    }

    // konwersja
    out_nodes.clear();
    out_nodes.reserve(verts.size());
    for (const auto &p : verts)
    {
        ray nd;
        nd.position = p;
        nd.velocity = vec3f{0}; // statyczne
        nd.energy = 0.0f;
        out_nodes.push_back(nd);
    }

    out_tris.clear();
    out_tris.reserve(faces.size());
    for (const auto &f : faces)
    {
        out_tris.push_back({{f.x, f.y, f.z}});
    }
}