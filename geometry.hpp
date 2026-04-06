#pragma once
#include "hmain.hpp"

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
inline glm::vec3 normalize_to_radius(const glm::vec3 &p, float r)
{
    float len = glm::length(p);
    if (len == 0.0f)
        return glm::vec3(r, 0, 0);
    return (p / len) * r;
}

// zwraca indeks �rodkowego wierzcho�ka (bez duplikatow)
inline int midpoint_index(int i0, int i1,
                          std::vector<glm::vec3> &verts,
                          std::unordered_map<uint64_t, int> &cache,
                          float radius)
{
    uint64_t key = edge_key(i0, i1);
    auto it = cache.find(key);
    if (it != cache.end())
        return it->second;

    glm::vec3 m = 0.5f * (verts[i0] + verts[i1]);
    m = normalize_to_radius(m, radius); // PROJEKCJA NA SFER�
    int idx = (int)verts.size();
    verts.push_back(m);
    cache.emplace(key, idx);
    return idx;
}