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

inline float computeAvgEdgeLen(const std::vector<glm::vec3> &V,
                               const std::vector<glm::ivec3> &F)
{
    double sum = 0;
    size_t cnt = 0;
    for (auto &t : F)
    {
        const auto &a = V[t.x], &b = V[t.y], &c = V[t.z];
        sum += glm::length(a - b);
        ++cnt;
        sum += glm::length(b - c);
        ++cnt;
        sum += glm::length(c - a);
        ++cnt;
    }
    return cnt ? float(sum / cnt) : 0.0f;
}


inline float dist2PointSegment(const glm::vec3 &p,
                               const glm::vec3 &a,
                               const glm::vec3 &b)
{
    glm::vec3 ab = b - a;
    float ab2 = glm::dot(ab, ab);
    if (ab2 < 1e-12f)
    {

        glm::vec3 d = p - a;
        return glm::dot(d, d);
    }

    float t = glm::dot(p - a, ab) / ab2;
    t = std::clamp(t, 0.0f, 1.0f);
    glm::vec3 q = a + t * ab;
    glm::vec3 d = p - q;
    return glm::dot(d, d);
}

inline float dist2PointTriangle(const glm::vec3 &p,
                                const glm::vec3 &a,
                                const glm::vec3 &b,
                                const glm::vec3 &c)
{
    glm::vec3 ab = b - a;
    glm::vec3 ac = c - a;
    glm::vec3 n = glm::cross(ab, ac);
    float n2 = glm::dot(n, n);

    if (n2 < 1e-12f)
    {
        float d2ab = dist2PointSegment(p, a, b);
        float d2bc = dist2PointSegment(p, b, c);
        float d2ca = dist2PointSegment(p, c, a);
        return std::min(d2ab, std::min(d2bc, d2ca));
    }

    glm::vec3 nNorm = n / std::sqrt(n2);
    float distPlane = glm::dot(p - a, nNorm);
    glm::vec3 proj = p - distPlane * nNorm;

    glm::vec3 v0 = ab;
    glm::vec3 v1 = ac;
    glm::vec3 v2 = proj - a;

    float d00 = glm::dot(v0, v0);
    float d01 = glm::dot(v0, v1);
    float d11 = glm::dot(v1, v1);
    float d20 = glm::dot(v2, v0);
    float d21 = glm::dot(v2, v1);
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
                                    std::vector<node> &out_nodes,
                                    std::vector<Triangle> &out_tris)
{
    // 12 wierzcho�k�w fali
    const float t = (1.0f + std::sqrt(5.0f)) * 0.5f;
    std::vector<glm::vec3> verts = {
        glm::normalize(glm::vec3(-1, t, 0)),
        glm::normalize(glm::vec3(1, t, 0)),
        glm::normalize(glm::vec3(-1, -t, 0)),
        glm::normalize(glm::vec3(1, -t, 0)),

        glm::normalize(glm::vec3(0, -1, t)),
        glm::normalize(glm::vec3(0, 1, t)),
        glm::normalize(glm::vec3(0, -1, -t)),
        glm::normalize(glm::vec3(0, 1, -t)),

        glm::normalize(glm::vec3(t, 0, -1)),
        glm::normalize(glm::vec3(t, 0, 1)),
        glm::normalize(glm::vec3(-t, 0, -1)),
        glm::normalize(glm::vec3(-t, 0, 1)),
    };
    for (auto &v : verts)
        v *= radius;

    std::vector<glm::ivec3> faces = {
        {0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11}, {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8}, {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9}, {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}};

    // podzialy z krawedzi
    for (int s = 0; s < subdiv; ++s)
    {
        std::unordered_map<uint64_t, int> cache;
        std::vector<glm::ivec3> new_faces;
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
        node nd;
        nd.position = p;
        nd.velocity = glm::vec3(0); // statyczne
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