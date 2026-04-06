#pragma once
#include "hmain.hpp"



class Wave
{
    private:
    protected:
    public:
    std::vector<node> nodes;
    std::vector<node> mic_nodes;
    std::vector<node> src_nodes;
    std::vector<Triangle> triangles;
    std::unordered_map<uint64_t, int> gEdgeMidCache; // edge -> midpoint
    bool mesh_dirty = true;

    protected:
    size_t gRefineCursor = 0;



    int addMidpoint(int a, int b)
    {
        node midpoint;
        midpoint.position = (nodes[a].position + nodes[b].position) * 0.5f;

        midpoint.energy = (nodes[a].energy + nodes[b].energy) * 0.5f;
        midpoint.tEmit = (nodes[a].tEmit + nodes[b].tEmit) * 0.5f;
        // midpoint.srcVel = (nodes[a].srcVel + nodes[b].srcVel) * 0.5f;
        midpoint.velocity = (nodes[a].velocity + nodes[b].velocity) * 0.5f;
        midpoint.velocity = glm::normalize(midpoint.velocity) * SOUND_V;

        // midpoint.srcVel = glm::normalize(midpoint.srcVel) * glm::length(nodes[a].srcVel);
        midpoint.bounces = nodes[a].bounces;
        midpoint.suppressUntilT = (nodes[a].suppressUntilT + nodes[b].suppressUntilT) * 0.5f;
        // midpoint.path = (nodes[a].path + nodes[b].path) * 0.5f;
        // midpoint.velocity = (nodes[a].velocity + nodes[b].velocity) * 0.5f;
        // midpoint.nEmit = (nodes[a].nEmit + nodes[b].nEmit) * 0.5f;
        // midpoint.nEmit = glm::normalize(midpoint.nEmit) * glm::length(nodes[a].nEmit);

        midpoint.doppler = (nodes[a].doppler + nodes[b].doppler) * 0.5f;
        nodes.push_back(midpoint);
        return (int)(nodes.size() - 1);
    }
    
    int midpoint_index_nodes_cached(int i0, int i1)
    {
        uint64_t key = edge_key(i0, i1);
        auto it = gEdgeMidCache.find(key);
        if (it != gEdgeMidCache.end())
            return it->second;
        int idx = addMidpoint(i0, i1); // midpoint(pozycja, predkosc)
        gEdgeMidCache.emplace(key, idx);
        return idx;
    }

    // Zwraca indeks midpointu; je�li nie istnieje tworzy addMidpoint
    int midpoint_index_nodes(int i0, int i1, std::unordered_map<uint64_t, int> &cache)
    {
        uint64_t key = edge_key(i0, i1);
        auto it = cache.find(key);
        if (it != cache.end())
            return it->second;

        int idx = addMidpoint(i0, i1); // �rednia pozycji i pr�dko�ci
        cache.emplace(key, idx);
        return idx;
    }

    public:

    bool refineIcosahedron_chunked_mt(float maxArea,
                                  size_t triBudget, Cuboid_dimensions Cube, // ile tr�jk�t�w obrabiamy w tej klatce
                                  int threadCount = std::thread::hardware_concurrency())
    {
        if (triangles.empty() || triBudget == 0)
            return false;
        if (threadCount <= 0)
            threadCount = 1;

        // je�li siatka by�a resetowana
        if (gRefineCursor > triangles.size())
        {
            gRefineCursor = 0;
            gEdgeMidCache.clear();
        }

        // pr�g bez sqrt: |cross|^2 > (2*maxArea)^2 zamiast porownywac pola porownyujemy kwadraty
        const float area2_threshold = 4.0f * maxArea * maxArea;

        // sta�e kolizyjne
        const float cuboidHalfWidth = Cube.width * 0.5f;
        const float cuboidHalfHeight = Cube.height * 0.5f;
        const float cuboidHalfDepth = Cube.depth * 0.5f;
        const float safetyMargin = 0.5f;

        // pracujemy wy��cznie na starej tablicy trojkatow
        const size_t size0 = triangles.size();
        size_t start = (gRefineCursor < size0 ? gRefineCursor : 0);
        size_t end = std::min(start + triBudget, size0);
        const size_t N = (end > start ? end - start : 0);
        if (N == 0)
            return false;

        threadCount = std::min<int>(threadCount, (int)N);

        // zbierz do podzia�u + kraw�dzie
        std::vector<std::vector<int>> tl_split(threadCount);
        std::vector<std::vector<Edge>> tl_edges(threadCount);

        auto worker_scan = [&](int tid)
        {
            size_t chunk = (N + threadCount - 1) / threadCount;
            size_t i0 = start + tid * chunk;
            size_t i1 = std::min(end, i0 + chunk);
            auto &split = tl_split[tid];
            auto &edges = tl_edges[tid];
            split.reserve((i1 > i0) ? (i1 - i0) / 2 : 0);
            edges.reserve((i1 > i0) ? 3 * (i1 - i0) / 2 : 0);

            for (size_t i = i0; i < i1; ++i)
            {
                Triangle t = triangles[i]; // kopiuj
                int a = t.indices[0], b = t.indices[1], c = t.indices[2];

                const glm::vec3 &pa = nodes[a].position;
                const glm::vec3 &pb = nodes[b].position;
                const glm::vec3 &pc = nodes[c].position;

                const bool nearWall =
                    (std::fabs(pa.x) > cuboidHalfWidth - safetyMargin) ||
                    (std::fabs(pa.y) > cuboidHalfHeight - safetyMargin) ||
                    (std::fabs(pa.z) > cuboidHalfDepth - safetyMargin) ||
                    (std::fabs(pb.x) > cuboidHalfWidth - safetyMargin) ||
                    (std::fabs(pb.y) > cuboidHalfHeight - safetyMargin) ||
                    (std::fabs(pb.z) > cuboidHalfDepth - safetyMargin) ||
                    (std::fabs(pc.x) > cuboidHalfWidth - safetyMargin) ||
                    (std::fabs(pc.y) > cuboidHalfHeight - safetyMargin) ||
                    (std::fabs(pc.z) > cuboidHalfDepth - safetyMargin);

                if (nearWall)
                    continue;

                const glm::vec3 ab = pb - pa;
                const glm::vec3 ac = pc - pa;
                const glm::vec3 cr = glm::cross(ab, ac);
                const float cr2 = glm::dot(cr, cr);

                if (cr2 > area2_threshold)
                {
                    split.push_back((int)i);
                    edges.push_back(make_edge(a, b));
                    edges.push_back(make_edge(b, c));
                    edges.push_back(make_edge(c, a));
                }
            }
        };

        std::vector<std::thread> threads;
        threads.reserve(threadCount);
        for (int t = 0; t < threadCount; ++t)
            threads.emplace_back(worker_scan, t);
        for (auto &th : threads)
            th.join();
        threads.clear();

        // scalenie wynik�w
        std::vector<int> splitIdx;
        std::vector<Edge> edges;
        {
            size_t totalSplit = 0, totalEdges = 0;
            for (int t = 0; t < threadCount; ++t)
            {
                totalSplit += tl_split[t].size();
                totalEdges += tl_edges[t].size();
            }
            if (totalSplit == 0)
            { // nic do roboty
                gRefineCursor = end < size0 ? end : 0;
                return false;
            }
            splitIdx.reserve(totalSplit);
            edges.reserve(totalEdges);
            for (int t = 0; t < threadCount; ++t)
            {
                splitIdx.insert(splitIdx.end(), tl_split[t].begin(), tl_split[t].end());
                edges.insert(edges.end(), tl_edges[t].begin(), tl_edges[t].end());
                tl_split[t].clear();
                tl_edges[t].clear();
            }
        }

        // kraw�dzie
        std::sort(edges.begin(), edges.end(), edge_less);
        edges.erase(std::unique(edges.begin(), edges.end(), edge_eq), edges.end());

        // utw�rz midpointy -
        std::vector<int> edgeMidIdx(edges.size());
        nodes.reserve(nodes.size() + edges.size()); // minimalizacja realokacji
        for (size_t i = 0; i < edges.size(); ++i)
        {
            edgeMidIdx[i] = midpoint_index_nodes_cached(edges[i].a, edges[i].b);
        }

        auto edge_lookup = [&](int u, int v) -> int
        {
            if (u > v)
                std::swap(u, v);
            const uint64_t key = edge_key(u, v);
            auto it = gEdgeMidCache.find(key);
            return it->second;
        };

        // zbuduj nowe tr�jk�ty do bufor�w
        std::vector<std::vector<std::pair<int, Triangle>>> tl_replace(threadCount);
        std::vector<std::vector<Triangle>> tl_append(threadCount);

        auto worker_build = [&](int tid)
        {
            size_t M = splitIdx.size();
            size_t chunk = (M + threadCount - 1) / threadCount;
            size_t i0 = tid * chunk;
            size_t i1 = std::min(M, i0 + chunk);
            auto &rep = tl_replace[tid];
            auto &app = tl_append[tid];
            rep.reserve((i1 > i0) ? (i1 - i0) : 0);
            app.reserve((i1 > i0) ? 3 * (i1 - i0) : 0);

            for (size_t k = i0; k < i1; ++k)
            {
                int ti = splitIdx[k];
                Triangle t = triangles[ti];
                int a = t.indices[0], b = t.indices[1], c = t.indices[2];

                int ab = edge_lookup(a, b);
                int bc = edge_lookup(b, c);
                int ca = edge_lookup(c, a);

                // zamieniamy bie��cy
                rep.emplace_back(ti, Triangle{{a, ab, ca}});
                // 3 nowe
                app.push_back({{b, bc, ab}});
                app.push_back({{c, ca, bc}});
                app.push_back({{ab, bc, ca}});
            }
        };

        for (int t = 0; t < threadCount; ++t)
            threads.emplace_back(worker_build, t);
        for (auto &th : threads)
            th.join();

        // zapisz wynik do 'triangles' ---
        size_t toAppend = 0;
        for (int t = 0; t < threadCount; ++t)
            toAppend += tl_append[t].size();
        triangles.reserve(triangles.size() + toAppend);

        // podmiany
        for (int t = 0; t < threadCount; ++t)
        {
            for (auto &p : tl_replace[t])
            {
                triangles[p.first] = p.second;
            }
        }
        // dopisywanie 3 nowych
        for (int t = 0; t < threadCount; ++t)
        {
            triangles.insert(triangles.end(), tl_append[t].begin(), tl_append[t].end());
        }

        // przesuwamy kursor (zawsze wzgl�dem dawnego size0)
        gRefineCursor = end;
        if (gRefineCursor >= size0)
            gRefineCursor = 0;

        return true;
    }
    int pruneSlowNodes(float minEnergy, float time_passed)
    {
        if (nodes.empty())
            return 0;

        float thr2 = std::abs(minEnergy) / 200.0f;
        if (thr2 <= 0)
            thr2 = 0.05f;
        const size_t N = nodes.size();

        std::vector<int> remap(N, -1);
        std::vector<node> kept;
        kept.reserve(N);

        int test = 0;
        for (size_t i = 0; i < N; ++i)
        {
            float p = nodes[i].energy;
            float r = time_passed * SOUND_V;
            float E = std::abs(nodes[i].energy);
            if (r > 1)
                E = p / (r);

            bool EnergyEnough = (E >= thr2);


            if (EnergyEnough)
            {
                remap[i] = (int)kept.size();
                node cp = nodes[i];
                kept.push_back(cp);
            }
        }

        // zostaw te triangles, ktore przetrwaly
        std::vector<Triangle> newTris;
        newTris.reserve(triangles.size());
        for (const auto &t : triangles)
        {
            int na = remap[t.indices[0]];
            int nb = remap[t.indices[1]];
            int nc = remap[t.indices[2]];
            if (na >= 0 && nb >= 0 && nc >= 0)
            {
                newTris.push_back({{na, nb, nc}});
            }
        }

        nodes.swap(kept);
        triangles.swap(newTris);

        // Uniewa�nij cache i rebuild VBO
        gRefineCursor = 0;
        gEdgeMidCache.clear();
        mesh_dirty = true;

        return (int)(N - nodes.size());
    }
    // Zabija wszystkie nody i tr�jk�ty � do debugu.
    void killAllNodes()
    {
        nodes.clear();
        triangles.clear();

        // wyczy�� stan mesh
        gRefineCursor = 0;
        gEdgeMidCache.clear();
        mesh_dirty = true;
    }

};