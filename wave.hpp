#pragma once
#include "UASimLib.hpp"



class Wave
{
    private:
    protected:
    public:
    std::vector<ray> nodes;
    std::vector<ray> mic_nodes;
    std::vector<ray> src_nodes;
    std::vector<Triangle> triangles;
    std::unordered_map<uint64_t, int> gEdgeMidCache; // edge -> midpoint
    bool mesh_dirty = true;

    float gAvgEdgeLen = 0.0f;                     // siatka
    int gBlindRings = 9 * 3 * 9 * 27 * 9 * 9 * 9; // 3 tr�jk�ty
    float gBlindRadius = 0.0f;                    // = gBlindRings * gAvgEdgeLen

    float inv_fp;
    float dt;
    int windows_number;
    std::vector<WindowPacket> gWinPackets;
    int gWinIdx = 0;

    shared_ptr<Red::ThreadPool> thread_pool;


    protected:
    size_t gRefineCursor = 0;



    int addMidpoint(int a, int b)
    {
        ray midpoint;
        midpoint.position = (nodes[a].position + nodes[b].position) * 0.5f;

        midpoint.energy = (nodes[a].energy + nodes[b].energy) * 0.5f;
        midpoint.tEmit = (nodes[a].tEmit + nodes[b].tEmit) * 0.5f;
        // midpoint.srcVel = (nodes[a].srcVel + nodes[b].srcVel) * 0.5f;
        midpoint.velocity = (nodes[a].velocity + nodes[b].velocity) * 0.5f;
        midpoint.velocity = vec3f::normalized(midpoint.velocity) * SOUND_V;

        // midpoint.srcVel = vec3f::normalized(midpoint.srcVel) * vec3f::length(nodes[a].srcVel);
        midpoint.bounces = nodes[a].bounces;
        midpoint.suppressUntilT = (nodes[a].suppressUntilT + nodes[b].suppressUntilT) * 0.5f;
        // midpoint.path = (nodes[a].path + nodes[b].path) * 0.5f;
        // midpoint.velocity = (nodes[a].velocity + nodes[b].velocity) * 0.5f;
        // midpoint.nEmit = (nodes[a].nEmit + nodes[b].nEmit) * 0.5f;
        // midpoint.nEmit = vec3f::normalized(midpoint.nEmit) * vec3f::length(nodes[a].nEmit);

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

        if(thread_pool == nullptr)
            thread_pool = make_shared<Red::ThreadPool>(threadCount);

        // zbierz do podzia�u + kraw�dzie
        std::vector<std::vector<int>> tl_split(threadCount);
        std::vector<std::vector<Edge>> tl_edges(threadCount);

        auto worker_scan = [&](std::any atid)
        {
            int tid = std::any_cast<int>(atid);
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

                const vec3f &pa = nodes[a].position;
                const vec3f &pb = nodes[b].position;
                const vec3f &pc = nodes[c].position;

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

                const vec3f ab = pb - pa;
                const vec3f ac = pc - pa;
                const vec3f cr = vec3f::cross(ab, ac);
                const float cr2 = vec3f::dot(cr, cr);

                if (cr2 > area2_threshold)
                {
                    split.push_back((int)i);
                    edges.push_back(make_edge(a, b));
                    edges.push_back(make_edge(b, c));
                    edges.push_back(make_edge(c, a));
                }
            }
        };
        
        std::vector<std::shared_ptr<Red::ThreadPool::DispatchHandle>> handles = thread_pool->DispatchIndexed(worker_scan, threadCount);
        
        for(auto& h : handles)
        {
            h->Wait();
            //printf("Handle %i competed\n", h->index);
            Red::Assert(h->done);
        }
        

        // std::vector<std::thread> threads;
        // threads.reserve(threadCount);
        // for (int t = 0; t < threadCount; ++t)
        //     threads.emplace_back(worker_scan, t);
        // for (auto &th : threads)
        //     th.join();
        // threads.clear();

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

        auto worker_build = [&](std::any atid)
        {
            int tid = std::any_cast<int>(atid);
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

        handles = thread_pool->DispatchIndexed(worker_build, threadCount);
        for(auto& h : handles)
        {
            h->Wait();
            //printf("Handle %i competed\n", h->index);
            Assert(h->done);
        }

        // for (int t = 0; t < threadCount; ++t)
        //     threads.emplace_back(worker_build, t);
        // for (auto &th : threads)
        //     th.join();

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
        std::vector<ray> kept;
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
                ray cp = nodes[i];
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
    void Begin(const SoundSource& source, int index=-1, float radius = DEFAULT_INITIAL_WAVE_RADIUS)
    {
        if (index != -1)
            gWinIdx = index;
        WindowPacket& packet = gWinPackets[gWinIdx];
                // gestosc - ka�dy poziom �4 liczba tr�jk�t�w
        constexpr int SUBDIV = 2; // 2 optymalnie, wiecej laguje
        // 12 wierzcho�k�w  na sferze o promieniu 'radius'
        const float t = (1.0f + std::sqrt(5.0f)) * 0.5f; // z�ota proporcja
        std::vector<vec3f> verts = {
            vec3f::normalized(vec3f{-1, t, 0}),
            vec3f::normalized(vec3f{1, t, 0}),
            vec3f::normalized(vec3f{-1, -t, 0}),
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
            v *= radius; // na promie� 'radius'

        // 20 �cian
        std::vector<vec3i> faces = {
            {0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11}, {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8}, {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9}, {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}};
        /*
        std::vector<vec3i> faces = {
            {0,11,5}, {0,5,1}, {0,1,7}, {0,7,10}, {0,10,11},
            {1,5,9},  {5,11,4}, {11,10,2}, {10,7,6}, {7,1,8},
            {3,9,4},  {3,4,2},  {3,2,6},  {3,6,8},  {3,8,9},
            {4,9,5},  {2,4,11}, {6,2,10}
        };*/

        // Podzial
        for (int s = 0; s < SUBDIV; ++s)
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

                // 4 nowe tr�jk�ty
                new_faces.push_back({i0, a, c});
                new_faces.push_back({i1, b, a});
                new_faces.push_back({i2, c, b});
                new_faces.push_back({a, b, c});
            }
            faces.swap(new_faces);
        }

        nodes.reserve(verts.size());
        vec3f buf2 = vec3f{source.src_x, source.src_y, source.src_z};
        vec3f gSourceVel = source.velocity;
        int i = 0;
        for (const auto &p : verts)
        {
            ray nd;
            nd.position = p;
            // nd.nEmit = vec3f::normalized(p);
            nd.velocity = vec3f::normalized(p) * SOUND_V; // sta�a pr�dko�� fali w wodzie

            nd.position += buf2;
            nd.doppler = vec3f::length(nd.velocity) + vec3f::length(source.velocity);
            // nd.srcVel = gSourceVel;
            // nd.energy = audioE;
            // nd.tEmit = (gWinIdx)*gAudio.window_ms / 1000.0f; // zapisz czas emisji (sim-time)
            // nd.tEmit = (gWinIdx)*window_ms / 1000.0f;

            nd.energy = packet.amplitude; // amplituda okna
            nd.tEmit = packet.tEmit;      // czas emisji okna (sek)
            // nd.winId = (int)gWinIdx;
            nd.seedId = i++;

            nodes.push_back(nd);
        }

        triangles.reserve(faces.size());
        for (const auto &f : faces)
        {
            triangles.push_back({{f.x, f.y, f.z}});
            // break;
        }
        gAvgEdgeLen = computeAvgEdgeLen(verts, faces);
        gBlindRadius = gBlindRings * gAvgEdgeLen;
    }
    Wave(float frequency, float delta_step, int windows, vector<WindowPacket> packets)
    {
        inv_fp = frequency;
        dt = delta_step;
        windows_number = windows;
        gWinPackets = packets;
    }
};