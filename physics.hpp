#pragma once


// interpolacja liniowa  logMicHit na krok dt + kwantyzacja q5
inline float flush_echo_interpolated(Microphone& Mic, shared_ptr<Wave> wave, const std::vector<std::pair<float, float>> &buf,
                                     float TS_native, bool quantize = true)
{
    if (buf.empty())
        return 0;
    if (buf.size() == 1)
    {
        const float t0 = quantize ? q5_inter(wave->inv_fp, buf[0].first) : buf[0].first;
        Mic.logMicHit(t0, buf[0].second);
        return 0;
    }

    const float EPS = TS_native * 1e-3;
    const float STEP = wave->inv_fp;       // rozdzielczo�� siatki (np. 1e-5)
    const float DUP_E = 0.5f * STEP; // pr�g duplikatu
    const float GAP_E = 1.5f * STEP; // pr�g przeskoku o wi�cej ni� 1 tick

    float t_start = buf.front().first;
    float t_end = buf.back().first;

    size_t k = 0;
    float new_T = 0.0f;

    // zapisany wskaznik czasu
    float poprzednie_t = std::numeric_limits<float>::quiet_NaN();

    // brak duplikator i przeskokow
    auto snap_guard = [&](float t_prop)
    {
        if (!quantize || !std::isfinite(poprzednie_t))
            return t_prop;
        const float dt = t_prop - poprzednie_t;

        if (dt <= DUP_E)
        {
            return poprzednie_t + STEP;
        }

        if (dt >= GAP_E)
        {
            return poprzednie_t + STEP;
        }

        return t_prop;
    };

    for (float tg = t_start; tg <= t_end + 0.5f * TS_native; tg += TS_native)
    {
        tg = q5_inter(wave->inv_fp, tg);
        while (k + 1 < buf.size() && buf[k + 1].first < tg - EPS)
            ++k;
        if (k + 1 >= buf.size())
        {
            const float v_last = buf.back().second;
            const float t_raw = buf.back().first;
            float t_q = quantize ? q5_inter(wave->inv_fp, t_raw) : t_raw;
            t_q = q5_inter(wave->inv_fp, snap_guard(t_q));

            Mic.logMicHit(t_q, v_last);
            poprzednie_t = t_q;
            new_T = t_q;
            break;
        }

        const float ta = buf[k].first, tb = buf[k + 1].first;
        const float va = buf[k].second, vb = buf[k + 1].second;

        float v;
        if (std::fabs(tb - ta) <= 1e-12f)
            v = va;
        else if (tg <= ta + EPS)
            v = va;
        else if (tg >= tb - EPS)
            v = vb;
        else
        {
            const float a = (tg - ta) / (tb - ta);
            v = va + a * (vb - va);
        }

        // proponowany znacznik czasu
        float t_prop = quantize ? q5_inter(wave->inv_fp, tg) : tg;

        float t = q5_inter(wave->inv_fp, snap_guard(t_prop));

        Mic.logMicHit(t, v);
        poprzednie_t = t;
    }
    return new_T;
}

inline int pick_prev_hit(const EchoHit &cur,
                         const std::vector<EchoHit> &prev,
                         std::vector<char> &prevTaken,
                         float dt,
                         float ang_deg_gate = 20.f,
                         float max_logE = 1.0f)
{
    if (prev.empty())
        return -1;

    const float cos_gate = std::cos(glm::radians(ang_deg_gate));

    int best = -1;
    float bestCost = 1e9f;

    for (int p = 0; p < (int)prev.size(); ++p)
    {
        if (prevTaken.size() != prev.size())
            prevTaken.assign(prev.size(), 0);
        if (prevTaken[p])
            continue;

        const auto &pr = prev[p];

        // Twarde bramkowanie po liczbie odbi�
        if (pr.bounces != cur.bounces)
            continue;

        // kierunek musi by� bliski
        float cosang = glm::dot(glm::normalize(cur.dir_in), glm::normalize(pr.dir_in));
        if (cosang < cos_gate)
            continue;

        // Energia: log-przestrze�
        float logE = std::fabs(std::log(std::max(cur.energy, 1e-12f)) - std::log(std::max(pr.energy, 1e-12f)));
        if (logE > max_logE)
            continue;

        // predykcja czasu startu nast�pnego okna na bazie poprzedniego T2
        float Tpred = pr.T2 + dt;

        // Sk�adowe kosztu
        float dT = std::fabs(cur.T - Tpred);
        float dang = std::acos(std::clamp(cosang, -1.0f, 1.0f)); // [rad]
        float dD = std::fabs(cur.doppler - pr.doppler);

        float cost = 3.0f * dT + 0.6f * dang + 1.0f * logE + 0.3f * dD;

        if (cost < bestCost)
        {
            bestCost = cost;
            best = p;
        }
    }

    if (best >= 0)
        prevTaken[best] = 1; // zarezerwuj
    return best;
}

inline bool touchesMicrophonePoint(float dt, Microphone& Mic, shared_ptr<Wave> wave, int nodeIndex)
{
    const glm::vec3 center(Mic.mic_x, Mic.mic_y, Mic.mic_z);
    const glm::vec3 &pNode = wave->nodes[nodeIndex].position;

    float maxExtra = wave->gAvgEdgeLen * 1.5f;
    float farR2 = 1440 * dt * 10;
    float buf = glm::length(pNode - center);
    if (buf > farR2)
    {
        return false;
    }

    const float soundSpeed = 1440.0f;
    const float maxAdvance = (soundSpeed * dt) * 1.5;

    // kierunek propagacji z pr�dko�ci noda
    glm::vec3 n_prop = glm::normalize(wave->nodes[nodeIndex].velocity);

    for (const auto &tri : wave->triangles)
    {
        if (tri.indices[0] != nodeIndex &&
            tri.indices[1] != nodeIndex &&
            tri.indices[2] != nodeIndex)
            continue;

        const glm::vec3 &a = wave->nodes[tri.indices[0]].position;
        const glm::vec3 &b = wave->nodes[tri.indices[1]].position;
        const glm::vec3 &c = wave->nodes[tri.indices[2]].position;

        // normalna geometryczna
        glm::vec3 ab = b - a;
        glm::vec3 ac = c - a;
        glm::vec3 n = glm::cross(ab, ac);
        float n2 = glm::dot(n, n);
        if (n2 < 1e-12f)
            continue; // zdegenerowany tr�jk�t

        glm::vec3 n_hat = n / std::sqrt(n2);

        // dopasuj zwrot normalnej do kierunku propagacji
        if (glm::dot(n_hat, n_prop) < 0.0f)
            n_hat = -n_hat;

        // distance od p�aszczyzny czo�a
        float s = glm::dot(center - a, n_hat);

        // mikrofon musi by� przed czo�em, ale nie dalej ni� fala dojdzie w tym kroku
        if (std::fabs(s) > maxAdvance)
            continue;

        // rzut mikrofonu na p�aszczyzn� czo�a
        glm::vec3 proj = center - s * n_hat;

        // centra dla proj
        glm::vec3 v0 = ab;
        glm::vec3 v1 = ac;
        glm::vec3 v2 = proj - a;

        float d00 = glm::dot(v0, v0);
        float d01 = glm::dot(v0, v1);
        float d11 = glm::dot(v1, v1);
        float d20 = glm::dot(v2, v0);
        float d21 = glm::dot(v2, v1);
        float denom = d00 * d11 - d01 * d01;
        if (std::fabs(denom) < 1e-12f)
            continue;

        float v = (d11 * d20 - d01 * d21) / denom;
        float w = (d00 * d21 - d01 * d20) / denom;
        float u = 1.0f - v - w;

        if (u >= 0.0f && v >= 0.0f && w >= 0.0f)
        {
            // punktowy mikrofon zostanie przeci�ty przez to czo�o fali w czasie <= dt
            return true;
        }
    }

    return false;
}

inline void retime_echo(shared_ptr<Wave> wave, std::vector<std::pair<float, float>> &buf, float newEnd, float prevE, float currE)
{
    if (buf.size() < 2)
    {
        if (!buf.empty())
            buf.back().first = newEnd;
        return;
    }
    const float t0 = buf.front().first; // stary start (by� r�wny T)
    const float t1 = buf.back().first;  // stary koniec (by� r�wny T2_prev)
    const float den = (t1 - t0);
    if (std::fabs(den) < 1e-12f)
    {
        buf.back().first = newEnd;
        return;
    }
    const float s = (newEnd - t0) / den; // skala czasu
    int j = 0;
    for (auto &p : buf)
    {
        p.first = t0 + (p.first - t0) * s;
        // if (j == 0) p.first = t0;
        // if (j == buf.size() - 1) p.first = newEnd;
        j++;
    }

    float invAmpPrev = wave->gWinPackets[wave->gWinIdx - 1].amplitude / prevE;
    float PrevAmp = prevE / wave->gWinPackets[wave->gWinIdx - 1].amplitude;
    float newAmpS = currE / wave->gWinPackets[wave->gWinIdx].amplitude;
    newAmpS = (PrevAmp - newAmpS) / buf.size();
    int i = 0;
    for (auto &p : buf)
    {
        p.second = p.second * invAmpPrev;
        p.second = p.second * (PrevAmp - newAmpS * i);
        i++;
    }
}

inline int bounce1D(float &pos, float &vel, float minb, float maxb)
{
    const float eps = 0.001f; // minimalne odsuni�cie od �ciany
    if (pos < minb)
    {
        pos = minb + eps; // wyci�gnij ze sciany
        vel = -vel;       // odbij
        return -1;
    }
    else if (pos > maxb)
    {
        pos = maxb - eps;
        vel = -vel;
        return +1;
    }
    return 0;
};

inline void updatePhysics(float dt, float window_ms, float time_passed, shared_ptr<Wave> wave, SoundSource& source, Microphone& Mic, 
    struct Cuboid_dimensions Pool, struct Cuboid_dimensions temp_Obstacle) // glowna petla fizyki programu
{
    const float Pool_halfW = 0.5f * Pool.width;
    const float Pool_halfH = 0.5f * Pool.height;
    const float Pool_halfD = 0.5f * Pool.depth;

    const float e = 0.8;      // wsp. spr�ysto�ci
    const float eps = 0.001f; // minimalne odsuni�cie od �ciany

    const float micR = Mic.radius;
    // Pomocnik do odbicia



    // odbicie mikrofonu
    Mic.mic_x += Mic.mic_velocity.x * dt;
    Mic.mic_y += Mic.mic_velocity.y * dt;
    Mic.mic_z += Mic.mic_velocity.z * dt;

    source.src_x += source.velocity.x * dt;
    source.src_y += source.velocity.y * dt;
    source.src_z += source.velocity.z * dt;

    // Odbicia mikrofonu od �cian basenu
    bounce1D(Mic.mic_x, Mic.mic_velocity.x, -Pool_halfW + Pool.x_offset + micR, Pool_halfW + Pool.x_offset - micR);
    bounce1D(Mic.mic_y, Mic.mic_velocity.y, -Pool_halfH + Pool.y_offset + micR, Pool_halfH + Pool.y_offset - micR);
    bounce1D(Mic.mic_z, Mic.mic_velocity.z, -Pool_halfD + Pool.z_offset + micR, Pool_halfD + Pool.z_offset - micR);

    // odbicia zrodla od scian
    bounce1D(source.src_x, source.velocity.x, -Pool_halfW + Pool.x_offset, Pool_halfW + Pool.x_offset);
    bounce1D(source.src_y, source.velocity.y, -Pool_halfH + Pool.y_offset, Pool_halfH + Pool.y_offset);
    bounce1D(source.src_z, source.velocity.z, -Pool_halfD + Pool.z_offset, Pool_halfD + Pool.z_offset);

    // odbicia fali od scian
    for (int i = 0; i < (int)wave->nodes.size(); ++i)
    {
        auto &p = wave->nodes[i].position;
        auto &v = wave->nodes[i].velocity;
        auto &energy = wave->nodes[i].energy;

        // nowe pozycje
        p += v * dt;

        // Odbicia w XYZ
        // --- w p�tli po node'ach ---
        bool bouncedAny = false;

        // --- Odbicie X: t�umienie 0.5 ---
        int sideX = bounce1D(p.x, v.x,
                             -Pool_halfW + Pool.x_offset,
                             Pool_halfW + Pool.x_offset);
        if (sideX != 0)
        {
            bouncedAny = true;
            energy *= 0.5f;
        }

        // --- Odbicie Y: d� 0.5, G�RA 0.3 ---
        int sideY = bounce1D(p.y, v.y,
                             -Pool_halfH + Pool.y_offset,
                             Pool_halfH + Pool.y_offset);
        if (sideY != 0)
        {
            bouncedAny = true;

            if (sideY > 0)
            {
                // g�rna �ciana basenu (y = maxb)
                energy *= 0.3f;
            }
            else
            {
                // dolna �ciana
                energy *= 0.5f;
            }
        }

        // --- Odbicie Z: t�umienie 0.5 ---
        int sideZ = bounce1D(p.z, v.z,
                             -Pool_halfD + Pool.z_offset,
                             Pool_halfD + Pool.z_offset);
        if (sideZ != 0)
        {
            bouncedAny = true;
            energy *= 0.5f;
        }
        bool hitMic = false;
        hitMic = touchesMicrophonePoint(dt, Mic, wave, (int)i);
        //glm::vec3 buf = wave->nodes[i].position - glm::vec3(Mic.mic_x, Mic.mic_y, Mic.mic_z);
        if (hitMic)
        {
            float p = wave->nodes[i].energy;
            float r = time_passed * SOUND_V;
            float E = std::abs(wave->nodes[i].energy);
            if (r > 1)
                E = p / (r);

            float win_s = window_ms / 1000.0f - (wave->gWinPackets[wave->gWinIdx].times[1] - wave->gWinPackets[wave->gWinIdx].times[0]); // 0.005s w teorii w praktyce lekko mniej niz 0.005s zeby nie brac dwa razy tej samej probki
            float T = wave->gWinPackets[wave->gWinIdx].tEmit + time_passed;                                                  // czas przyjscia pocz�tku okna
            const int wid = wave->gWinIdx;
            // Je�eli ten node jest czasowo st�umiony nie loguj do CSV
            if (T <= wave->nodes[i].suppressUntilT)
            {
                continue;
            }

            ktore_odbicie++;
            // DOPPLER (rozci�gni�cie/�ci�ni�cie osi czasu okna)
            const glm::vec3 n_hat = glm::normalize(wave->nodes[i].velocity);
            const float v_mic_proj = glm::dot(Mic.mic_velocity, n_hat);
            float vel_test = glm::length(wave->nodes[i].velocity);
            // CECHY echa
            if (r > 1)
                wave->nodes[i].energy = p / (r);
            EchoHit curHit;
            curHit.T = T; //  T z bie��cej ramki
            curHit.energy = std::fabs(wave->nodes[i].energy);
            curHit.bounces = wave->nodes[i].bounces;
            curHit.doppler = wave->nodes[i].doppler;
            curHit.dir_in = glm::normalize(wave->nodes[i].velocity);
            int test = -1;

            const glm::vec3 micC(Mic.mic_x, Mic.mic_y, Mic.mic_z);

            // Proba dopasowania okna do poprzedniego echa
            if (wave->gWinIdx != 0 && !Mic.gPrevHits.empty())
            {
                int match = pick_prev_hit(curHit, Mic.gPrevHits, Mic.gPrevTaken, dt);
                Mic.matches.push_back(match);
                test = match;
                if (match >= 0)
                {
                    // start tego okna jest koncem poprzedniego
                    //   przeskaluj ca�y bufor poprzedniego okna: stare T2_prev -> nowe T
                    if (match < (int)Mic.wektorPrev.size() && !Mic.wektorPrev[match].empty())
                    {
                        retime_echo(wave, Mic.wektorPrev[match], /*newEnd=*/T, Mic.gPrevHits[match].energy, wave->nodes[i].energy);
                        // interpolacja -> Mic csv
                        float new_T = flush_echo_interpolated(Mic, wave, Mic.wektorPrev[match], /*TS_native=*/wave->inv_fp, /*quantize=*/true);
                        Mic.wektorPrev[match].clear();
                    }
                }
            }
            // T2 liczymy dopiero po korekcie czasu T
            float T2 = T + win_s * wave->nodes[i].doppler * ((1) / (vel_test - v_mic_proj));
            curHit.T2 = T2;
            Mic.gCurrHits.push_back(curHit);

            float scale = (T2 - T) / win_s;
            if (scale <= 0.0f)
                scale = 1e-4f;

            // Logowanie pe�nego okna 5 ms (czas + warto�ci)
            if (wid >= 0 && wid < (int)wave->gWinPackets.size())
            {
                const auto &wp = wave->gWinPackets[wid];
                const float Awin = (wp.amplitude != 0.0f) ? wp.amplitude : 1e-12f;
                // --- sta�e do amplitudy
                const float scaleAmp = std::fabs(wave->nodes[i].energy / Awin);

                // BUFOR: (t_abs, val) dla wszystkich pr�bek okna
                std::vector<std::pair<float, float>> tmp;
                tmp.reserve(wp.times.size());

                for (size_t j = 0; j < wp.times.size(); ++j)
                {
                    // czas absolutny po Dopplerze (bez kwantyzacji)
                    float t_abs = T + scale * wp.times[j];

                    float val = (wp.values[j]) * scaleAmp;
                    if (wave->gWinIdx == wave->gWinPackets.size() - 1)
                        val = val;

                    tmp.emplace_back(t_abs, val);
                }
                Mic.wektor.push_back(tmp);
                Mic.wektorCurr.push_back(tmp);

                // posortuj po czasie (powinno ju� by� rosn�co przy scale>0)
                std::sort(tmp.begin(), tmp.end(), [](const auto &a, const auto &b)
                        { return a.first < b.first; });

                // INTERPOLACJA do kroku tego okna
                if (wave->gWinIdx == wave->windows_number - 1)
                {
                    flush_echo_interpolated(Mic, wave, tmp, /*TS_native=*/wave->inv_fp, /*quantize=*/true);
                }
            }

            // O�LEPIANIE WY��CZNIE TEGO SAMEGO CZO�A - zeby nie odczytac 2 razy tego samego
            // Wersja sta�a:
            const float blind_until = T + 0.00005f;

            // parametry filtr�w klastra
            const glm::vec3 center = wave->nodes[i].position; // pozycja trafionego noda
            // const glm::vec3 micC = glm::vec3(Mic.mic_x, Mic.mic_y, Mic.mic_z);
            const float cosMax = std::cos(glm::radians(20.0f));          // max r�nica kierunku
            const float s_i = glm::dot(micC - wave->nodes[i].position, n_hat); // proxy czasu doj�cia

            for (size_t j = 0; j < wave->nodes.size(); ++j)
            {
                if (j == i)
                    continue;

                // To samo okno emisji (to samo czo�o czasowo)
                if (wave->gWinIdx != wid)
                    continue;

                // Zgodny kierunek propagacji (wyklucz inne odbicia)
                const glm::vec3 nj_hat = glm::normalize(wave->nodes[j].velocity);
                if (glm::dot(n_hat, nj_hat) < cosMax)
                    continue;

                // Node musi zbli�a� si� do mikrofonu
                const float s_j = glm::dot(micC - wave->nodes[j].position, nj_hat);
                // if (s_j <= 0.0f) continue;

                // Zbli�ony przewidywany czas doj�cia
                if (std::fabs(s_j - s_i) > 1.5f * wave->gBlindRadius)
                    continue;

                // Blisko w przestrzeni (ok. �3 tr�jk�ty�)
                if (glm::length(wave->nodes[j].position - center) > wave->gBlindRadius)
                    continue;

                
                wave->nodes[j].suppressUntilT = std::max(wave->nodes[j].suppressUntilT, blind_until);
            }

            std::cout << "win=" << wid << "  T=" << T << " " << ktore_odbicie << " " << "T2=" << T2 << "\n";
        }
    }
}