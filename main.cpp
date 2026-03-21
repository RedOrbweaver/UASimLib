#include "hmain.hpp"

#define M_PI 3.1415

std::vector<int> matches;
// float dt = 0.00001f;
float inv_fp = 1 / 1000000.0f; // z jaka czestotliwoscia probki maja byc interpolowane
float dt = 1 / 100000.0f;      // dokladnosc i predkosc symulacji

std::vector<std::vector<std::pair<float, float>>> wektor;
static std::vector<std::vector<std::pair<float, float>>> wektorPrev; // bufory okien z poprzedniej ramki
static std::vector<std::vector<std::pair<float, float>>> wektorCurr; // bufory z bie��cej ramki



// Bufory do laczenia ramek
static std::vector<EchoHit> gPrevHits, gCurrHits;
static std::vector<char> gPrevTaken;

static int pick_prev_hit(const EchoHit &cur,
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

bool write_two_vectors_csv(const std::string &filepath,
                           const std::vector<float> &test_T,
                           const std::vector<float> &test_T2,
                           int precision = 5,
                           char sep = ',',
                           bool truncate_values = false)
{
    std::ofstream out(filepath);
    if (!out.is_open())
        return false;

    // kropka jako separator
    out.imbue(std::locale::classic());

    // nag��wek
    out << "T" << sep << "T2\n";

    const std::size_t n = std::max(test_T.size(), test_T2.size());
    out << std::fixed << std::setprecision(precision);

    for (std::size_t i = 0; i < n; ++i)
    {
        // Kolumna 1 (T)
        if (i < test_T.size() && std::isfinite(test_T[i]))
        {
            double v = test_T[i];
            out << v;
        }
        // Separator
        out << sep;

        // Kolumna 2 (T2)
        if (i < test_T2.size() && std::isfinite(test_T2[i]))
        {
            double v = test_T2[i];
            out << v;
        }

        out << '\n';
    }

    out.flush();
    return out.good();
}

int test = 0;
std::string file = "kryzys.csv";           // plik wejsciowy
std::string write_file = "kryzys_out.csv"; // plik wyjsciowy
// int windows_number = 2 * 5 * 5 * 2*2;
int windows_number = 10; // ile okien symulacji ma przejsc

int ktore_odbicie = 0;
float T2 = 0;

std::vector<float> time_T2;
std::vector<float> time_T2_prev;

static constexpr float R0_ATTEN = 0.0001f; // [m] � near-field cap (np. 5 cm)

// globalne
static uint32_t gFrameId = 0;
static float gAvgEdgeLen = 0.0f;                     // siatka
static int gBlindRings = 9 * 3 * 9 * 27 * 9 * 9 * 9; // 3 tr�jk�ty
static float gBlindRadius = 0.0f;                    // = gBlindRings * gAvgEdgeLen

static constexpr float SOUND_V = 1440.0f / 1.0f; // C++
bool serio_first = false;
bool rewind_punkt = false;

static size_t gWinIdx = 0; // kt�ry 5 ms segment aktualnie nadajemy

// WAZNE DANE
float radius = 0.02f;
float src_radius = 0.02f;
float time_passed = 0.0f;
float window_ms = 1.0f;
// dane z csv
std::vector<double> tSec; // czas w sekundach (po przeskalowaniu)
std::vector<float> y;     // warto�ci pr�bek (2. kolumna)

// do podzialu sfery na klatki
static size_t gRefineCursor = 0;                        // gdzie sko�czyli�my ostatnio
static std::unordered_map<uint64_t, int> gEdgeMidCache; // edge -> midpoint

const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

glm::vec3 cameraPos = glm::vec3(0.0f, 0.0f, 3.0f);
glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);

bool firstMouse = true;
bool mousePressed = false;
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
float yaw = -90.0f;
float pitch = 0.0f;
float fov = 45.0f;
float deltaTime = 0.0f;
float lastFrame = 0.0f;

bool simulate = false;

static bool gMeshDirty = true; // do odbudowania buforow

std::vector<Triangle> triangles;
std::vector<Triangle> microphone;
std::vector<Triangle> source_tri;
 

static std::vector<WindowPacket> gWinPackets;

// fala mikrofon i zrodlo
static MeshGL gWaveGL, gMicGL, gSrcGL;

// do FPS
int frameCount = 0;
double previousTime = 0.0;
double fps = 0.0;

bool first = true;
bool doKill = false;


static std::vector<MicSample> gMicEvents;
std::vector<float> winMean; // �rednie z okien X ms






source Source;


Cuboid_dimensions Cube{
    20.0f, 6.0f, 15.0f, // width, height, depth
    0.0f, 0.0f, 0.0f    // x/y/z offset
};

Cuboid_dimensions Obstacle{
    0.3f, 0.7f, 0.7f,   // width, height, depth
    500.0f, 1.0f, 50.0f // x/y/z offset
};


Microphone Mic;

// #include <cstdlib> // wymagane dla exit()
void framebuffer_size_callback(GLFWwindow *window, int width, int height);
void mouse_callback(GLFWwindow *window, double xpos, double ypos);
void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);
void processInput(GLFWwindow *window);
void renderScene();
// void drawCuboid(float width, float height, float depth);
void drawCuboidTransparentSorted(struct Cuboid_dimensions temp_Cube);
int printOversizedTriangles(float maxArea);
// Ustawia w�z�y w pozycji �r�d�a i nadaje im pr�dko�ci/kierunki startowe.
void killAllNodes();
void buildBuffersFor(const std::vector<node> &verts, const std::vector<Triangle> &tris, MeshGL &m, bool dynamic = true);
void updatePositionsFor(const std::vector<node> &verts, MeshGL &m);
void drawMesh(const MeshGL &m, const glm::vec3 &offset = glm::vec3(0), const glm::vec4 &fill = glm::vec4(1.0f, 0.5f, 0.0f, 1.0f), const glm::vec4 &wire = glm::vec4(0.0f, 0.0f, 0.0f, 0.6f), float wireWidth = 0.2f);
static void buildIcosphereNodesTris(float radius, int subdiv, std::vector<node> &out_nodes, std::vector<Triangle> &out_tris);
// Zapis do pliku po symulacji
// int removeSlowNodes(float minSpeed);

static inline float q5(float t)
{
    // double w srodku zmniejsza blad
    double buf = 1 / dt;
    return (float)(std::llround((double)t * buf) / buf);
}

static inline float q5_inter(float t)
{
    // double w srodku zmniejsza blad
    double buf = 1 / inv_fp;
    return (float)(std::llround((double)t * buf) / buf);
}

static float computeAvgEdgeLen(const std::vector<glm::vec3> &V,
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

// do separatora
static inline char detect_sep(const std::string &line)
{
    return (line.find(';') != std::string::npos) ? ';' : ',';
}

inline void logMicHit(float t_dop, float value)
{
    if (t_dop < 0.0f)
        t_dop = 0.0f; // na wszelki wypadek
    // std::lock_guard<std::mutex> lk(gMicMtx);
    gMicEvents.push_back({t_dop, value});
}

inline void resetMicEvents() { gMicEvents.clear(); }

bool writeMicCsv(const std::string &path = write_file)
{
    if (gMicEvents.empty())
        return false;

    float t_end = gMicEvents.back().t;
    for (float t = 0; t <= t_end; t += inv_fp)
    {
        gMicEvents.push_back({q5_inter(t), 0});
    }

    // posortuj po czasie
    std::sort(gMicEvents.begin(), gMicEvents.end(),
              [](const MicSample &a, const MicSample &b)
              { return a.t < b.t; });

    // scal duplikaty czas�w (dok�adnie r�wne) przez sumowanie warto�ci
    std::vector<MicSample> merged;
    merged.reserve(gMicEvents.size());
    for (const auto &s : gMicEvents)
    {
        if (!merged.empty() && merged.back().t == s.t)
        {
            merged.back().value += s.value;
        }
        else
        {
            merged.push_back(s);
        }
        // merged.push_back(s);
    }

    // zapisz do csv
    std::ofstream f(path, std::ios::out | std::ios::trunc);
    if (!f)
        return false;

    f.setf(std::ios::fixed);
    f << "time,value\n";
    f << std::setprecision(9);
    for (const auto &s : merged)
    {
        f << s.t << ",";
        f << std::setprecision(9) << s.value << "\n";
        f << std::setprecision(9); // wr�� do prec dla czasu
    }
    return true;
}

// odczytywanie z csv czas - wartosc
bool loadCsvSimple(const std::string &path,
                   char sep = ',', // separator
                   bool hasHeader = true,
                   double timeScale = 1.0) // 1.0 gdy time_s w s; 0.001 gdy w ms
{
    tSec.clear();
    y.clear();
    winMean.clear();

    std::ifstream f(path);
    if (!f)
        return false;

    std::string line;
    if (hasHeader)
        std::getline(f, line);
    std::cout << line << std::endl;

    while (std::getline(f, line))
    {
        if (line.empty())
            continue;
        size_t pos = line.find(sep);
        if (pos == std::string::npos)
            continue;

        double t = std::stod(line.substr(0, pos)) * timeScale;
        double v = std::stod(line.substr(pos + 1));
        tSec.push_back(t);
        y.push_back((float)v);
    }
    if (tSec.empty())
        return false;

    // Stale 5 ms
    const double win_s = window_ms / 1000; // 5 ms w sekundach
    const double t0 = tSec.front();
    const double tEnd = tSec.back();
    const size_t nWin = (size_t)std::floor((tEnd - t0) / win_s) + 1;

    // Przygotuj pakiety i winMean jako amplitude
    gWinPackets.clear();
    gWinPackets.resize(nWin);
    winMean.assign(nWin, 0.0f);
    std::vector<float> maxAbs(nWin, 0.0f);

    // Zainicjuj tEmit dla ka�dego okna
    for (size_t k = 0; k < nWin; ++k)
    {
        gWinPackets[k].tEmit = float(t0 + k * win_s);
    }

    // Rozdziel pr�bki do okien
    for (size_t i = 0; i < tSec.size(); ++i)
    {
        size_t k = (size_t)((tSec[i] - t0) / win_s);
        if (k >= nWin)
            k = nWin - 1;

        float tRel = float(tSec[i] - (t0 + k * win_s)); // czas wzgl startu okna
        gWinPackets[k].times.push_back(tRel);
        gWinPackets[k].values.push_back(y[i]);

        float a = std::fabs(y[i]);
        if (a > maxAbs[k])
            maxAbs[k] = a;
    }

    // Ustal amplitude i dla zgodno�ci wpisz j� te� do winMean
    for (size_t k = 0; k < nWin; ++k)
    {
        gWinPackets[k].amplitude = maxAbs[k]; // max(y)
        winMean[k] = maxAbs[k];
    }
    return true;
}

// �rednia okna odpowiadaj�cego czasowi t [s]
float getAtTime(double t_sec)
{
    if (!gWinPackets.empty())
    {
        const double win_s = window_ms / 1000; // 5 ms
        size_t k = (size_t)std::floor((t_sec + 1e-9) / win_s);
        if (k >= gWinPackets.size())
            return 0.0f;
        return gWinPackets[k].amplitude; // amplituda okna
    }
    // Fallback do starego
    if (winMean.empty())
        return 0.0f;
    const double idx = (t_sec * 1000.0 + 1e-3) / double(window_ms);
    size_t k = (size_t)std::floor(idx);
    if (k >= winMean.size())
        return 0.0f;
    return winMean[k];
}

static bool beginNextWindow()
{
    if (gWinIdx >= gWinPackets.size() - 1)
        return false;
    // if(gWinPackets[gWinIdx].times.size() < 2) return false;
    gWinIdx++;
    if (gWinPackets[gWinIdx].times.size() < 2)
        return false;
    first = true; // nowa fala
    return true;
}

static void retime_echo(std::vector<std::pair<float, float>> &buf, float newEnd, float prevE, float currE)
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

    float invAmpPrev = gWinPackets[gWinIdx - 1].amplitude / prevE;
    float PrevAmp = prevE / gWinPackets[gWinIdx - 1].amplitude;
    float newAmpS = currE / gWinPackets[gWinIdx].amplitude;
    newAmpS = (PrevAmp - newAmpS) / buf.size();
    int i = 0;
    for (auto &p : buf)
    {
        p.second = p.second * invAmpPrev;
        p.second = p.second * (PrevAmp - newAmpS * i);
        i++;
    }
}

// interpolacja liniowa  logMicHit na krok dt + kwantyzacja q5
static float flush_echo_interpolated(const std::vector<std::pair<float, float>> &buf,
                                     float TS_native, bool quantize = true)
{
    if (buf.empty())
        return 0;
    if (buf.size() == 1)
    {
        const float t0 = quantize ? q5_inter(buf[0].first) : buf[0].first;
        logMicHit(t0, buf[0].second);
        return 0;
    }

    const float EPS = TS_native * 1e-3;
    const float STEP = inv_fp;       // rozdzielczo�� siatki (np. 1e-5)
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
        tg = q5_inter(tg);
        while (k + 1 < buf.size() && buf[k + 1].first < tg - EPS)
            ++k;
        if (k + 1 >= buf.size())
        {
            const float v_last = buf.back().second;
            const float t_raw = buf.back().first;
            float t_q = quantize ? q5_inter(t_raw) : t_raw;
            t_q = q5_inter(snap_guard(t_q));

            logMicHit(t_q, v_last);
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
        float t_prop = quantize ? q5_inter(tg) : tg;

        float t = q5_inter(snap_guard(t_prop));

        logMicHit(t, v);
        poprzednie_t = t;
    }
    return new_T;
}

int pruneSlowNodes(float minSpeed);

static float dist2PointSegment(const glm::vec3 &p,
                               const glm::vec3 &a,
                               const glm::vec3 &b);
static float dist2PointTriangle(const glm::vec3 &p,
                                const glm::vec3 &a,
                                const glm::vec3 &b,
                                const glm::vec3 &c);
static inline bool touchesMicrophone(int nodeIndex);
// unikalny klucz kraw�dzi (mniejszy indeks najpierw)
static inline uint64_t edge_key(int a, int b)
{
    if (a > b)
        std::swap(a, b);
    return (uint64_t(a) << 32) | uint32_t(b);
}

// rzut na sfer� (promie� r)
static inline glm::vec3 normalize_to_radius(const glm::vec3 &p, float r)
{
    float len = glm::length(p);
    if (len == 0.0f)
        return glm::vec3(r, 0, 0);
    return (p / len) * r;
}

// zwraca indeks �rodkowego wierzcho�ka (bez duplikatow)
static int midpoint_index(int i0, int i1,
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



float calculateTriangleArea(int a, int b, int c)
{
    glm::vec3 ab = nodes[b].position - nodes[a].position;
    glm::vec3 ac = nodes[c].position - nodes[a].position;
    glm::vec3 cross = glm::cross(ab, ac);
    return 0.5f * glm::length(cross); // funkcja do obliczania pola
}

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

// POMOCNICZE
struct Edge
{
    int a, b;
}; // a<b
static inline Edge make_edge(int u, int v)
{
    if (u > v)
        std::swap(u, v);
    return {u, v};
}
static inline bool edge_less(const Edge &x, const Edge &y)
{
    return (x.a < y.a) || (x.a == y.a && x.b < y.b);
}
static inline bool edge_eq(const Edge &x, const Edge &y)
{
    return x.a == y.a && x.b == y.b;
}

static int midpoint_index_nodes_cached(int i0, int i1)
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
static int midpoint_index_nodes(int i0, int i1,
                                std::unordered_map<uint64_t, int> &cache)
{
    uint64_t key = edge_key(i0, i1);
    auto it = cache.find(key);
    if (it != cache.end())
        return it->second;

    int idx = addMidpoint(i0, i1); // �rednia pozycji i pr�dko�ci
    cache.emplace(key, idx);
    return idx;
}
// WIELOWATKOWOSC
bool refineIcosahedron_chunked_mt(float maxArea,
                                  size_t triBudget, // ile tr�jk�t�w obrabiamy w tej klatce
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

bool refineIcosahedron_chunked(float maxArea,
                               size_t triBudget /* ile tr�jk�t�w obrabiamy na wywo�anie */)
{
    if (triangles.empty() || triBudget == 0)
        return false;

    // Je�li od ostatniego razu siatka zosta�a zresetowana � wyzeruj stan
    if (gRefineCursor > triangles.size())
    {
        gRefineCursor = 0;
        gEdgeMidCache.clear();
    }

    // Sta�e i pr�g bez sqrt: |cross|^2 > (2*maxArea)^2
    const float cuboidHalfWidth = Cube.width * 0.5f;
    const float cuboidHalfHeight = Cube.height * 0.5f;
    const float cuboidHalfDepth = Cube.depth * 0.5f;
    const float safetyMargin = 0.2f;
    const float area2_threshold = 4.0f * maxArea * maxArea;

    // tylko trojkaty, ktore istnialy
    const size_t size0 = triangles.size();
    size_t start = (gRefineCursor < size0 ? gRefineCursor : 0);
    size_t end = std::min(start + triBudget, size0);

    bool changed = false;

    for (size_t i = start; i < end; ++i)
    {
        // Triangle t = triangles[i];
        int a = triangles[i].indices[0], b = triangles[i].indices[1], c = triangles[i].indices[2];

        const glm::vec3 &pa = nodes[a].position;
        const glm::vec3 &pb = nodes[b].position;
        const glm::vec3 &pc = nodes[c].position;

        // sprawdz, czy blisko sciany. jak tak - nie dziel
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

        if (!nearWall)
        {
            // Pole bez sqrt: |cross|^2
            const glm::vec3 ab = pb - pa;
            const glm::vec3 ac = pc - pa;
            const glm::vec3 cr = glm::cross(ab, ac);
            const float cr2 = glm::dot(cr, cr);

            if (cr2 > area2_threshold)
            {
                // Zast�p tr�jk�t i do�� 3 nowe
                const int ab_i = midpoint_index_nodes_cached(a, b);
                const int bc_i = midpoint_index_nodes_cached(b, c);
                const int ca_i = midpoint_index_nodes_cached(c, a);

                triangles[i] = {{a, ab_i, ca_i}};       // zamiana bie��cego
                triangles.push_back({{b, bc_i, ab_i}}); // 3 nowe
                triangles.push_back({{c, ca_i, bc_i}});
                triangles.push_back({{ab_i, bc_i, ca_i}});

                changed = true;
            }
        }
    }

    // Przesu� kursor; po doj�ciu do ko�ca partii
    gRefineCursor = end;
    if (gRefineCursor >= size0)
        gRefineCursor = 0;

    return changed;
}

void calculateFPS()
{
    double currentTime = glfwGetTime();
    frameCount++;

    if (currentTime - previousTime >= 1.0)
    {
        fps = frameCount / (currentTime - previousTime);
        std::cout << "FPS: " << fps << std::endl;

        frameCount = 0;
        previousTime = currentTime;
        printOversizedTriangles(0.05f);
    }
} // pokaz ile fps ma program

void mouse_button_callback(GLFWwindow *window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_RIGHT)
    {
        if (action == GLFW_PRESS)
        {
            mousePressed = true;
        }
        else if (action == GLFW_RELEASE)
        {
            mousePressed = false;
            firstMouse = true;
        }
    }
} // do obslugi myszki w opengl

void scroll_callback(GLFWwindow *window, double xoffset, double yoffset)
{
    // Zmiana odleglosci kamery za pomoca scrolla
    cameraPos += cameraFront * static_cast<float>(yoffset) * 0.5f;
}

static void glfwErrorCallback(int code, const char *desc) { fprintf(stderr, "GLFW[%d]: %s\n", code, desc); }

//---------------------
//--------MAIN---------
//---------------------
int main()
{
    glfwSetErrorCallback(glfwErrorCallback);
    if (!glfwInit())
        return -1;

    bool pokazano_dane = false; // testowe, do usuniecia

    GLFWwindow *window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Symulacja", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // Wczytaj plik csv (ze sciezki wyzej)
    if (!loadCsvSimple(file, ',', /*hasHeader=*/true))
    {
        std::cerr << "CSV: nie wczytano �adnych danych\n";
    }
    else
    {
        std::cout << "CSV: wczytano " << tSec.size()
                  << " wierszy" << std::endl;
    }
    // beginNextWindow(); // uruchamiamy pierwsze okno 5 ms
    // gAudio.window_ms = 5.0f;   // trzymamy 5 ms

    // to pod tym dodane do VBO
    // if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    //     std::cerr << "GLAD fail\n"; return -1;
    // }
    // std::cout << "GL " << GLVersion.major << "." << GLVersion.minor << "\n";
    // glEnable(GL_DEPTH_TEST);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

    // Rzeczy od ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext(); // inicjalizacja ImGui
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    ImGui::StyleColorsDark();                   // ciemny motyw imgui
    ImGui_ImplGlfw_InitForOpenGL(window, true); // setup
    ImGui_ImplOpenGL3_Init("#version 330");

    while (!glfwWindowShouldClose(window))
    {
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        processInput(window);

        glClearColor(0.5f, 0.5f, 0.5f, 0.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 projection = glm::perspective(glm::radians(fov), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);

        glMatrixMode(GL_PROJECTION);
        glLoadMatrixf(&projection[0][0]);

        glMatrixMode(GL_MODELVIEW);
        glLoadMatrixf(&view[0][0]);

        // Oblicz FPS
        // calculateFPS();
        // if (true)
        //{
        renderScene();
        //}

        glfwPollEvents();

        //---IMGUI---
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGui::SetWindowSize(ImVec2(200, 200));

        ImGui::Begin("Start/Stop");
        if (ImGui::Button("Start"))
        {
            simulate = true;
        }
        if (ImGui::Button("Stop"))
        {
            simulate = false;
        }
        ImGui::End();

        ImGui::Begin("Parametry");
        ImGui::Text("Mikrofon");
        ImGui::InputFloat("Mic x", &Mic.mic_x, 0.1f, Mic.mic_x);
        ImGui::InputFloat("Mic y", &Mic.mic_y, 0.1f, Mic.mic_y);
        ImGui::InputFloat("Mic z", &Mic.mic_z, 0.1f, Mic.mic_z);
        ImGui::InputFloat("Mic_vel x", &Mic.mic_velocity.x, 5.0f, Mic.mic_velocity.x);
        ImGui::InputFloat("Mic_vel y", &Mic.mic_velocity.y, 5.0f, Mic.mic_velocity.y);
        ImGui::InputFloat("Mic_vel z", &Mic.mic_velocity.z, 5.0f, Mic.mic_velocity.z);
        ImGui::Text("Zrodlo");
        ImGui::InputFloat("Src x", &Source.src_x, 0.1f, Source.src_x);
        ImGui::InputFloat("Src y", &Source.src_y, 0.1f, Source.src_y);
        ImGui::InputFloat("Src z", &Source.src_z, 0.1f, Source.src_z);
        ImGui::InputFloat("Src_vel x", &Source.velocity.x, 5.0f, Source.velocity.x);
        ImGui::InputFloat("Src_vel y", &Source.velocity.y, 5.0f, Source.velocity.y);
        ImGui::InputFloat("Src_vel z", &Source.velocity.z, 5.0f, Source.velocity.z);
        ImGui::Text("Basen");
        ImGui::InputFloat("Cube width", &Cube.width, 2.0f, Cube.width);
        ImGui::InputFloat("Cube height", &Cube.height, 2.0f, Cube.height);
        ImGui::InputFloat("Cube depth", &Cube.depth, 2.0f, Cube.depth);
        ImGui::InputFloat("Cube x_offset", &Cube.x_offset, 2.0f, Cube.x_offset);
        ImGui::InputFloat("Cube y_offset", &Cube.y_offset, 2.0f, Cube.y_offset);
        ImGui::InputFloat("Cube z_offset", &Cube.z_offset, 2.0f, Cube.z_offset);
        ImGui::End();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        //---Koniec IMGUI----

        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwTerminate();
    return 0;
}

void processInput(GLFWwindow *window)
{
    float cameraSpeed = 2.5f * deltaTime;

    // Zmiana wysokosci kamery za pomoca klawiszy W i S
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        cameraPos.y += cameraSpeed;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        cameraPos.y -= cameraSpeed;

    // Poruszanie kamera w plaszczyznie XZ za pomoca klawiszy A i D
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
}

void mouse_callback(GLFWwindow *window, double xpos, double ypos)
{
    if (!mousePressed)
        return;

    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos;
    lastX = xpos;
    lastY = ypos;

    float sensitivity = 0.1f;
    xoffset *= sensitivity;
    yoffset *= sensitivity;

    yaw += xoffset;
    pitch += yoffset;

    if (pitch > 89.0f)
        pitch = 89.0f;
    if (pitch < -89.0f)
        pitch = -89.0f;

    glm::vec3 front;
    front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    front.y = sin(glm::radians(pitch));
    front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    cameraFront = glm::normalize(front);
} // tez myszka w opengl

void framebuffer_size_callback(GLFWwindow *window, int width, int height)
{
    glViewport(0, 0, width, height);
}

void renderScene()
{
    // glEnable(GL_BLEND);
    // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // static std::vector<Triangle> triangles;
    if (first && !rewind_punkt)
    {
        doKill = false;
        // glfwSetTime(0.0);  // wyzeruj stoper
        time_passed = 0.0f;
        // microphone
        Mic.mic_x = Mic.rewind_point.x;
        Mic.mic_y = Mic.rewind_point.y;
        Mic.mic_z = Mic.rewind_point.z;
        Mic.mic_velocity = Mic.rewind_vel;
        // source
        Source.src_x = Source.rewind_point.x;
        Source.src_y = Source.rewind_point.y;
        Source.src_z = Source.rewind_point.z;
        Source.velocity = Source.rewind_vel;

        rewind_punkt = true;

        nodes.clear();
        triangles.clear();

        if (serio_first)
        {
            serio_first = false;
            // Zbuduj geometri� dla mikrofonu i �r�d�a
            const int SUBDIV_RIGID = 2;

            mic_nodes.clear();
            microphone.clear();
            buildIcosphereNodesTris(mic_radius, SUBDIV_RIGID, mic_nodes, microphone);

            src_nodes.clear();
            source_tri.clear();
            buildIcosphereNodesTris(src_radius, SUBDIV_RIGID, src_nodes, source_tri);

            // Bufory GPU :
            buildBuffersFor(mic_nodes, microphone, gMicGL, /*dynamic=*/false);
            buildBuffersFor(src_nodes, source_tri, gSrcGL, /*dynamic=*/false);
        }
        // gestosc - ka�dy poziom �4 liczba tr�jk�t�w
        constexpr int SUBDIV = 2; // 2 optymalnie, wiecej laguje
        // 12 wierzcho�k�w  na sferze o promieniu 'radius'
        const float t = (1.0f + std::sqrt(5.0f)) * 0.5f; // z�ota proporcja
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
            v *= radius; // na promie� 'radius'

        // 20 �cian
        std::vector<glm::ivec3> faces = {
            {0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11}, {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8}, {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9}, {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}};
        /*
        std::vector<glm::ivec3> faces = {
            {0,11,5}, {0,5,1}, {0,1,7}, {0,7,10}, {0,10,11},
            {1,5,9},  {5,11,4}, {11,10,2}, {10,7,6}, {7,1,8},
            {3,9,4},  {3,4,2},  {3,2,6},  {3,6,8},  {3,8,9},
            {4,9,5},  {2,4,11}, {6,2,10}
        };*/

        // Podzial
        for (int s = 0; s < SUBDIV; ++s)
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

                // 4 nowe tr�jk�ty
                new_faces.push_back({i0, a, c});
                new_faces.push_back({i1, b, a});
                new_faces.push_back({i2, c, b});
                new_faces.push_back({a, b, c});
            }
            faces.swap(new_faces);
        }

        nodes.reserve(verts.size());
        glm::vec3 buf2 = glm::vec3(Source.src_x, Source.src_y, Source.src_z);
        glm::vec3 gSourceVel = Source.velocity;
        // const float audioE = getAtTime((gWinIdx)*window_ms / 1000.0f);
        const auto &wp = gWinPackets[gWinIdx]; // aktualne 5 ms
        int i = 0;
        for (const auto &p : verts)
        {
            node nd;
            nd.position = p;
            // nd.nEmit = glm::normalize(p);
            nd.velocity = glm::normalize(p) * SOUND_V; // sta�a pr�dko�� fali w wodzie

            nd.position += buf2;
            nd.doppler = glm::length(nd.velocity) + glm::length(Source.velocity);
            // nd.srcVel = gSourceVel;
            // nd.energy = audioE;
            // nd.tEmit = (gWinIdx)*gAudio.window_ms / 1000.0f; // zapisz czas emisji (sim-time)
            // nd.tEmit = (gWinIdx)*window_ms / 1000.0f;

            nd.energy = wp.amplitude; // amplituda okna
            nd.tEmit = wp.tEmit;      // czas emisji okna (sek)
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
        // buildSphereBuffers(/*dynamic=*/true);
        buildBuffersFor(nodes, triangles, gWaveGL, /*dynamic=*/true);

        // po zbudowaniu dwudziestoscianu
        gAvgEdgeLen = computeAvgEdgeLen(verts, faces);
        gBlindRadius = gBlindRings * gAvgEdgeLen;
        // gBlindRadius = 2.5f;

        first = false;
    }

    static int frameCount = 0;
    if ((frameCount % 8) == 0)
    {
        size_t budget = std::min<size_t>(4000, std::max<size_t>(1, triangles.size() / 10));
        budget = triangles.size(); // TO DO: MOZNA ZMIENIC NA WIEKSZE (W SENSIE ZMIENIC np. 4 -> 2)
        int threads = std::max(1u, std::thread::hardware_concurrency());

        if (refineIcosahedron_chunked_mt(0.05f, budget, threads))
        {
            gMeshDirty = true; // przebuduj bufory tylko gdy zasz�a zmiana
        }
    }
    frameCount++;
    if (simulate)
    {
        // updatePhysics(dt, Cube, Obstacle);
        // usuwanie zuzytych nodes
        pruneSlowNodes(/*minEnergy=*/getAtTime((gWinIdx)*window_ms / 1000.0f));
        updatePhysics(dt, Cube, Obstacle);
    }

    // odbuduj bufory tylko gdy trzeba
    if (gMeshDirty)
    {
        // buildSphereBuffers(/*dynamic=*/true);
        buildBuffersFor(nodes, triangles, gWaveGL, /*dynamic=*/true);
        gMeshDirty = false;
    }
    else
    {
        // w przeciwnym razie tylko podmie� pozycje
        // updateSpherePositions();
        updatePositionsFor(nodes, gWaveGL);
        updatePositionsFor(mic_nodes, gMicGL);
        updatePositionsFor(src_nodes, gSrcGL);
    }

    // 3) rysuj TYLKO z VBO/IBO
    // drawSphereWithBuffers();

    // kulka
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST);
    // drawSphereWithBuffers();
    // glEnable(GL_DEPTH_TEST);
    //  FALA (bez offsetu � pozycje absolutne)
    drawMesh(gWaveGL);

    glEnable(GL_DEPTH_TEST);
    glm::vec3 micOffset = glm::vec3(Mic.mic_x, Mic.mic_y, Mic.mic_z);
    drawMesh(gMicGL, micOffset, /*fill*/ {1.0f, 0.4f, 0.8f, 0.5f});
    glm::vec3 srcOffset = glm::vec3(Source.src_x, Source.src_y, Source.src_z);
    drawMesh(gSrcGL, srcOffset, /*fill*/ {1.0f, 1.0f, 1.0f, 0.5f});
    // drawMicrophone();
    // drawSource();
    // PRZESZKODA

    glColor4f(0.2f, 0.5f, 0.2f, 0.7f);
    drawCuboidTransparentSorted(Obstacle);
    // glEnable(GL_DEPTH_TEST);
    //  Basen
    // glDisable(GL_DEPTH_TEST);
    glColor4f(0.0f, 0.0f, 1.0f, 0.1f);
    drawCuboidTransparentSorted(Cube);
} // do wyswietlania symulacji

int printOversizedTriangles(float maxArea)
{
    // te same sta�e co w refine
    const float cuboidHalfWidth = Cube.width * 0.5f;
    const float cuboidHalfHeight = Cube.height * 0.5f;
    const float cuboidHalfDepth = Cube.depth * 0.5f;
    const float safetyMargin = 0.2f;

    // por�wnujemy |cross|^2 > (2*maxArea)^2
    const float thr2 = 4.0f * maxArea * maxArea;

    int count = 0;
    for (const auto &t : triangles)
    {
        int a = t.indices[0], b = t.indices[1], c = t.indices[2];
        const glm::vec3 &pa = nodes[a].position;
        const glm::vec3 &pb = nodes[b].position;
        const glm::vec3 &pc = nodes[c].position;

        // pomijamy tr�jk�ty blisko �ciany (jak w refine)
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

        // pole bez sqrt: |(pb-pa) x (pc-pa)|^2
        const glm::vec3 cr = glm::cross(pb - pa, pc - pa);
        const float cr2 = glm::dot(cr, cr);

        if (cr2 > thr2)
            ++count;
    }
    float procent = (float)count / (float)triangles.size() * 100.0f;
    std::cout << "Oversized (splittable) triangles: "
              << triangles.size() << "\n";
    return count;
}

static float dist2PointSegment(const glm::vec3 &p,
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

static float dist2PointTriangle(const glm::vec3 &p,
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

static inline bool touchesMicrophone(int nodeIndex)
{
    const glm::vec3 center(Mic.mic_x, Mic.mic_y, Mic.mic_z);
    const float r = mic_radius;
    const float r2 = r * r;

    const glm::vec3 &p = nodes[nodeIndex].position;
    glm::vec3 d = p - center;
    float dist2 = glm::dot(d, d);

    // if (dist2 <= r2) return true;

    float maxExtra = gAvgEdgeLen * 1.5f;
    float farR2 = (maxExtra) * (maxExtra);
    if (dist2 > farR2)
    {
        return false;
    }

    for (const auto &tri : triangles)
    {
        if (tri.indices[0] != nodeIndex &&
            tri.indices[1] != nodeIndex &&
            tri.indices[2] != nodeIndex)
            continue;

        const glm::vec3 &a = nodes[tri.indices[0]].position;
        const glm::vec3 &b = nodes[tri.indices[1]].position;
        const glm::vec3 &c = nodes[tri.indices[2]].position;

        float d2Tri = dist2PointTriangle(center, a, b, c);
        if (d2Tri <= (1440 * dt) * 1440 * dt)
        {
            int siemka2 = 5;
        }
        d2Tri = dist2PointTriangle(center, a, b, c);
        if (d2Tri <= (1440 * dt) * 1440 * dt)
        {
            return true;
        }
    }

    return false;
} // czy fala dotyka mikrofonu

static inline bool touchesMicrophonePoint(int nodeIndex)
{
    const glm::vec3 center(Mic.mic_x, Mic.mic_y, Mic.mic_z);
    const glm::vec3 &pNode = nodes[nodeIndex].position;

    float maxExtra = gAvgEdgeLen * 1.5f;
    float farR2 = 1440 * dt * 10;
    float buf = glm::length(pNode - center);
    if (buf > farR2)
    {
        return false;
    }

    const float soundSpeed = 1440.0f;
    const float maxAdvance = (soundSpeed * dt) * 1.5;

    // kierunek propagacji z pr�dko�ci noda
    glm::vec3 n_prop = glm::normalize(nodes[nodeIndex].velocity);

    for (const auto &tri : triangles)
    {
        if (tri.indices[0] != nodeIndex &&
            tri.indices[1] != nodeIndex &&
            tri.indices[2] != nodeIndex)
            continue;

        const glm::vec3 &a = nodes[tri.indices[0]].position;
        const glm::vec3 &b = nodes[tri.indices[1]].position;
        const glm::vec3 &c = nodes[tri.indices[2]].position;

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

int pruneSlowNodes(float minEnergy)
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
        glm::vec3 buf = nodes[i].position - glm::vec3(Mic.mic_x, Mic.mic_y, Mic.mic_z);
        bool hitMic = false;
        hitMic = touchesMicrophonePoint((int)i);

        if (hitMic)
        {
            float win_s = window_ms / 1000.0f - (gWinPackets[gWinIdx].times[1] - gWinPackets[gWinIdx].times[0]); // 0.005s w teorii w praktyce lekko mniej niz 0.005s zeby nie brac dwa razy tej samej probki
            float T = gWinPackets[gWinIdx].tEmit + time_passed;                                                  // czas przyjscia pocz�tku okna
            const int wid = gWinIdx;
            // Je�eli ten node jest czasowo st�umiony nie loguj do CSV
            if (T <= nodes[i].suppressUntilT)
            {
                continue;
            }

            ktore_odbicie++;
            // DOPPLER (rozci�gni�cie/�ci�ni�cie osi czasu okna)
            const glm::vec3 n_hat = glm::normalize(nodes[i].velocity);
            const float v_mic_proj = glm::dot(Mic.mic_velocity, n_hat);
            float vel_test = glm::length(nodes[i].velocity);
            // CECHY echa
            if (r > 1)
                nodes[i].energy = p / (r);
            EchoHit curHit;
            curHit.T = T; //  T z bie��cej ramki
            curHit.energy = std::fabs(nodes[i].energy);
            curHit.bounces = nodes[i].bounces;
            curHit.doppler = nodes[i].doppler;
            curHit.dir_in = glm::normalize(nodes[i].velocity);
            int test = -1;

            const glm::vec3 micC(Mic.mic_x, Mic.mic_y, Mic.mic_z);

            // Proba dopasowania okna do poprzedniego echa
            if (gWinIdx != 0 && !gPrevHits.empty())
            {
                int match = pick_prev_hit(curHit, gPrevHits, gPrevTaken, dt);
                matches.push_back(match);
                test = match;
                if (match >= 0)
                {
                    // start tego okna jest koncem poprzedniego
                    //   przeskaluj ca�y bufor poprzedniego okna: stare T2_prev -> nowe T
                    if (match < (int)wektorPrev.size() && !wektorPrev[match].empty())
                    {
                        retime_echo(wektorPrev[match], /*newEnd=*/T, gPrevHits[match].energy, nodes[i].energy);
                        // interpolacja -> Mic csv
                        float new_T = flush_echo_interpolated(wektorPrev[match], /*TS_native=*/inv_fp, /*quantize=*/true);
                        wektorPrev[match].clear();
                    }
                }
            }
            // T2 liczymy dopiero po korekcie czasu T
            T2 = T + win_s * nodes[i].doppler * ((1) / (vel_test - v_mic_proj));
            curHit.T2 = T2;
            gCurrHits.push_back(curHit);

            float scale = (T2 - T) / win_s;
            if (scale <= 0.0f)
                scale = 1e-4f;

            // Logowanie pe�nego okna 5 ms (czas + warto�ci)
            if (wid >= 0 && wid < (int)gWinPackets.size())
            {
                const auto &wp = gWinPackets[wid];
                const float Awin = (wp.amplitude != 0.0f) ? wp.amplitude : 1e-12f;
                // --- sta�e do amplitudy
                const float scaleAmp = std::fabs(nodes[i].energy / Awin);

                // BUFOR: (t_abs, val) dla wszystkich pr�bek okna
                std::vector<std::pair<float, float>> tmp;
                tmp.reserve(wp.times.size());

                for (size_t j = 0; j < wp.times.size(); ++j)
                {
                    // czas absolutny po Dopplerze (bez kwantyzacji)
                    float t_abs = T + scale * wp.times[j];

                    float val = (wp.values[j]) * scaleAmp;
                    if (gWinIdx == gWinPackets.size() - 1)
                        val = val;

                    tmp.emplace_back(t_abs, val);
                }
                wektor.push_back(tmp);
                wektorCurr.push_back(tmp);

                // posortuj po czasie (powinno ju� by� rosn�co przy scale>0)
                std::sort(tmp.begin(), tmp.end(), [](const auto &a, const auto &b)
                          { return a.first < b.first; });

                // INTERPOLACJA do kroku tego okna
                if (gWinIdx == windows_number - 1)
                {
                    // int siema6 = 2;
                    flush_echo_interpolated(tmp, /*TS_native=*/inv_fp, /*quantize=*/true);
                }
            }

            // O�LEPIANIE WY��CZNIE TEGO SAMEGO CZO�A - zeby nie odczytac 2 razy tego samego
            // Wersja sta�a:
            const float blind_until = T + 0.00005f;

            // parametry filtr�w klastra
            const glm::vec3 center = nodes[i].position; // pozycja trafionego noda
            // const glm::vec3 micC = glm::vec3(Mic.mic_x, Mic.mic_y, Mic.mic_z);
            const float cosMax = std::cos(glm::radians(20.0f));          // max r�nica kierunku
            const float s_i = glm::dot(micC - nodes[i].position, n_hat); // proxy czasu doj�cia

            for (size_t j = 0; j < nodes.size(); ++j)
            {
                if (j == i)
                    continue;

                // To samo okno emisji (to samo czo�o czasowo)
                if (gWinIdx != wid)
                    continue;

                // Zgodny kierunek propagacji (wyklucz inne odbicia)
                const glm::vec3 nj_hat = glm::normalize(nodes[j].velocity);
                if (glm::dot(n_hat, nj_hat) < cosMax)
                    continue;

                // Node musi zbli�a� si� do mikrofonu
                const float s_j = glm::dot(micC - nodes[j].position, nj_hat);
                // if (s_j <= 0.0f) continue;

                // Zbli�ony przewidywany czas doj�cia
                if (std::fabs(s_j - s_i) > 1.5f * gBlindRadius)
                    continue;

                // Blisko w przestrzeni (ok. �3 tr�jk�ty�)
                if (glm::length(nodes[j].position - center) > gBlindRadius)
                    continue;

                // Ustaw t�umienie do blind_until � zar�wno w nodes[] i  kept[]
                if (remap[j] != -1)
                {
                    const int kj = remap[j];
                    kept[kj].suppressUntilT = std::max(kept[kj].suppressUntilT, blind_until);
                }
                else
                {
                    nodes[j].suppressUntilT = std::max(nodes[j].suppressUntilT, blind_until);
                }
            }

            // doKill = true;
            // only_one_read = true;   // jesli usuwa nodes po odczycie
            //  (opcjonalnie) debug:
            std::cout << "win=" << wid << "  T=" << T << " " << ktore_odbicie << " " << "T2=" << T2 << "\n";
            // continue;
        }

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
    gMeshDirty = true;

    if (doKill)
        killAllNodes(); // do testow
    // zgasniecie fali
    const bool windowDied = nodes.empty();

    if (windowDied)
    {
        ktore_odbicie = 0;
        time_T2_prev.swap(time_T2);

        // Domknij wszystkie wci�� wisz�ce odbicia z poprzedniej ramki
        for (auto &buf : wektorPrev)
        {
            if (!buf.empty())
            {
                flush_echo_interpolated(buf, /*TS_native=*/inv_fp, /*quantize=*/true);
                buf.clear();
            }
        }

        gPrevHits.swap(gCurrHits);

        wektorPrev.swap(wektorCurr);
        wektorCurr.clear();

        gCurrHits.clear();
        gPrevTaken.assign(gPrevHits.size(), 0); // reset znacznik�w

        time_T2.clear();
        if (!beginNextWindow() or gWinIdx == windows_number)
        {
            writeMicCsv(write_file);
            resetMicEvents();
        }
        // rewind_punkt = false;
    }

    return (int)(N - nodes.size());
}

void drawCuboidTransparentSorted(struct Cuboid_dimensions temp_Cube)
{
    float halfWidth = temp_Cube.width / 2.0f;
    float halfHeight = temp_Cube.height / 2.0f;
    float halfDepth = temp_Cube.depth / 2.0f;

    glm::vec3 vertices[] = {
        {-halfWidth + temp_Cube.x_offset, -halfHeight + temp_Cube.y_offset, halfDepth + temp_Cube.z_offset},
        {halfWidth + temp_Cube.x_offset, -halfHeight + temp_Cube.y_offset, halfDepth + temp_Cube.z_offset},
        {halfWidth + temp_Cube.x_offset, halfHeight + temp_Cube.y_offset, halfDepth + temp_Cube.z_offset},
        {-halfWidth + temp_Cube.x_offset, halfHeight + temp_Cube.y_offset, halfDepth + temp_Cube.z_offset},
        {-halfWidth + temp_Cube.x_offset, -halfHeight + temp_Cube.y_offset, -halfDepth + temp_Cube.z_offset},
        {halfWidth + temp_Cube.x_offset, -halfHeight + temp_Cube.y_offset, -halfDepth + temp_Cube.z_offset},
        {halfWidth + temp_Cube.x_offset, halfHeight + temp_Cube.y_offset, -halfDepth + temp_Cube.z_offset},
        {-halfWidth + temp_Cube.x_offset, halfHeight + temp_Cube.y_offset, -halfDepth + temp_Cube.z_offset}};

    struct Face
    {
        int indices[4];
        float distance;
    };

    Face faces[6] = {
        {{0, 1, 2, 3}, 0.0f}, // prz�d
        {{4, 5, 6, 7}, 0.0f}, // ty�
        {{0, 3, 7, 4}, 0.0f}, // lewo
        {{1, 2, 6, 5}, 0.0f}, // prawo
        {{0, 1, 5, 4}, 0.0f}, // d�
        {{2, 3, 7, 6}, 0.0f}  // g�ra
    };

    // Oblicz odleg�o�ci �cian od kamery
    for (int i = 0; i < 6; ++i)
    {
        glm::vec3 center(0.0f);
        for (int j = 0; j < 4; ++j)
        {
            center += vertices[faces[i].indices[j]];
        }
        center /= 4.0f;
        faces[i].distance = glm::length(center - cameraPos);
    }

    // Sortuj �ciany od najdalszej do najbli�szej
    for (int i = 0; i < 5; ++i)
    {
        for (int j = i + 1; j < 6; ++j)
        {
            if (faces[i].distance < faces[j].distance)
            {
                std::swap(faces[i], faces[j]);
            }
        }
    }

    // transparentne sciany
    glBegin(GL_QUADS);
    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            const glm::vec3 &v = vertices[faces[i].indices[j]];
            glVertex3f(v.x, v.y, v.z);
        }
    }
    glEnd();

    // KRAW�DZIE (drut)
    //  glColor4f(0.0f, 0.0f, 0.0f, 1.0f); // czarne, nieprzezroczyste
    glDisable(GL_DEPTH_TEST);
    glLineWidth(2.0f);

    static const int edges[12][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0}, // prz�d
        {4, 5},
        {5, 6},
        {6, 7},
        {7, 4}, // ty�
        {0, 4},
        {1, 5},
        {2, 6},
        {3, 7} // ��cz�ce
    };
    glColor4f(1.0f, 1.0f, 1.0f, 0.5f);
    glBegin(GL_LINES);
    for (int e = 0; e < 12; ++e)
    {
        const glm::vec3 &v0 = vertices[edges[e][0]];
        const glm::vec3 &v1 = vertices[edges[e][1]];
        glVertex3f(v0.x, v0.y, v0.z);
        glVertex3f(v1.x, v1.y, v1.z);
    }
    glEnd();
}

// Zabija wszystkie nody i tr�jk�ty � do debugu.
void killAllNodes()
{
    nodes.clear();
    triangles.clear();

    // wyczy�� stan mesh
    gRefineCursor = 0;
    gEdgeMidCache.clear();
    gMeshDirty = true;

    // gIndexCount = 0;
}

void buildBuffersFor(const std::vector<node> &verts,
                     const std::vector<Triangle> &tris,
                     MeshGL &m,
                     bool dynamic)
{
    // if (!GLAD_GL_VERSION_1_5) { std::cerr << "VBO niedost�pne (GL < 1.5)\n"; return; }
    m.dynamic = dynamic;

    if (!m.vbo)
        glGenBuffers(1, &m.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, m.vbo);

    const GLsizeiptr vbSize = (GLsizeiptr)(verts.size() * sizeof(node));
    const GLenum usage = dynamic ? GL_DYNAMIC_DRAW : GL_STATIC_DRAW;
    glBufferData(GL_ARRAY_BUFFER, vbSize, verts.empty() ? nullptr : (const void *)verts.data(), usage);

    if (!m.ibo)
        glGenBuffers(1, &m.ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m.ibo);

    const GLsizeiptr ibSize = (GLsizeiptr)(tris.size() * 3 * sizeof(unsigned int));
    const void *idxSrc = tris.empty() ? nullptr : (const void *)&tris[0].indices[0];
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, ibSize, idxSrc, GL_STATIC_DRAW);

    m.indexCount = (GLsizei)(tris.size() * 3);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
} // bufory

void updatePositionsFor(const std::vector<node> &verts, MeshGL &m)
{
    if (!m.vbo)
        return;
    glBindBuffer(GL_ARRAY_BUFFER, m.vbo);

    const GLsizeiptr vbSize = (GLsizeiptr)(verts.size() * sizeof(node));
    glBufferData(GL_ARRAY_BUFFER, vbSize, nullptr, GL_DYNAMIC_DRAW);
    if (!verts.empty())
        glBufferSubData(GL_ARRAY_BUFFER, 0, vbSize, (const void *)verts.data());

    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void drawMesh(const MeshGL &m,
              const glm::vec3 &offset,
              const glm::vec4 &fill,
              const glm::vec4 &wire,
              float wireWidth)
{
    if (!m.vbo || !m.ibo || m.indexCount == 0)
        return;

    glPushMatrix();
    if (offset.x != 0 || offset.y != 0 || offset.z != 0)
        glTranslatef(offset.x, offset.y, offset.z);

    glBindBuffer(GL_ARRAY_BUFFER, m.vbo);
    glEnableClientState(GL_VERTEX_ARRAY); // compat profile
    glVertexPointer(3, GL_FLOAT, (GLsizei)sizeof(node), (const void *)offsetof(node, position));

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m.ibo);

    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0f, 1.0f);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glColor4f(fill.r, fill.g, fill.b, fill.a);
    glDrawElements(GL_TRIANGLES, m.indexCount, GL_UNSIGNED_INT, (void *)0);

    glDisable(GL_POLYGON_OFFSET_FILL);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glLineWidth(wireWidth);
    glColor4f(wire.r, wire.g, wire.b, wire.a);
    glDrawElements(GL_TRIANGLES, m.indexCount, GL_UNSIGNED_INT, (void *)0);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDisableClientState(GL_VERTEX_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glPopMatrix();
}

// Zwraca sfer� wok� (0,0,0) o promieniu 'radius' i g�sto�ci 'subdiv'
static void buildIcosphereNodesTris(float radius, int subdiv,
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
