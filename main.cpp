#include "hmain.hpp"

#define M_PI 3.1415

// float dt = 0.00001f;





//std::vector<Wave> waves;
Wave current_wave;



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



//float T2 = 0;

std::vector<float> time_T2;
std::vector<float> time_T2_prev;

static constexpr float R0_ATTEN = 0.0001f; // [m] � near-field cap (np. 5 cm)


bool serio_first = true;



// WAZNE DANE
float radius = 0.02f;
float time_passed = 0.0f;
float window_ms = 1.0f;
// dane z csv
std::vector<double> tSec; // czas w sekundach (po przeskalowaniu)
std::vector<float> y;     // warto�ci pr�bek (2. kolumna)

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

std::vector<Triangle> microphone;
std::vector<Triangle> source_tri;

// fala mikrofon i zrodlo
static MeshGL gWaveGL, gMicGL, gSrcGL;

// do FPS
int frameCount = 0;
double previousTime = 0.0;
double fps = 0.0;

bool first = true;



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
void RenderWave(Wave& wave);
// void drawCuboid(float width, float height, float depth);
void drawCuboidTransparentSorted(struct Cuboid_dimensions temp_Cube);
int printOversizedTriangles(Wave& wave, float maxArea);
// Ustawia w�z�y w pozycji �r�d�a i nadaje im pr�dko�ci/kierunki startowe.
void killAllNodes(Wave& wave);
void buildBuffersFor(const std::vector<node> &verts, const std::vector<Triangle> &tris, MeshGL &m, bool dynamic = true);
void updatePositionsFor(const std::vector<node> &verts, MeshGL &m);
void drawMesh(const MeshGL &m, const glm::vec3 &offset = glm::vec3(0), const glm::vec4 &fill = glm::vec4(1.0f, 0.5f, 0.0f, 1.0f), const glm::vec4 &wire = glm::vec4(0.0f, 0.0f, 0.0f, 0.6f), float wireWidth = 0.2f);
static void buildIcosphereNodesTris(float radius, int subdiv, std::vector<node> &out_nodes, std::vector<Triangle> &out_tris);
// Zapis do pliku po symulacji
// int removeSlowNodes(float minSpeed);



// do separatora
static inline char detect_sep(const std::string &line)
{
    return (line.find(';') != std::string::npos) ? ';' : ',';
}

bool writeMicCsv(const std::string &path = write_file)
{
    if (Mic.gMicEvents.empty())
        return false;

    float t_end = Mic.gMicEvents.back().t;
    for (float t = 0; t <= t_end; t += inv_fp)
    {
        Mic.gMicEvents.push_back({q5_inter(t), 0});
    }

    // posortuj po czasie
    std::sort(Mic.gMicEvents.begin(), Mic.gMicEvents.end(),
              [](const MicSample &a, const MicSample &b)
              { return a.t < b.t; });

    // scal duplikaty czas�w (dok�adnie r�wne) przez sumowanie warto�ci
    std::vector<MicSample> merged;
    merged.reserve(Mic.gMicEvents.size());
    for (const auto &s : Mic.gMicEvents)
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








float calculateTriangleArea(Wave& wave, const std::vector<node>& nodes, int a, int b, int c)
{
    glm::vec3 ab = wave.nodes[b].position - wave.nodes[a].position;
    glm::vec3 ac = wave.nodes[c].position - wave.nodes[a].position;
    glm::vec3 cross = glm::cross(ab, ac);
    return 0.5f * glm::length(cross); // funkcja do obliczania pola
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
        // for (auto& wave : waves)
        //     printOversizedTriangles(wave, 0.05f);
        printOversizedTriangles(current_wave, 0.05f);
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

void RenderWave(Wave& wave)
{
    // glEnable(GL_BLEND);
    // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // static std::vector<Triangle> triangles;
    if (first && !rewind_punkt)
    {
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

        wave.nodes.clear();
        wave.triangles.clear();

        if (serio_first)
        {
            serio_first = false;
            // Zbuduj geometri� dla mikrofonu i �r�d�a
            const int SUBDIV_RIGID = 2;

            Mic.verts.clear();
            microphone.clear();
            buildIcosphereNodesTris(Mic.radius, SUBDIV_RIGID, Mic.verts, microphone);

            Source.verts.clear();
            source_tri.clear();
            buildIcosphereNodesTris(Source.radius, SUBDIV_RIGID, Source.verts, source_tri);

            // Bufory GPU :
            buildBuffersFor(Mic.verts, microphone, gMicGL, /*dynamic=*/false);
            buildBuffersFor(Source.verts, source_tri, gSrcGL, /*dynamic=*/false);
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

        wave.nodes.reserve(verts.size());
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

            wave.nodes.push_back(nd);
        }

        wave.triangles.reserve(faces.size());
        for (const auto &f : faces)
        {
            wave.triangles.push_back({{f.x, f.y, f.z}});
            // break;
        }
        // buildSphereBuffers(/*dynamic=*/true);
        buildBuffersFor(wave.nodes, wave.triangles, gWaveGL, /*dynamic=*/true);

        // po zbudowaniu dwudziestoscianu
        gAvgEdgeLen = computeAvgEdgeLen(verts, faces);
        gBlindRadius = gBlindRings * gAvgEdgeLen;
        // gBlindRadius = 2.5f;

        first = false;
    }

    static int frameCount = 0;
    if ((frameCount % 8) == 0)
    {
        size_t budget = std::min<size_t>(4000, std::max<size_t>(1, wave.triangles.size() / 10));
        budget = wave.triangles.size(); // TO DO: MOZNA ZMIENIC NA WIEKSZE (W SENSIE ZMIENIC np. 4 -> 2)
        int threads = std::max(1u, std::thread::hardware_concurrency());

        wave.refineIcosahedron_chunked_mt(0.05f, budget, Cube, threads);
    }
    frameCount++;
    if (simulate)
    {
        // updatePhysics(dt, Cube, Obstacle);
        // usuwanie zuzytych nodes
        wave.pruneSlowNodes(getAtTime((gWinIdx)*window_ms / 1000.0f), time_passed);
        if (wave.nodes.empty())
        {
            ktore_odbicie = 0;
            time_T2_prev.swap(time_T2);

            // Domknij wszystkie wci�� wisz�ce odbicia z poprzedniej ramki
            for (auto &buf : Mic.wektorPrev)
            {
                if (!buf.empty())
                {
                    flush_echo_interpolated(Mic, buf, /*TS_native=*/inv_fp, /*quantize=*/true);
                    buf.clear();
                }
            }

            Mic.gPrevHits.swap(Mic.gCurrHits);

            Mic.wektorPrev.swap(Mic.wektorCurr);
            Mic.wektorCurr.clear();

            Mic.gCurrHits.clear();
            Mic.gPrevTaken.assign(Mic.gPrevHits.size(), 0); // reset znacznik�w

            time_T2.clear();
            if (!beginNextWindow() or gWinIdx == windows_number)
            {
                writeMicCsv(write_file);
                Mic.resetMicEvents();
            }
            // rewind_punkt = false;
        }
        updatePhysics(dt, window_ms, time_passed, wave, Source, Mic, Cube, Obstacle);
    }

    // odbuduj bufory tylko gdy trzeba
    if (wave.mesh_dirty)
    {
        // buildSphereBuffers(/*dynamic=*/true);
        buildBuffersFor(wave.nodes, wave.triangles, gWaveGL, /*dynamic=*/true);
        wave.mesh_dirty = false;
    }
    else
    {
        // w przeciwnym razie tylko podmie� pozycje
        // updateSpherePositions();
        updatePositionsFor(wave.nodes, gWaveGL);
        updatePositionsFor(Mic.verts, gMicGL);
        updatePositionsFor(Source.verts, gSrcGL);
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

int printOversizedTriangles(Wave& wave, float maxArea)
{
    // te same sta�e co w refine
    const float cuboidHalfWidth = Cube.width * 0.5f;
    const float cuboidHalfHeight = Cube.height * 0.5f;
    const float cuboidHalfDepth = Cube.depth * 0.5f;
    const float safetyMargin = 0.2f;

    // por�wnujemy |cross|^2 > (2*maxArea)^2
    const float thr2 = 4.0f * maxArea * maxArea;

    int count = 0;
    for (const auto &t : wave.triangles)
    {
        int a = t.indices[0], b = t.indices[1], c = t.indices[2];
        const glm::vec3 &pa = wave.nodes[a].position;
        const glm::vec3 &pb = wave.nodes[b].position;
        const glm::vec3 &pc = wave.nodes[c].position;

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
    float procent = (float)count / (float)wave.triangles.size() * 100.0f;
    std::cout << "Oversized (splittable) triangles: "
              << wave.triangles.size() << "\n";
    return count;
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

    //waves.push_back(Wave());
    current_wave = Wave();

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
        //for (auto& wave : waves)
        //    RenderWave(wave);
        RenderWave(current_wave);

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