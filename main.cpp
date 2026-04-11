#include "hmain.hpp"

#define M_PI 3.1415

// float dt = 0.00001f;




int test = 0;



//float T2 = 0;

std::vector<float> time_T2;
std::vector<float> time_T2_prev;

bool serio_first = true;








// WAZNE DANE
float time_passed = 0.0f;
float window_ms = 1.0f;
// dane z csv
std::vector<double> tSec; // czas w sekundach (po przeskalowaniu)
std::vector<float> y;     // warto�ci pr�bek (2. kolumna)

int screen_width = 1280;
int screen_height = 720;

glm::vec3 cameraPos = glm::vec3(0.0f, 0.0f, 3.0f);
glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);

bool firstMouse = true;
bool mousePressed = false;
float lastX = screen_width / 2.0f;
float lastY = screen_height / 2.0f;
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


Cuboid_dimensions Cube
{
    20.0f, 6.0f, 15.0f, // width, height, depth
    0.0f, 0.0f, 0.0f    // x/y/z offset
};

Cuboid_dimensions Obstacle
{
    0.3f, 0.7f, 0.7f,   // width, height, depth
    500.0f, 1.0f, 50.0f // x/y/z offset
};


SoundSource source;
Microphone Mic;
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
        printOversizedTriangles(current_wave, Cube, 0.05f);
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





void StepSimulation(Wave& wave)
{
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
    updatePhysics(dt, window_ms, time_passed, wave, source, Mic, Cube, Obstacle);
}

void DrawGUI()
{
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
    ImGui::InputFloat("Src x", &source.src_x, 0.1f, source.src_x);
    ImGui::InputFloat("Src y", &source.src_y, 0.1f, source.src_y);
    ImGui::InputFloat("Src z", &source.src_z, 0.1f, source.src_z);
    ImGui::InputFloat("Src_vel x", &source.velocity.x, 5.0f, source.velocity.x);
    ImGui::InputFloat("Src_vel y", &source.velocity.y, 5.0f, source.velocity.y);
    ImGui::InputFloat("Src_vel z", &source.velocity.z, 5.0f, source.velocity.z);
    ImGui::Text("Basen");
    ImGui::InputFloat("Cube width", &Cube.width, 2.0f, Cube.width);
    ImGui::InputFloat("Cube height", &Cube.height, 2.0f, Cube.height);
    ImGui::InputFloat("Cube depth", &Cube.depth, 2.0f, Cube.depth);
    ImGui::InputFloat("Cube x_offset", &Cube.x_offset, 2.0f, Cube.x_offset);
    ImGui::InputFloat("Cube y_offset", &Cube.y_offset, 2.0f, Cube.y_offset);
    ImGui::InputFloat("Cube z_offset", &Cube.z_offset, 2.0f, Cube.z_offset);
    ImGui::End();

    auto avg = [](double* buf, int len)
    {
        double sum = 0;
        for(int i = 0; i < len; i++)
            sum += buf[i];
        return sum / (double)len;
    };

    ImGui::Begin("Performance");

    ImGui::Text("Time passed: %f", time_passed);

    ImGui::Text("Last frame time: %f", last_frame_time);
    ImGui::Text("Last render time: %f", last_render_time);
    ImGui::Text("Last physics time: %f", last_physics_time);
    ImGui::Text("Vertices/s: %f", last_vertices_per_second);

    ImGui::Text("render avg (%i): %f", RENDER_TIMES_KEPT, avg(render_times, RENDER_TIMES_KEPT));
    ImGui::Text("physics avg (%i): %f", PHYSICS_TIMES_KEPT, avg(physics_times, PHYSICS_TIMES_KEPT));
    double vavg = avg(vertex_times, VERTICES_PER_SEC_KEPT);
    ImGui::Text("vertices/s avg: (%i): %f", VERTICES_PER_SEC_KEPT, vavg);

    if(vertices_per_sec_avg_max < vavg)
        vertices_per_sec_avg_max = vavg;

    ImGui::Text("MAX vertices/s avg: (%i): %f", VERTICES_PER_SEC_KEPT, vertices_per_sec_avg_max);

    ImGui::Text("Vertices: %i", current_wave.nodes.size());

    ImGui::End();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

double PushTime(clock_t rawtm, double* buffer, int buffer_len, int& buffer_index, double multiplier = 1.0f)
{
    double tm = (((double)rawtm) / (double)CLOCKS_PER_SEC) * multiplier;

    buffer[buffer_index] = tm;
    buffer_index = (buffer_index + 1) % buffer_len;
    return tm;
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

    GLFWwindow *window = glfwCreateWindow(screen_width, screen_height, "Symulacja", NULL, NULL);
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

    while (!glfwWindowShouldClose(window))
    {
        clock_t frame_start = clock();
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        processInput(window);

        glClearColor(0.5f, 0.5f, 0.5f, 0.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glfwGetWindowSize(window, &screen_width, &screen_height);

        glm::mat4 projection = glm::perspective(glm::radians(fov), (float)screen_width / (float)screen_height, 0.1f, 100.0f);
        glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);

        glMatrixMode(GL_PROJECTION);
        glLoadMatrixf(&projection[0][0]);

        glMatrixMode(GL_MODELVIEW);
        glLoadMatrixf(&view[0][0]);

        // Oblicz FPS
        // calculateFPS();
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
            source.src_x = source.rewind_point.x;
            source.src_y = source.rewind_point.y;
            source.src_z = source.rewind_point.z;
            source.velocity = source.rewind_vel;

            rewind_punkt = true;
            current_wave.Begin(source, gWinPackets[gWinIdx]);

            if (serio_first)
            {
                serio_first = false;
                // Zbuduj geometri� dla mikrofonu i �r�d�a
                const int SUBDIV_RIGID = 2;

                Mic.verts.clear();
                microphone.clear();
                buildIcosphereNodesTris(Mic.radius, SUBDIV_RIGID, Mic.verts, microphone);

                source.verts.clear();
                source_tri.clear();
                buildIcosphereNodesTris(source.radius, SUBDIV_RIGID, source.verts, source_tri);

                // Bufory GPU :
                buildBuffersFor(Mic.verts, microphone, gMicGL, /*dynamic=*/false);
                buildBuffersFor(source.verts, source_tri, gSrcGL, /*dynamic=*/false);
            }

            // buildSphereBuffers(/*dynamic=*/true);
            buildBuffersFor(current_wave.nodes, current_wave.triangles, gWaveGL, /*dynamic=*/true);

            // po zbudowaniu dwudziestoscianu
            
            // gBlindRadius = 2.5f;

            first = false;
        }
        if (simulate)
        {
            clock_t sim_start = clock();
            StepSimulation(current_wave);
            clock_t sim_tm = clock() - sim_start;
            last_physics_time = PushTime(sim_tm, physics_times, PHYSICS_TIMES_KEPT, physics_times_index);
            total_physics_time += last_physics_time;

            last_vertices_per_second = vertex_times[vertex_times_index] = (double)current_wave.nodes.size() / last_physics_time;
            vertex_times_index = (vertex_times_index+1) % VERTICES_PER_SEC_KEPT;

            time_passed += dt;

            if (time_passed / dt >= (window_ms / 1000.0f) / dt && rewind_punkt)
            {
                Mic.rewind_point = glm::vec3(Mic.mic_x, Mic.mic_y, Mic.mic_z);
                Mic.rewind_vel = Mic.mic_velocity;
                source.rewind_point = glm::vec3(source.src_x, source.src_y, source.src_z);
                source.rewind_vel = source.velocity;
                rewind_punkt = false;
            }
        }

        
        clock_t render_start = clock();
        RenderWave(current_wave, Mic, source, cameraPos, gWaveGL, gMicGL, gSrcGL, Cube, Obstacle);
        clock_t render_tm = clock() - render_start;
        last_render_time = PushTime(render_tm, render_times, RENDER_TIMES_KEPT, render_times_index);
        total_render_time += last_render_time;



        glfwPollEvents();

        DrawGUI();

        glfwSwapBuffers(window);

        clock_t frame_tm = clock() - frame_start;
        last_frame_time = (double)frame_tm / (double)CLOCKS_PER_SEC;
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwTerminate();
    return 0;
}