#pragma once

class Microphone
{
    public:
    float radius = 0.51f;

    float mic_x = 4;
    float mic_y = 0.0;
    float mic_z = 0.0;
    // glm::vec3 starting_point = glm::vec3(mic_x, mic_y, mic_z);
    glm::vec3 mic_velocity = glm::vec3(20.0f, 000.0f, 0.0f);
    glm::vec3 rewind_point = glm::vec3(mic_x, mic_y, mic_z);
    glm::vec3 rewind_vel = mic_velocity;
    std::vector<node> verts;

    std::vector<std::vector<std::pair<float, float>>> wektor;
    std::vector<std::vector<std::pair<float, float>>> wektorPrev; // bufory okien z poprzedniej ramki
    std::vector<std::vector<std::pair<float, float>>> wektorCurr; // bufory z bie��cej ramki

    // Bufory do laczenia ramek
    std::vector<EchoHit> gPrevHits, gCurrHits;
    std::vector<char> gPrevTaken;

    std::vector<int> matches;

    std::vector<MicSample> gMicEvents;

    void logMicHit(float t_dop, float value)
    {
        if (t_dop < 0.0f)
            t_dop = 0.0f; // na wszelki wypadek
        // std::lock_guard<std::mutex> lk(gMicMtx);
        gMicEvents.push_back({t_dop, value});
    }
    void resetMicEvents() 
    { 
        gMicEvents.clear(); 
    }
    Microphone()
    {

    }
};