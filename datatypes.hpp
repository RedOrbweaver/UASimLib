#pragma once
#include "UASimLib.hpp"

struct EchoHit
{
    float T = 0.f;                // start okna
    float T2 = 0.f;               // koniec okna
    float energy = 0.f;           // energy noda
    float doppler = 1.f;          // nodes[i].doppler
    int bounces = 0;              // nodes[i].bounces
    glm::vec3 dir_in = {0, 0, 1}; // znormalizowany kierunek fali w chwili trafienia (velocity)
};

struct MeshGL
{
    GLuint vbo = 0;
    GLuint ibo = 0;
    GLsizei indexCount = 0;
    bool dynamic = false;
};

struct Triangle
{
    int indices[3];
};

struct node
{
    glm::vec3 position = {0, 0, 0};
    glm::vec3 velocity = {0, 0, 0}; // kierunek propagacji * C_SOUND
    float energy = 0.0f;
    uint8_t bounces = 0; // ile odbic
    float tEmit = 0.0f;  // czas emisji (sim-time)
    int seedId = -1;     // indeks wierzcho�ka siatki
    float suppressUntilT = -1e30f;
    // float path = 0.0f;
    float doppler = 1.0f;
};

struct SoundSource
{
    float src_x = 0;
    float src_y = 0;
    float src_z = 0;
    // glm::vec3 starting_point = glm::vec3(src_x, src_y, src_z);
    glm::vec3 velocity = glm::vec3(0.0f, 0, 0);
    glm::vec3 rewind_point = glm::vec3(src_x, src_y, src_z);
    glm::vec3 rewind_vel = velocity;
    std::vector<node> verts;
    float radius = 0.02f;
};

struct Cuboid_dimensions
{
    float width = 0.0f;
    float height = 0.0f;
    float depth = 0.0f;
    float x_offset = 0.0f;
    float y_offset = 0.0f;
    float z_offset = 0.0f;
};

struct WindowPacket
{
    float tEmit = 0.0f;        // czas startu okna (sek)
    float amplitude = 0.0f;    // np. max |y| z okna (do progowania/prune)
    std::vector<float> times;  // czasy pr�bek relatywnie do tEmit [s], 0..5ms
    std::vector<float> values; // warto�ci pr�bek
};

struct MicSample
{
    float t;
    float value;
}; // czas i wartos