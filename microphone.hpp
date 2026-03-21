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
};