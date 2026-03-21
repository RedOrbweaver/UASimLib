void updatePhysics(float dt, float& window_ms, float& time_passed, Wave& wave, source& Source, Microphone& Mic, struct Cuboid_dimensions Pool, struct Cuboid_dimensions temp_Obstacle) // glowna petla fizyki programu
{
    const float Pool_halfW = 0.5f * Pool.width;
    const float Pool_halfH = 0.5f * Pool.height;
    const float Pool_halfD = 0.5f * Pool.depth;

    const float e = 0.8;      // wsp. spr�ysto�ci
    const float eps = 0.001f; // minimalne odsuni�cie od �ciany

    const float micR = Mic.radius;
    ++gFrameId;
    // Pomocnik do odbicia
    auto bounce1D = [&](float &pos, float &vel, float minb, float maxb) -> int
    {
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

    auto bounceObstacleMic = [&](Microphone &temp_Mic, struct Cuboid_dimensions temp_Obstacle) // TO DO:: narazie nie dziala, do odbic od przeszkody
    {
        const float temp_Obstacle_halfW = 0.5f * temp_Obstacle.width;
        const float temp_Obstacle_halfH = 0.5f * temp_Obstacle.height;
        const float temp_Obstacle_halfD = 0.5f * temp_Obstacle.depth;

        if (temp_Obstacle_halfW + temp_Obstacle.x_offset >= 0)
        {
            if (temp_Mic.mic_x > -temp_Obstacle_halfW + temp_Obstacle.x_offset && temp_Mic.mic_x < temp_Obstacle_halfW + temp_Obstacle.x_offset)
            {
                temp_Mic.mic_velocity.x *= -1;
            }
        }
        else
        {
            if (temp_Mic.mic_x < -temp_Obstacle_halfW + temp_Obstacle.x_offset && temp_Mic.mic_x > temp_Obstacle_halfW + temp_Obstacle.x_offset)
            {
                temp_Mic.mic_velocity.x *= -1;
            }
        }

        if (temp_Obstacle_halfH + temp_Obstacle.y_offset >= 0)
        {
            if (temp_Mic.mic_y > -temp_Obstacle_halfH + temp_Obstacle.y_offset && temp_Mic.mic_y < temp_Obstacle_halfH + temp_Obstacle.y_offset)
            {
                temp_Mic.mic_velocity.y *= -1;
            }
        }
        else
        {
            if (temp_Mic.mic_y < -temp_Obstacle_halfH + temp_Obstacle.y_offset && temp_Mic.mic_y > temp_Obstacle_halfH + temp_Obstacle.y_offset)
            {
                temp_Mic.mic_velocity.y *= -1;
            }
        }

        if (temp_Obstacle_halfD + temp_Obstacle.z_offset >= 0)
        {
            if (temp_Mic.mic_z > -temp_Obstacle_halfD + temp_Obstacle.z_offset && temp_Mic.mic_z < temp_Obstacle_halfD + temp_Obstacle.z_offset)
            {
                temp_Mic.mic_velocity.z *= -1;
            }
        }
        else
        {
            if (temp_Mic.mic_z < -temp_Obstacle_halfD + temp_Obstacle.z_offset && temp_Mic.mic_z > temp_Obstacle_halfD + temp_Obstacle.z_offset)
            {
                temp_Mic.mic_velocity.z *= -1;
            }
        }
    };

    auto bounceObstacleWave = [&](node &temp_node, struct Cuboid_dimensions temp_Obstacle) // TODO:: narazie nie dziala, do odbic od przeszkody
    {
        const float temp_Obstacle_halfW = 0.5f * temp_Obstacle.width;
        const float temp_Obstacle_halfH = 0.5f * temp_Obstacle.height;
        const float temp_Obstacle_halfD = 0.5f * temp_Obstacle.depth;

        if ((temp_node.position.x > -temp_Obstacle_halfW + temp_Obstacle.x_offset && temp_node.position.y > -temp_Obstacle_halfH + temp_Obstacle.y_offset && temp_node.position.z > -temp_Obstacle_halfD + temp_Obstacle.z_offset) &&
            (temp_node.position.x < temp_Obstacle_halfW + temp_Obstacle.x_offset && temp_node.position.y < temp_Obstacle_halfH + temp_Obstacle.y_offset && temp_node.position.z < temp_Obstacle_halfD + temp_Obstacle.z_offset))
        {
            // std::cout << "KOLIZJA Z FALA" << std::endl;
            if (temp_node.position.x - 5 * eps < -temp_Obstacle_halfW + temp_Obstacle.x_offset || temp_node.position.x + 5 * eps > temp_Obstacle_halfW + temp_Obstacle.x_offset)
            {
                temp_node.velocity.x *= -1;
            }
            else if (temp_node.position.y - 5 * eps < -temp_Obstacle_halfH + temp_Obstacle.y_offset || temp_node.position.y + 5 * eps > temp_Obstacle_halfH + temp_Obstacle.y_offset)
            {
                temp_node.velocity.y *= -1;
            }
            else if (temp_node.position.z - 5 * eps < -temp_Obstacle_halfD + temp_Obstacle.z_offset || temp_node.position.z + 5 * eps > temp_Obstacle_halfD + temp_Obstacle.z_offset)
            {
                temp_node.velocity.z *= -1;
            }
        }
    };

    // odbicie mikrofonu
    Mic.mic_x += Mic.mic_velocity.x * dt;
    Mic.mic_y += Mic.mic_velocity.y * dt;
    Mic.mic_z += Mic.mic_velocity.z * dt;

    Source.src_x += Source.velocity.x * dt;
    Source.src_y += Source.velocity.y * dt;
    Source.src_z += Source.velocity.z * dt;

    // Odbicia mikrofonu od �cian basenu
    bounce1D(Mic.mic_x, Mic.mic_velocity.x, -Pool_halfW + Pool.x_offset + micR, Pool_halfW + Pool.x_offset - micR);
    bounce1D(Mic.mic_y, Mic.mic_velocity.y, -Pool_halfH + Pool.y_offset + micR, Pool_halfH + Pool.y_offset - micR);
    bounce1D(Mic.mic_z, Mic.mic_velocity.z, -Pool_halfD + Pool.z_offset + micR, Pool_halfD + Pool.z_offset - micR);

    // odbicia zrodla od scian
    bounce1D(Source.src_x, Source.velocity.x, -Pool_halfW + Pool.x_offset, Pool_halfW + Pool.x_offset);
    bounce1D(Source.src_y, Source.velocity.y, -Pool_halfH + Pool.y_offset, Pool_halfH + Pool.y_offset);
    bounce1D(Source.src_z, Source.velocity.z, -Pool_halfD + Pool.z_offset, Pool_halfD + Pool.z_offset);

    // odbicia od przeszkody (MIKROFON)
    // bounceObstacleMic(Mic, temp_Obstacle);

    // doKill = (glfwGetTime() >= 8.0);
    // odbicia fali od scian
    for (int i = 0; i < (int)wave.nodes.size(); ++i)
    {
        auto &p = wave.nodes[i].position;
        auto &v = wave.nodes[i].velocity;
        auto &energy = wave.nodes[i].energy;

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
    }
    // if (doKill) doKill = false;
    // dodaj czas
    time_passed += dt;
    // std::cout << "Czas propagacji danej ramki:" << " " << time_passed << std::endl;
    if (time_passed * 1000 >= window_ms)
    {
        // true = false;
    }

    if (time_passed / dt >= (window_ms / 1000.0f) / dt && rewind_punkt)
    {
        Mic.rewind_point = glm::vec3(Mic.mic_x, Mic.mic_y, Mic.mic_z);
        Mic.rewind_vel = Mic.mic_velocity;
        Source.rewind_point = glm::vec3(Source.src_x, Source.src_y, Source.src_z);
        Source.rewind_vel = Source.velocity;
        rewind_punkt = false;
    }

    // rewind_punkt = false;
}