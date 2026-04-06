#pragma once
#include "hmain.hpp"


inline std::string file = "data/kryzys.csv";           // plik wejsciowy
inline std::string write_file = "output/kryzys_out.csv"; // plik wyjsciowy

constexpr float SOUND_V = 1440.0f / 1.0f; // C++

inline float inv_fp = 1 / 1000000.0f; // z jaka czestotliwoscia probki maja byc interpolowane
inline float dt = 1 / 100000.0f;      // dokladnosc i predkosc symulacji


inline float gAvgEdgeLen = 0.0f;                     // siatka
inline int gBlindRings = 9 * 3 * 9 * 27 * 9 * 9 * 9; // 3 tr�jk�ty
inline float gBlindRadius = 0.0f;                    // = gBlindRings * gAvgEdgeLen


inline int ktore_odbicie = 0;

inline std::vector<WindowPacket> gWinPackets;
inline size_t gWinIdx = 0; // kt�ry 5 ms segment aktualnie nadajemy
inline int windows_number = 10; // ile okien symulacji ma przejsc

inline bool rewind_punkt = false;