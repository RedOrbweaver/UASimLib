#pragma once
#include <thread>
#include <algorithm>
#include <iostream>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <fstream>
#include <string>
#include <iomanip>
#include <sstream>
#include <cctype>
#include <memory>

using std::shared_ptr;
using std::make_shared;
using std::vector;
using std::unordered_map;

#include "RedCppLib/RedCppLib.hpp"

using Red::vec2f;
using Red::vec3f;
using Red::vec3i;

#include "datatypes.hpp"

#include "CollisionMaterials/CollisionMaterial.hpp"

#include "CollisionObjects/CollisionObject.hpp"

#include "UASimConstants.hpp"

#include "geometry.hpp"
#include "wave.hpp"
#include "microphone.hpp"
#include "physics.hpp"