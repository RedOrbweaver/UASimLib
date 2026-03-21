// #include <glad/glad.h>
#include <GLES3/gl3.h>
#include <GL/glew.h>
#include <EGL/egl.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
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
#include "libraries/imgui-master/imgui.h"
#include "libraries/imgui-master/imgui_impl_glfw.h"
#include "libraries/imgui-master/imgui_impl_opengl3.h"
#include "libraries/imgui-master/imgui_impl_opengl3_loader.h"

#include "datatypes.hpp"
#include "wave.hpp"
#include "microphone.hpp"
#include "physics.hpp"
#include "render.hpp"