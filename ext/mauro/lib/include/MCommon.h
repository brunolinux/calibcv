
#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include <chrono>
#include <thread>

#ifndef GLM_ENABLE_EXPERIMENTAL
#define GLM_ENABLE_EXPERIMENTAL
#endif

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtx/transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

namespace mauro 
{	
    struct MRodrigues
    {
        double theta;
        double ux;
        double uy;
        double uz;
    };

	
    glm::mat4 tf_rodrigues2Rotation( const MRodrigues& rodRot );



}

