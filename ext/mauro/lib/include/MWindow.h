
#pragma once

#include <MCommon.h>

using namespace std;

namespace mauro 
{


    class MWindow
    {

        private :

        GLFWwindow* m_glfwWindowHandle;

        int m_width;
        int m_height;
        string m_name;

        bool m_initialized;

        public :

        MWindow( GLFWwindow* sharedWindowResources, string windowName, int wWidth, int wHeight );


        void render();

        int width() { return m_width; }
        int height() { return m_height; }
        string name() { return m_name; }

    };




}