
#pragma once

#include <MCommon.h>

using namespace std;

namespace mauro 
{


    class MWindow
    {

        protected :

        GLFWwindow* m_glfwWindowHandle;

        int m_width;
        int m_height;
        string m_name;

        bool m_initialized;
        bool m_isVisible;

        float m_bgColor[4];

        virtual void _drawContents();

        public :

        MWindow( GLFWwindow* sharedWindowResources, 
                 string windowName, int wWidth, int wHeight );

        ~MWindow();
        
        void render();

        void show();
        void hide();

        int width() { return m_width; }
        int height() { return m_height; }
        string name() { return m_name; }
    };




}