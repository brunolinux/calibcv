
#include <MWindow.h>

using namespace std;

namespace mauro
{


	MWindow::MWindow( GLFWwindow* sharedWindowResources, string windowName, int width, int height )
	{
		m_name = windowName;
		m_width = width;
		m_height = height;

		m_initialized = false;

		m_glfwWindowHandle = glfwCreateWindow( m_width, m_height, m_name.c_str(), NULL, sharedWindowResources );

		if ( m_glfwWindowHandle == NULL )
		{
			cout << "error creating window " << windowName << endl;
			return;
		}

		m_initialized = true;
	}




	void MWindow::render()
	{
		glfwMakeContextCurrent( m_glfwWindowHandle );


	}


}