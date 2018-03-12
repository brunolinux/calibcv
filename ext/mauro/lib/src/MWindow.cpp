
#include <MWindow.h>

using namespace std;

namespace mauro
{


	MWindow::MWindow( GLFWwindow* sharedWindowResources, 
					  string windowName, int width, int height )
	{
		m_name = windowName;
		m_width = width;
		m_height = height;

		m_initialized = false;

		m_glfwWindowHandle = glfwCreateWindow( m_width, m_height, m_name.c_str(), 
											   NULL, sharedWindowResources );

		if ( m_glfwWindowHandle == NULL )
		{
			cout << "error creating window " << windowName << endl;
			return;
		}

		m_bgColor[0] = 1.0f;
		m_bgColor[1] = 1.0f;
		m_bgColor[2] = 1.0f;
		m_bgColor[3] = 1.0f;

		glfwMakeContextCurrent( m_glfwWindowHandle );

		glfwGetFramebufferSize( m_glfwWindowHandle, &m_width, &m_height );
		glViewport( 0, 0, m_width, m_height );

		m_initialized = true;
		m_isVisible = true;
	}

	MWindow::~MWindow()
	{
		glfwDestroyWindow( m_glfwWindowHandle );
		m_glfwWindowHandle = NULL;
	}

    void MWindow::show()
    {
    	m_isVisible = true;
    	glfwShowWindow( m_glfwWindowHandle );
    }

    void MWindow::hide()
    {
    	m_isVisible = false;
    	glfwHideWindow( m_glfwWindowHandle );
    }


	void MWindow::render()
	{
		glfwMakeContextCurrent( m_glfwWindowHandle );

		glClearColor( m_bgColor[0], m_bgColor[1],
					  m_bgColor[2], m_bgColor[3] );

		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

		if ( m_isVisible )
		{
			_drawContents();
		}
			
		glfwSwapBuffers( m_glfwWindowHandle );
	}

	void MWindow::_drawContents()
	{
		// Override this
	}
}