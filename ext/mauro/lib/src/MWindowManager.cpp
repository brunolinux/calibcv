
#include <MWindowManager.h>

using namespace std;


namespace mauro
{

    MWindowManager* MWindowManager::INSTANCE = NULL;


    MWindowManager::MWindowManager()
    {
        m_initialized = false;

        glfwInit();
        // Initialize shared res window
        glfwWindowHint( GLFW_CONTEXT_VERSION_MAJOR, MAURO_GL_CONTEXT_VERSION_MAJOR );
        glfwWindowHint( GLFW_CONTEXT_VERSION_MINOR, MAURO_GL_CONTEXT_VERSION_MINOR );
        glfwWindowHint( GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE );
        glfwWindowHint( GLFW_VISIBLE, GL_FALSE );
        glfwWindowHint( GLFW_RESIZABLE, GL_FALSE );

        m_sharedWindow = glfwCreateWindow( MAURO_GL_SHARED_WINDOW_WIDTH, 
                                           MAURO_GL_SHARED_WINDOW_HEIGHT, 
                                           "BaseWindow", NULL, NULL );
        if ( m_sharedWindow == NULL )
        {
            cout << "Couldn't initialize base shared res window" << endl;
            glfwTerminate();
            return;
        }

        glfwMakeContextCurrent( m_sharedWindow );

        if ( !gladLoadGLLoader( ( GLADloadproc ) glfwGetProcAddress ) )
        {
            cout << "Couldn't initialize glad" << endl;
            glfwTerminate();
            return;
        }

        // Just in case ?
        glfwHideWindow( m_sharedWindow );

        // Restore options for child windows
        glfwWindowHint( GLFW_VISIBLE, GL_TRUE );
        glfwWindowHint( GLFW_RESIZABLE, GL_TRUE );
    }

    MWindowManager::~MWindowManager()
    {
        // Release all child windows ************
        for ( auto _keypair : m_windows )
        {
            delete _keypair.second;
        }

        m_windows.clear();

        // Release shared window ****************
        glfwDestroyWindow( m_sharedWindow );
        m_sharedWindow = NULL;
    }

    MWindowManager* MWindowManager::create()
    {
        if ( MWindowManager::INSTANCE != NULL )
        {
            cout << "window manager already created" << endl;
            return MWindowManager::INSTANCE;
        }

        MWindowManager::INSTANCE = new MWindowManager();

        return MWindowManager::INSTANCE;
    }

    void MWindowManager::release()
    {
        if ( MWindowManager::INSTANCE == NULL )
        {
            cout << "window manager already released" << endl;
            return;
        }

        delete MWindowManager::INSTANCE;
        MWindowManager::INSTANCE = NULL;
    }


    MWindow* MWindowManager::createWindow( string windowName, int width, int height )
    {
        if ( m_windows.find( windowName ) != m_windows.end() )
        {
            // Returning already existing window if there is one
            cout << "Window with name ( " << windowName << " ) already exists in windowmanager" << endl;
            return m_windows[ windowName ];
        }

        auto _window = new MWindow( m_sharedWindow, windowName, width, height );

        m_windows[ windowName ] = _window;

        return _window;
    }


    void MWindowManager::run()
    {
        m_isActive = true;
        pthread_create( &m_threadHandle, NULL, MWindowManager::renderCall, NULL );
    }

    void MWindowManager::stop()
    {
        m_isActive = false;
        pthread_join( m_threadHandle, NULL );
    }

    void* MWindowManager::renderCall( void* _ )
    {
        while( MWindowManager::INSTANCE->isActive() )
        {
            auto _windows = MWindowManager::INSTANCE->getWindows();

            for ( auto _keypair : _windows )
            {
                _keypair.second->render();
            }
        }
    }
}