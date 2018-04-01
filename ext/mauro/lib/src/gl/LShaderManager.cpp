

#include <gl/LShaderManager.h>

// General purpose shaders
#include <gl/LShaderBasic3d.h>

using namespace std;

namespace engine
{

    LShaderManager* LShaderManager::INSTANCE = NULL;

    LShaderManager::LShaderManager()
    {
        // Load all shaders available

        GLuint _vShader, _fShader, _program;

        cout << "LShaderManager-> using glsl 330 core shaders" << endl;

        // Common shaders **************************************************

        _vShader = createShader( "basic/basic3d_vs.glsl", GL_VERTEX_SHADER );
        _fShader = createShader( "basic/basic3d_fs.glsl", GL_FRAGMENT_SHADER );
        _program = createProgram( _vShader, _fShader );

        programs["basic3d"] = _program;
        programObjs["basic3d"] = new LShaderBasic3d( _program );

        // *****************************************************************
    }

    LShaderManager::~LShaderManager()
    {
        map< string, GLuint >::iterator _it;
        for ( _it = programs.begin(); _it != programs.end(); ++_it )
        {
            glDeleteProgram( _it->second );
        }
    }

    void LShaderManager::release()
    {
        if ( LShaderManager::INSTANCE != NULL )
        {
            delete LShaderManager::INSTANCE;
            LShaderManager::INSTANCE = NULL;
        }
    }

    void LShaderManager::create()
    {
        if ( LShaderManager::INSTANCE != NULL )
        {
            delete LShaderManager::INSTANCE;
        }

        LShaderManager::INSTANCE = new LShaderManager();
    }


    GLuint LShaderManager::createShader( const char* filename, int shaderType )
    {
        // Load the shader code into a string ********************************************
        string _shaderStrCode;
        ifstream _shaderFile;

        string _fullFilePath = GL_RESOURCES_PATH;
        _fullFilePath += "shaders/";
        _fullFilePath += filename;

        cout << "file: " << _fullFilePath << endl;

        _shaderFile.exceptions( ifstream::badbit );

        try 
        {
            _shaderFile.open( _fullFilePath.c_str() );
            stringstream _shaderStream;
            _shaderStream << _shaderFile.rdbuf();

            _shaderFile.close();

            _shaderStrCode = _shaderStream.str();
        }
        catch ( ... )
        {
            cout << "LShader::createFromFile> failed opening the resource file" << endl;
            return 0;
        }

        const char* _shaderCode_cstr = _shaderStrCode.c_str();
        // *******************************************************************************

        GLint _success;
        GLchar _infoLog[1024];

        GLuint _shaderId = glCreateShader( shaderType );
        glShaderSource( _shaderId, 1, &_shaderCode_cstr, NULL );
        glCompileShader( _shaderId );
        glGetShaderiv( _shaderId, GL_COMPILE_STATUS,
                       &_success );

        if ( !_success )
        {
            glGetShaderInfoLog( _shaderId, 1024, 
                                NULL, _infoLog );
            cout << "Shader: " << _fullFilePath << endl;
            cout << "Failed to compile shader" << endl;
            cout << "Error: " << _infoLog << endl;

            return 0;
        }

        return _shaderId;
    }

    GLuint LShaderManager::createProgram( GLuint vShaderId, GLuint fShaderId )
    {
        GLint _success;
        GLchar _infoLog[1024];

        GLuint _programId = glCreateProgram();
        glAttachShader( _programId, vShaderId );
        glAttachShader( _programId, fShaderId );
        glLinkProgram( _programId );

        glDetachShader( _programId, vShaderId );
        glDetachShader( _programId, fShaderId );
        glDeleteShader( vShaderId );
        glDeleteShader( fShaderId );

        glGetProgramiv( _programId, GL_LINK_STATUS, &_success );
        if ( !_success )
        {
            glGetProgramInfoLog( _programId, 1024, NULL, _infoLog );
            cout << "LINKING ERROR: " << _infoLog << endl;

            return 0;
        }

        return _programId;
    }

}