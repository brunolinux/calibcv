

#pragma once

#include <glad/glad.h>


namespace engine
{
    
    class LVertexBuffer
    {

        private :

        GLuint m_bufferId;
        GLuint m_usage;
        GLuint m_componentCount;

        public :


        LVertexBuffer();
        LVertexBuffer( GLuint usage );

        ~LVertexBuffer();

        void setData( GLuint size, GLuint componentCount, GLfloat* pData );
        void updateData( GLuint size, GLfloat* pData );
        void bind();
        void unbind();

        GLuint getComponentCount() { return m_componentCount; }
    };



}