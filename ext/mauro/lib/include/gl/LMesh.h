
#pragma once

#include "LCommon.h"
#include "LVertexBuffer.h"
#include "LVertexArray.h"
#include "LIndexBuffer.h"
#include "LIRenderable.h"

using namespace std;


namespace engine
{



    class LMesh : public LIRenderable
    {

        private :

        LVertexArray* m_vertexArray;
        LIndexBuffer* m_indexBuffer;

        LVertexBuffer* m_vBuffer;
        LVertexBuffer* m_nBuffer;

        public :

        LMesh( const vector<LVec3>& vertices, 
               const vector<LVec3>& normals,
               const vector<LInd3>& indices );

        ~LMesh();

        glm::mat4 getModelMatrix();

        LVertexArray* getVertexArray() const { return m_vertexArray; }
        LIndexBuffer* getIndexBuffer() const { return m_indexBuffer; }

        void render() override;
    };





}
