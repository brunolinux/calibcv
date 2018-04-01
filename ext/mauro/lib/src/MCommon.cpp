
#include <MCommon.h>



namespace mauro
{



    glm::mat4 tf_rodrigues2Rotation( const MRodrigues& rodRot )
    {
        return glm::rotate( rodRot.theta, rodRot.x, rodRot.y, rodRot.z );
    }




}