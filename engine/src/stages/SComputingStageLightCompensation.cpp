
#include <stages/SComputingStageLightCompensation.h>

namespace calibcv
{


    SComputingStageLightCompensation::SComputingStageLightCompensation()
        : SComputingStage()
    {
        
    }

    SComputingStageLightCompensation::~SComputingStageLightCompensation()
    {

    }


    void SComputingStageLightCompensation::_run( const cv::Mat& input )
    {
        // from here : https://stackoverflow.com/questions/24341114/simple-illumination-correction-in-images-opencv-c

        cv::Mat _imageLab, _tmp;

        cv::cvtColor( input, _imageLab, CV_BGR2Lab );
        // Extract the L channel
        std::vector<cv::Mat> _labPlanes( 3 );
        cv::split( _imageLab, _labPlanes );  // now we have the L image in lab_planes[0]
        // apply the CLAHE algorithm to the L channel
        cv::Ptr< cv::CLAHE >_clahe = cv::createCLAHE();
        _clahe->setClipLimit( 4 );
        _clahe->apply( _labPlanes[0], _tmp );
        // Merge the the color planes back into an Lab image
        _tmp.copyTo( _labPlanes[0] );
        cv::merge( _labPlanes, _imageLab );

        // convert back to RGB
        cv::cvtColor( _imageLab, m_stageResult, CV_Lab2BGR );
    }

    void SComputingStageLightCompensation::_run( SComputingStage* parent )
    {
        assert( parent == NULL );
    }
}
