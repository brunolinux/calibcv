
#include <stages/SComputingStageFeatures.h>


namespace calibcv
{


    SComputingStageFeatures::SComputingStageFeatures()
        : SComputingStage()
    {
        cv::SimpleBlobDetector::Params _detectorCreationParams;
        _detectorCreationParams.filterByArea = true;
        _detectorCreationParams.minArea = 200;
        _detectorCreationParams.maxArea = 1000;
        _detectorCreationParams.filterByColor = true;
        _detectorCreationParams.blobColor = 0;
        _detectorCreationParams.filterByConvexity = true;
        _detectorCreationParams.minConvexity = 0.9;
        _detectorCreationParams.maxConvexity = 1;

        m_blobsDetector = cv::SimpleBlobDetector::create( _detectorCreationParams );
    }

    SComputingStageFeatures::~SComputingStageFeatures()
    {
        m_keypoints.clear();
    }

    void SComputingStageFeatures::_run( const cv::Mat& input )
    {
        m_blobsDetector->detect( input, m_keypoints );

        m_stageResult = m_frame.clone();

        for ( cv::KeyPoint _keypoint : m_keypoints )
        {
            cv::circle( m_stageResult, _keypoint.pt, 4, cv::Scalar( 255, 0, 255 ), 2 );
        }
    }

    void SComputingStageFeatures::_run( SComputingStage* parent )
    {
        _run( parent->getStageResult() );
    }

}