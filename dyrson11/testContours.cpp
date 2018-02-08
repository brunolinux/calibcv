

#include <opencv2/opencv.hpp>

#include "CUtils.h"
#include "CDetectionPanel.h"
#include "CDetectionPipeline.h"

using namespace std;

calibcv::CVideoHandler* g_videoHandler;
calibcv::detection::CDetectionPanel* g_detectionPanel;
calibcv::detection::CDetectionPipeline* g_detectionPipeline;

#define VIDEO_TEST_FILE "../res/calibration_ps3eyecam.avi"
#define SAMPLE_TIME 33 // 30fps

int main()
{
    g_videoHandler      = calibcv::CVideoHandler::create();
    g_detectionPanel    = calibcv::detection::CDetectionPanel::create();
    g_detectionPipeline = calibcv::detection::CDetectionPipeline::create();

    if ( !g_videoHandler->openVideo( VIDEO_TEST_FILE ) )
    {
        cout << "couldnt open: " << VIDEO_TEST_FILE << endl;
        exit( -1 );
    }

    while ( 1 )
    {
        int _key = cv::waitKey( SAMPLE_TIME ) & 255;

        if ( _key == KEY_SPACE )
        {
            g_videoHandler->togglePause();
        }
        else if( _key == KEY_ESCAPE )
        {
            break;
        }

        cv::Mat _frame;
        cv::Mat _mask;
        cv::Mat _masked;
        cv::Mat _edges;
        cv::Mat _matContours;
        cv::Mat _result;

        calibcv::CTypeContours _contours;
        calibcv::CTypeHierarchy _hierarchy;
        //vector< cv::RotatedRect > _ellipsesBOB;
        vector< cv::KeyPoint > keypoints;

        g_videoHandler->takeFrame( _frame );

        // apply pipeline
        g_detectionPipeline->stepMaskCreation( _frame, _mask, keypoints);
        g_detectionPipeline->stepMaskedCreation( _frame, _mask, _masked );
        g_detectionPipeline->stepEdgesCreation( _masked, _edges,
                                                g_detectionPanel->blurSize(),
                                                g_detectionPanel->cannyMin(),
                                                g_detectionPanel->cannyMax(),
                                                g_detectionPanel->cannySobelMaskSize() );
        g_detectionPipeline->stepFindEllipses( _edges, _matContours, _contours, _hierarchy, keypoints,
                                               g_detectionPanel->ellipseCountThreshold() );
        // g_detectionPipeline->stepProcessEllipses( _ellipsesBOB, _pEllipsesBOB );
        g_detectionPipeline->stepBlendResults( _frame, _result, keypoints );

        // show results
        g_detectionPanel->showMask( _mask );
        g_detectionPanel->showMasked( _masked );
        g_detectionPanel->showEdges( _edges );
        g_detectionPanel->showContours( _matContours );
        g_detectionPanel->showEllipses( _result );

    }

    calibcv::CVideoHandler::release();
    calibcv::detection::CDetectionPanel::release();
    calibcv::detection::CDetectionPipeline::release();

    return 0;
}
