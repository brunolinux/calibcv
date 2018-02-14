

#include <opencv2/opencv.hpp>

#include "CUtils.h"
#include "CDetectionPanel.h"
#include "CDetectionPipeline.h"

using namespace std;

calibcv::CVideoHandler* g_videoHandler;
calibcv::detection::CDetectionPanel* g_detectionPanel;
calibcv::detection::CDetectionPipeline* g_detectionPipeline;

#define VIDEO_TEST_FILE "../res/calibration_ps3eyecam.avi"
#define SAMPLE_TIME 20 // 50fps

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
        int _key = cv::waitKey( SAMPLE_TIME );

        if ( _key == KEY_SPACE )
        {
            g_videoHandler->togglePause();
        }
        else if ( _key == KEY_ENTER && g_videoHandler->isPaused() )
        {
            g_videoHandler->togglePickingROI();
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
        vector< cv::RotatedRect > _ellipsesBOB;
        vector< cv::RotatedRect > _pEllipsesBOB;

        g_videoHandler->takeFrame( _frame );

        calibcv::detection::CProcessingParams _eparams;
        _eparams.minSize  = g_detectionPanel->ellipseMinSize();
        _eparams.maxSize  = g_detectionPanel->ellipseMaxSize();
        _eparams.minRatio = 0.01 * g_detectionPanel->ellipseMinRatio();
        _eparams.maxRatio = 0.01 * g_detectionPanel->ellipseMaxRatio();
        _eparams.roi = g_videoHandler->roi();

        // if ( _eparams.roi.size() == 4 )
        // {
        //     cout << "roi!!!" << endl;
        // }

        // apply pipeline
        g_detectionPipeline->stepMaskCreation( _frame, _mask );
        g_detectionPipeline->stepMaskedCreation( _frame, _mask, _masked );
        g_detectionPipeline->stepEdgesCreation( _masked, _edges,
                                                g_detectionPanel->blurSize(),
                                                g_detectionPanel->cannyMin(),
                                                g_detectionPanel->cannyMax(),
                                                g_detectionPanel->cannySobelMaskSize() );
        g_detectionPipeline->stepFindEllipses( _edges, _matContours, _contours, _hierarchy, _ellipsesBOB,
                                               g_detectionPanel->ellipseCountThreshold() );
        g_detectionPipeline->stepProcessEllipses( _ellipsesBOB, _pEllipsesBOB, _eparams );
        g_detectionPipeline->stepBlendResults( _frame, _result, _pEllipsesBOB );

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
