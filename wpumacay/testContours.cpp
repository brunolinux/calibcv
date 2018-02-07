

#include <opencv2/opencv.hpp>

#include "CUtils.h"
#include "CTuning.h"

using namespace std;

calibcv::CVideoHandler* g_videoHandler;
calibcv::tuning::CColorSpaceTuner* g_cspaceTuner;

#define VIDEO_TEST_FILE "../res/calibration_ps3eyecam.avi"
#define SAMPLE_TIME 33 // 30fps

int main()
{
    g_videoHandler = calibcv::CVideoHandler::create();

    if ( !g_videoHandler->openVideo( VIDEO_TEST_FILE ) )
    {
        cout << "couldnt open: " << VIDEO_TEST_FILE << endl;
        exit( -1 );
    }

    g_cspaceTuner = calibcv::tuning::CColorSpaceTuner::create();

    while ( 1 )
    {
        int _key = cv::waitKey( SAMPLE_TIME );

        if ( _key == KEY_SPACE )
        {
            g_videoHandler->togglePause();
        }
        else if( _key == KEY_ESCAPE )
        {
            break;
        }

        cv::Mat _frame;
        cv::Mat _threshed;
        cv::Mat _frameCSpace;

        g_videoHandler->takeFrame( _frame );

        // cv::cvtColor( _frame, _frameCSpace, cv::COLOR_BGR2HSV );
        cv::cvtColor( _frame, _frameCSpace, cv::COLOR_BGR2YCrCb );

        cv::Mat _eqChannel;
        cv::Mat _channel;
        // cv::extractChannel( _frameCSpace, _channel, 2 );
        cv::extractChannel( _frameCSpace, _channel, 0 );
        cv::equalizeHist( _channel, _eqChannel );

        // cv::insertChannel( _eqChannel, _frameCSpace, 2 );
        cv::insertChannel( _eqChannel, _frameCSpace, 0 );

        cv::inRange( _frameCSpace, 
                     g_cspaceTuner->cspaceMin(), 
                     g_cspaceTuner->cspaceMax(), 
                     _threshed );

        g_cspaceTuner->setBaseFrame( _frame );
        g_cspaceTuner->setThreshedFrame( _threshed );
        g_cspaceTuner->setEqualizedFrameBef( _channel );
        g_cspaceTuner->setEqualizedFrameAfter( _eqChannel );
    }

    calibcv::CVideoHandler::release();
    calibcv::tuning::CColorSpaceTuner::release();

    return 0;
}
