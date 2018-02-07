

#include <opencv2/opencv.hpp>

#include "CUtils.h"
#include "CTuning.h"

using namespace std;

calibcv::CVideoHandler* g_videoHandler;
calibcv::tuning::CThreshHSVTuner* g_hsvTuner;

#define VIDEO_TEST_FILE "../res/calibration_ps3eyecam.avi"

int main()
{
    g_videoHandler = calibcv::CVideoHandler::create();
    
    if ( !g_videoHandler->openVideo( VIDEO_TEST_FILE ) )
    {
        cout << "couldnt open: " << VIDEO_TEST_FILE << endl;
        exit( -1 );
    }

    g_hsvTuner = calibcv::tuning::CThreshHSVTuner::create();

    while ( 1 )
    {
        cv::Mat _frame;
        cv::Mat _threshed;

        g_videoHandler->takeFrame( _frame );

        cv::inRange( _frame, g_hsvTuner->hsvMin(), g_hsvTuner->hsvMax(), _threshed );

        g_hsvTuner->setBaseFrame( _frame );
        g_hsvTuner->setThreshedFrame( _threshed );

        if( cv::waitKey( 30 ) >= 0 ) 
        {
            break;
        }
    }


    return 0;
}