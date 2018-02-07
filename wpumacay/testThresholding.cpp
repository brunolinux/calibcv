

#include <opencv2/opencv.hpp>

#include "CUtils.h"
#include "CTuning.h"

using namespace std;

calibcv::CVideoHandler* g_videoHandler;
calibcv::tuning::CThreshHSVTuner* g_hsvTuner;

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

    g_hsvTuner = calibcv::tuning::CThreshHSVTuner::create();

    while ( 1 )
    {
        int _key = cv::waitKey( SAMPLE_TIME );

        if ( _key == 1048608 )
        {
            g_videoHandler->togglePause();
        }
        else if( _key == 1048603 )
        {
            break;
        }

        cv::Mat _frame;
        cv::Mat _threshed;

        g_videoHandler->takeFrame( _frame );

        cv::inRange( _frame, g_hsvTuner->hsvMin(), g_hsvTuner->hsvMax(), _threshed );

        g_hsvTuner->setBaseFrame( _frame );
        g_hsvTuner->setThreshedFrame( _threshed );

    }

    calibcv::CVideoHandler::release();
    calibcv::tuning::CThreshHSVTuner::release();

    return 0;
}
