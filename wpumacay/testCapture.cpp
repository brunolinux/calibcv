

#include <opencv2/opencv.hpp>

#include "CUtils.h"

using namespace std;

calibcv::CVideoHandler* g_videoHandler;

#define VIDEO_TEST_FILE "../res/calibration_ps3eyecam.avi"
#define WINDOW_ORIGINAL_FRAME "wOriginalFrame"

#define SAMPLE_TIME 33 // 30fps

int main()
{
    g_videoHandler = calibcv::CVideoHandler::create();

    if ( !g_videoHandler->openVideo( VIDEO_TEST_FILE ) )
    {
        cout << "couldnt open: " << VIDEO_TEST_FILE << endl;
        exit( -1 );
    }

    cv::namedWindow( WINDOW_ORIGINAL_FRAME );

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

        g_videoHandler->takeFrame( _frame );

        cv::imshow( WINDOW_ORIGINAL_FRAME, _frame );
    }

    calibcv::CVideoHandler::release();

    return 0;
}
