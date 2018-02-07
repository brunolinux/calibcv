

#include <opencv2/opencv.hpp>

#include "CUtils.h"

using namespace std;

calibcv::CVideoHandler* g_videoHandler;

#define VIDEO_TEST_FILE "../res/calibration_ps3eyecam.avi"
#define WINDOW_ORIGINAL_FRAME "wOriginalFrame"

int main()
{
    g_videoHandler = calibcv::CVideoHandler::create();

    if ( !g_videoHandler->openVideo( VIDEO_TEST_FILE ) )
    {
        cout << "couldnt open: " << VIDEO_TEST_FILE << endl;
        exit( -1 );
    }

    cv::namedWindow( WINDOW_ORIGINAL_FRAME, 1 );

    while ( 1 )
    {
        cv::Mat _frame;

        g_videoHandler->takeFrame( _frame );

        cv::imshow( WINDOW_ORIGINAL_FRAME, _frame );

        if( cv::waitKey( 30 ) >= 0 ) 
        {
            break;
        }
    }


    return 0;
}