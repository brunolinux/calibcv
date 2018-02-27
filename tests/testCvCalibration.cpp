
#include <SVideoHandler.h>
#include "calibration/calibrationInterface.h"

#include <opencv2/opencv.hpp>

#ifndef RESOURCES_PATH
#define RESOURCES_PATH "../res/"
#endif

#define WINDOW_ORIGINAL_FRAME "wOriginalFrame"
#define SAMPLE_TIME 33 // 30fps
//#define VIDEO_FILE_ID "calibration_ps3eyecam_checkerboard_3.avi"
#define VIDEO_FILE_ID "calibration_ps3eyecam_checkerboard_1.avi"

#define TEST_PATTERN_TYPE 0
#define TEST_PATTERN_SQUARE_LENGTH 0.0235
#define TEST_PATTERN_SIZE cv::Size( 7, 10 )

using namespace std;



int main()
{

    calibcv::SVideoHandler* _videoHandler = calibcv::SVideoHandler::create();

    string _videoFileStr = RESOURCES_PATH;
    _videoFileStr += VIDEO_FILE_ID;
    
    if ( !_videoHandler->openVideo( _videoFileStr ) )
    {
        cout << "couldn't open videofile : " << _videoFileStr << endl;
        exit( -1 );
    }

    cv::namedWindow( WINDOW_ORIGINAL_FRAME );

    while( 1 )
    {

        int _key = cv::waitKey( SAMPLE_TIME ) & 0xff;

        if ( _key == KEY_SPACE )
        {
            _videoHandler->togglePause();
        }
        else if ( _key == KEY_ENTER )
        {
            _videoHandler->togglePickingROI();
        }
        else if ( _key == KEY_ESCAPE )
        {
            break;
        }

        cv::Mat _frame, _gray;

        _videoHandler->takeFrame( _frame );

        cv::cvtColor( _frame, _gray, CV_BGR2GRAY );

        vector< cv::Point2f > _corners;

        calibration::PatternInfo _pInfo = { calibration::PATTERN_TYPE_CHESSBOARD,
                                            0.025f, cv::Size( 9, 6 ) };

        if ( calibration::getPatternCorners( _corners, _gray, _pInfo ) )
        {
            calibration::drawPatternCorners( _corners, _frame, _pInfo );
        }


        cv::imshow( WINDOW_ORIGINAL_FRAME, _frame );

    }

    calibcv::SVideoHandler::release();
    _videoHandler = NULL;

    return 0;

}