
#include <SVideoHandlerPSEye.h>

#include <opencv2/opencv.hpp>


#define WINDOW_ORIGINAL_FRAME "wOriginalFrame"
#define SAMPLE_TIME 10 // 60fps
#define CAMERA_DEVICE_ID 0

int main()
{

    calibcv::SVideoHandlerPSEye* _videoHandler = calibcv::SVideoHandlerPSEye::create();

    if ( !_videoHandler->openCamera( CAMERA_DEVICE_ID ) )
    {
        cout << "couldn't open webcam with deviceid: " << CAMERA_DEVICE_ID << endl;
        exit( -1 );
    }

    cv::namedWindow( WINDOW_ORIGINAL_FRAME );

    while( 1 )
    {

        int _key = cv::waitKey( 1 ) & 0xff;

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

        cv::Mat _frame;

        _videoHandler->takeFrame( _frame );



        cv::imshow( WINDOW_ORIGINAL_FRAME, _frame );

        // _frame.deallocate();
        free( _frame.data );

    }

    calibcv::SVideoHandler::release();
    _videoHandler = NULL;

    return 0;

}
