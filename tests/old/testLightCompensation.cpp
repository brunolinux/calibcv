
#include <SVideoHandler.h>

#include <opencv2/opencv.hpp>


#define WINDOW_ORIGINAL_FRAME "wOriginalFrame"
#define WINDOW_COMPENSATED_FRAME "wCompensatedFrame"
#define SAMPLE_TIME 16 // 60fps
#define CAMERA_DEVICE_ID 0

int main()
{

    calibcv::SVideoHandler* _videoHandler = calibcv::SVideoHandler::create();

    if ( !_videoHandler->openCamera( CAMERA_DEVICE_ID ) )
    {
        cout << "couldn't open webcam with deviceid: " << CAMERA_DEVICE_ID << endl;
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

        cv::Mat _frame;

        _videoHandler->takeFrame( _frame );

        cv::Mat lab_image;
        cv::cvtColor( _frame, lab_image, CV_BGR2Lab );

        // Extract the L channel
        std::vector<cv::Mat> lab_planes( 3 );
        cv::split( lab_image, lab_planes );  // now we have the L image in lab_planes[0]

        // apply the CLAHE algorithm to the L channel
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        clahe->setClipLimit( 4 );
        cv::Mat dst;
        clahe->apply( lab_planes[0], dst );

        // Merge the the color planes back into an Lab image
        dst.copyTo( lab_planes[0] );
        cv::merge( lab_planes, lab_image );

        // convert back to RGB
        cv::Mat image_clahe;
        cv::cvtColor( lab_image, image_clahe, CV_Lab2BGR );

        cv::imshow( WINDOW_ORIGINAL_FRAME, _frame );
        cv::imshow( WINDOW_COMPENSATED_FRAME, image_clahe);

        // _frame.deallocate();
        free( _frame.data );

    }

    calibcv::SVideoHandler::release();
    _videoHandler = NULL;

    return 0;

}
