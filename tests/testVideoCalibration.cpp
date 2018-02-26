
#include <SVideoHandler.h>
#include <panels/SPatternDetectorPanel.h>
#include <pipelines/SPatternDetectorPipeline.h>
#include <SUtils.h>

#include <opencv2/opencv.hpp>


#define WINDOW_ORIGINAL_FRAME "wOriginalFrame"
#define SAMPLE_TIME 15 // 30fps
#define VIDEO_FILE_ID "../res/calibration_ps3eyecam_checkerboard_4.avi"

int main()
{
    calibcv::SVideoHandler* _videoHandler = calibcv::SVideoHandler::create();

    if ( !_videoHandler->openVideo( VIDEO_FILE_ID ) )
    {
        cout << "couldn't open webcam with deviceid: " << VIDEO_FILE_ID << endl;
        exit( -1 );
    }

    cv::namedWindow( WINDOW_ORIGINAL_FRAME );
    cv::namedWindow( "Calibration Test" );

    auto _timer = calibcv::SCpuTimer::create();
    auto _panel = calibcv::SPatternDetectorPanel::create();
    auto _pipeline = new calibcv::SPatternDetectorPipeline();

    //_videoHandler->setPlaybackAtFrameIndex( 1 );
    //_videoHandler->togglePause();

    int numBoards = 30;
    int numSquares = 54;
    cv::Size board_sz = cv::Size(9, 6);

    vector< vector< cv::Point3f > > object_points;
    vector< vector< cv::Point2f > > image_points;
    vector< cv::Point2f > corners;
    int successes = 0;

    vector< cv::Point3f > obj;
    /*for( int j = 0; j < numSquares; j++ )
        obj.push_back( cv::Point3f( 24 * (j / 9), 24 * (j % 6), 0.0f ) );*/

    cv::Mat _frame;
    cv::Mat gray_image;
    _videoHandler->takeFrame( _frame );

    for( int i = 0; i < 6; i++ )
        for( int j = 0; j < 9; j++ )
            obj.push_back( cv::Point3f( 24 * i, 24 * j, 0.0f ) );



    while( successes<numBoards )
    {
        cv::cvtColor( _frame, gray_image, CV_BGR2GRAY );
        bool found = cv::findChessboardCorners( _frame, board_sz, corners,
                                                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );
        if(found)
        {
            //cv::cornerSubPix(gray_image, corners, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            cv::cornerSubPix( gray_image, corners, cv::Size(11,11),
                        cv::Size(-1,-1), cv::TermCriteria( cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1 ));
            cv::drawChessboardCorners(gray_image, board_sz, corners, found);
        }

        cv::imshow( WINDOW_ORIGINAL_FRAME, _frame );
        cv::imshow( "Calibration Test", gray_image );

        _videoHandler->takeFrame( _frame );
        int key = cv::waitKey(15) & 0xff;
        //cout<<key<<endl;
        if( key == KEY_ESCAPE )
            return 0;

        if( key == ' ' && found != 0 )
        {
            image_points.push_back(corners);
            object_points.push_back(obj);

            printf("Snap stored!\n");

            successes++;

            if( successes>=numBoards )
                break;
        }
    }
    cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);
    cv::Mat distCoeffs;
    vector<cv::Mat> rvecs;
    vector<cv::Mat> tvecs;

    //intrinsic.ptr<float>(0)[0] = _frame.size().width / _frame.size().height;
    intrinsic.ptr<float>(0)[0] = 1;
    intrinsic.ptr<float>(1)[1] = 1;
    double rms = cv::calibrateCamera(object_points, image_points, _frame.size(), intrinsic, distCoeffs, rvecs, tvecs);
    cout<<"Reprojection Error: "<<rms<<endl;
    cv::Mat imageUndistorted;
    while(1)
    {
        _videoHandler->takeFrame( _frame );
        cv::undistort( _frame, imageUndistorted, intrinsic, distCoeffs );

        cv::imshow(WINDOW_ORIGINAL_FRAME, _frame);
        cv::imshow("Calibration Test", imageUndistorted);
        int key = cv::waitKey(1);
        int _key = cv::waitKey( SAMPLE_TIME ) & 0xff;

        if ( _key == ' ' )
        {
            _videoHandler->togglePause();
        }
        else if ( _key == KEY_ESCAPE )
        {
            break;
        }
    }

    /*while( 1 )
    {
        float _totalTime;
        cv::Mat _frame;

        // frame capture ******************************

        _timer->start();
        double timer = (double)cv::getTickCount();
        _videoHandler->takeFrame( _frame );
        cv::imshow( WINDOW_ORIGINAL_FRAME, _frame );

        _totalTime = _timer->stop();



        // pipeline ***********************************
        /*calibcv::SPipelineParams _params;
        _params.roi = _videoHandler->fixedROI();

        _pipeline->run( _frame, _params );

        _totalTime += _pipeline->getTotalCost();*/



        /*float delta = ((double)cv::getTickCount() - timer)/cv::getTickFrequency();
        // cout << "delta: " << delta << endl;
        _panel->setFPS( 1.0f / delta );

        int _key = cv::waitKey( SAMPLE_TIME ) & 0xff;

        if ( _key == KEY_SPACE )
        {
            _videoHandler->togglePause();
        }
        else if ( _key == KEY_ENTER && _videoHandler->isPaused() )
        {
            _videoHandler->togglePickingROI();
        }
        else if ( _key == KEY_ESCAPE )
        {
            break;
        }

    }*/

    calibcv::SVideoHandler::release();
    _videoHandler = NULL;

    return 0;

}
