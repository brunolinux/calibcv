
#include <SVideoHandler.h>
#include <panels/SPatternDetectorPanel.h>
#include <pipelines/SPatternDetectorPipeline.h>
#include <SUtils.h>

#include <opencv2/opencv.hpp>


#define WINDOW_ORIGINAL_FRAME "wOriginalFrame"
#define SAMPLE_TIME 15 // 30fps
#define VIDEO_FILE_ID "../res/calibration_ps3eyecam.avi"

int main()
{

    calibcv::SVideoHandler* _videoHandler = calibcv::SVideoHandler::create();

    if ( !_videoHandler->openVideo( VIDEO_FILE_ID ) )
    {
        cout << "couldn't open webcam with deviceid: " << VIDEO_FILE_ID << endl;
        exit( -1 );
    }

    cv::namedWindow( WINDOW_ORIGINAL_FRAME );

    auto _timer = calibcv::SCpuTimer::create();
    auto _panel = calibcv::SPatternDetectorPanel::create();
    auto _pipeline = new calibcv::SPatternDetectorPipeline();

    _videoHandler->setPlaybackAtFrameIndex( 1 );
    _videoHandler->togglePause();

    while( 1 )
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
        calibcv::SPipelineParams _params;
        _params.roi = _videoHandler->fixedROI();

        _pipeline->run( _frame, _params );

        _totalTime += _pipeline->getTotalCost();

        float delta = ((double)cv::getTickCount() - timer)/cv::getTickFrequency();
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

    }

    calibcv::SVideoHandler::release();
    _videoHandler = NULL;

    return 0;

}
