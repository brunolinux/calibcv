
#include <SVideoHandler.h>
#include <panels/SPatternDetectorPanel.h>
#include <pipelines/SPatternDetectorPipeline.h>
#include <SUtils.h>

#include <opencv2/opencv.hpp>

#define SAMPLE_TIME 16 // 60fps
#define CAMERA_ID 0

int main()
{

    calibcv::SVideoHandler* _videoHandler = calibcv::SVideoHandler::create();

    if ( !_videoHandler->openCamera( CAMERA_ID ) )
    {
        cout << "couldn't open webcam with deviceid: " << CAMERA_ID << endl;
        exit( -1 );
    }

    auto _timer = calibcv::SCpuTimer::create();
    auto _panel = calibcv::SPatternDetectorPanel::create();
    auto _pipeline = new calibcv::SPatternDetectorPipeline();

    while( 1 )
    {
        cv::Mat _frame;

        // frame capture ******************************
        double timer = (double)cv::getTickCount();
        _videoHandler->takeFrame( _frame );

        // pipeline ***********************************
        calibcv::SPipelineParams _params;
        _params.roi = _videoHandler->fixedROI();

        _pipeline->run( _frame, _params );

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
