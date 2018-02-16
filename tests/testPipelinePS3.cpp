
#include <SVideoHandler.h>
#include <panels/SPatternDetectorPanel.h>
#include <pipelines/SPatternDetectorPipeline.h>
#include <SUtils.h>

#include <opencv2/opencv.hpp>

#include <SVideoHandlerPSEye.h>

#define SAMPLE_TIME 16 // 50fps

int main()
{

    calibcv::SVideoHandlerPSEye* _videoHandler = calibcv::SVideoHandlerPSEye::create();

    if ( !_videoHandler->openCamera( 0 ) )
    {
        cout << "couldn't open webcam with deviceid: " << 0 << endl;
        exit( -1 );
    }

    auto _timer = calibcv::SCpuTimer::create();
    auto _panel = calibcv::SPatternDetectorPanel::create();
    auto _pipeline = new calibcv::SPatternDetectorPipeline();


    while( 1 )
    {
        cv::Mat _frame;
        float _captureTime = 0;
        // frame capture ******************************

        double timer = (double)cv::getTickCount();
        _timer->start();
        _videoHandler->takeFrame( _frame );
        _captureTime = _timer->stop();

        // pipeline ***********************************
        calibcv::SPipelineParams _params;
        _params.roi = _videoHandler->fixedROI();

        _pipeline->run( _frame, _params );
        // ********************************************

        float delta = ( ( double ) cv::getTickCount() - timer )/ cv::getTickFrequency();
        //cout << "delta: " << delta << endl;
        //cout << "totalCost: " << _pipeline->getTotalCost() + _captureTime << endl;
        _panel->setFPS( 1.0f / delta );

        int _key = cv::waitKey( SAMPLE_TIME ) & 0xff;

        if ( _key == KEY_SPACE )
        {
            _videoHandler->togglePause();
        }
        else if ( _key == KEY_ENTER )
        {
            _pipeline->reset();
        }
        else if ( _key == KEY_ESCAPE )
        {
            break;
        }

        free( _frame.data );
    }

    calibcv::SVideoHandler::release();
    _videoHandler = NULL;

    return 0;

}
