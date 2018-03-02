
#include <SVideoHandler.h>
#include <calibrationInterface.h>

#include <iostream>
#include <opencv2/opencv.hpp>

#ifndef RESOURCES_PATH
#define RESOURCES_PATH "../res/"
#endif

#define WINDOW_ORIGINAL_FRAME "wOriginalFrame"
#define WINDOW_CORRECTED_FRAME "wCorrectedFrame"
#define SAMPLE_TIME 33 // 30fps
// #define VIDEO_FILE_ID "calibration_ps3eyecam_checkerboard_3.avi"

// #define TEST_PATTERN_TYPE calibration::PATTERN_TYPE_CHESSBOARD
// #define TEST_PATTERN_SQUARE_LENGTH 0.0235
// #define TEST_PATTERN_SIZE cv::Size( 9, 6 )
// #define TEST_CALIBRATION_THRESHOLD_COUNT 30
// #define VIDEO_FILE_ID "calibration_test_chessboard_1.avi"
// #define TEST_CALIBRATION_SAVE_FILE "calibration_test_chessboard_1.yaml"

// #define TEST_PATTERN_TYPE calibration::PATTERN_TYPE_ASYMMETRIC_CIRCLES
// #define TEST_PATTERN_SQUARE_LENGTH 0.0367
// #define TEST_PATTERN_SIZE cv::Size( 4, 11 )
// #define TEST_CALIBRATION_THRESHOLD_COUNT 30
// #define VIDEO_FILE_ID "calibration_test_asymmetric_1.avi"
// #define TEST_CALIBRATION_SAVE_FILE "calibration_test_asymmetric_1.yaml"

// #define TEST_PATTERN_TYPE calibration::PATTERN_TYPE_CONCENTRIC_CIRCLES
// #define TEST_PATTERN_SQUARE_LENGTH 0.0235
// #define TEST_PATTERN_SIZE cv::Size( 5, 4 )
// #define TEST_CALIBRATION_THRESHOLD_COUNT 30
// #define VIDEO_FILE_ID "calibration_test_concentric_1.avi"
// #define TEST_CALIBRATION_SAVE_FILE "calibration_test_concentric_1.yaml"

#define TEST_PATTERN_TYPE calibration::PATTERN_TYPE_CONCENTRIC_CIRCLES
#define TEST_PATTERN_SQUARE_LENGTH 0.0235
#define TEST_PATTERN_SIZE cv::Size( 5, 4 )
#define TEST_CALIBRATION_THRESHOLD_COUNT 30
#define VIDEO_FILE_ID "calibrationVideos/LifeCam/calib_rings_1.avi"
#define TEST_CALIBRATION_SAVE_FILE "calibration_test_concentric_2.yaml"

// #define TEST_PATTERN_TYPE calibration::PATTERN_TYPE_CONCENTRIC_CIRCLES
// #define TEST_PATTERN_SQUARE_LENGTH 0.0235
// #define TEST_PATTERN_SIZE cv::Size( 5, 4 )
// #define TEST_CALIBRATION_THRESHOLD_COUNT 30
// #define VIDEO_FILE_ID "calibrationVideos/Camera_Play_3/calib_ps3_concentric.avi"
// #define TEST_CALIBRATION_SAVE_FILE "calibration_test_concentric_3.yaml"

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

    calibration::PatternInfo _patternInfo = { TEST_PATTERN_TYPE,
                                              TEST_PATTERN_SQUARE_LENGTH, 
                                              TEST_PATTERN_SIZE };

    cv::Size _frameSize = _videoHandler->getVideoFrameSize();
    cout << "fw: " << _frameSize.width << " - fh: " << _frameSize.height << endl;

    calibration::Calibrator _calibrator( _frameSize, _patternInfo );

    if ( !_calibrator.loadFromFile( TEST_CALIBRATION_SAVE_FILE ) )
    {
        cout << "calibration file " << TEST_CALIBRATION_SAVE_FILE << " not found, using new" << endl;
    }

    while( 1 )
    {

        int _key = cv::waitKey( SAMPLE_TIME ) & 0xff;

        // cout << "key: " << _key << endl;

        bool _pickCalibrationBucket = false;

        if ( _key == KEY_SPACE )
        {
            _videoHandler->togglePause();
        }
        else if ( _key == KEY_R )
        {
            _videoHandler->togglePickingROI();
        }
        else if ( _key == KEY_ENTER )
        {
            cout << "try to pick calibration bucket" << endl;
            _pickCalibrationBucket = true;
        }
        else if ( _key == KEY_ESCAPE )
        {
            break;
        }

        cv::Mat _frame;

        _videoHandler->takeFrame( _frame );

        vector< cv::Point2f > _corners;

        calibration::DetectionInfo _detInfo;
        _detInfo.roi = _videoHandler->roi();

        if ( calibration::getPatternCorners( _corners, _frame, _patternInfo, _detInfo ) )
        {
            calibration::drawPatternCorners( _corners, _frame, _patternInfo );

            if ( _pickCalibrationBucket )
            {
                cout << "current buckets in calibrator: " << _calibrator.getCalibrationSize() << endl;
                _calibrator.addCalibrationBucket( _frame, _corners );
            }
        }

        if ( !_calibrator.isCalibrated() && 
             _calibrator.getCalibrationSize() >= TEST_CALIBRATION_THRESHOLD_COUNT )
        {
            cout << "Calibrating ......" << endl;
            _calibrator.calibrate();
            cout << "DONE calibrating" << endl;

            _calibrator.saveToFile( TEST_CALIBRATION_SAVE_FILE );
            
            cv::namedWindow( WINDOW_CORRECTED_FRAME );
        }

        cv::Mat _frameCorrected;

        if ( _calibrator.isCalibrated() )
        {
            _calibrator.applyCalibrationCorrection( _frame, _frameCorrected );
            cv::imshow( WINDOW_CORRECTED_FRAME, _frameCorrected );
        }

        cv::imshow( WINDOW_ORIGINAL_FRAME, _frame );

    }

    calibcv::SVideoHandler::release();
    _videoHandler = NULL;

    return 0;

}