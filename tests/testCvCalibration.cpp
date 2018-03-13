
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

using namespace std;



int main( int argc, char** argv )
{
    // Parsing required data **************************************************************
    if ( argc < 2 )
    {
        cout << "Usage: ./testCvCalibration PATTERN" << endl;
        cout << "PATTERN: chessboard, rings, asymmetric_circles" << endl;
        return -1;
    }

    cv::FileStorage _fs;
    
    string _configFile = RESOURCES_PATH;
    _configFile += "calibrationConfig/config_";
    _configFile += argv[1];
    _configFile += ".yaml";

    _fs.open( _configFile, cv::FileStorage::READ );

    if ( !_fs.isOpened() )
    {
        cout << "error while reading file: " << _configFile << endl;
        return -1;
    }

    float _squareSpacing;
    int _patternType;
    int _patternSizeWidth, _patternSizeHeight;
    string _videoFile;
    string _calibrationFile;
    string _calibrationFileRefined;

    _fs[ "SquareSpacing" ] >> _squareSpacing;
    _fs[ "PatternType" ] >> _patternType;
    _fs[ "PatternSizeWidth" ] >> _patternSizeWidth;
    _fs[ "PatternSizeHeight" ] >> _patternSizeHeight;
    _fs[ "CalibrationVideo" ] >> _videoFile;
    _fs[ "CalibrationFile" ] >> _calibrationFile;
    _fs[ "CalibrationFileRefined" ] >> _calibrationFileRefined;

    _fs.release();
    // ************************************************************************************

    calibcv::SVideoHandler* _videoHandler = calibcv::SVideoHandler::create();

    string _videoFileStr = RESOURCES_PATH;
    _videoFileStr += "calibrationVideos/";
    _videoFileStr += _videoFile;
    
    if ( !_videoHandler->openVideo( _videoFileStr ) )
    {
        cout << "couldn't open videofile : " << _videoFileStr << endl;
        return -1;
    }

    cv::namedWindow( WINDOW_ORIGINAL_FRAME );
    cv::namedWindow( WINDOW_CORRECTED_FRAME );

    calibration::PatternInfo _patternInfo = { _patternType,
                                              _squareSpacing, 
                                              cv::Size( _patternSizeWidth, _patternSizeHeight ) };

    cv::Size _frameSize = _videoHandler->getVideoFrameSize();
    cout << "fw: " << _frameSize.width << " - fh: " << _frameSize.height << endl;

    calibration::Calibrator _calibrator( _frameSize, _patternInfo, _calibrationFile, _calibrationFileRefined );

    if ( !_calibrator.loadFromFile( _calibrationFile ) )
    {
        cout << "calibration file " << _calibrationFile << " not found, using new" << endl;
    }

    // _videoHandler->setPlaybackAtFrameIndex( 565 );
    // _videoHandler->togglePause();

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

        cv::Mat _frameCorrected;

        _calibrator.applyCalibrationCorrection( _frame, _frameCorrected );
        cv::imshow( WINDOW_CORRECTED_FRAME, _frameCorrected );

        _calibrator.update();

        cv::imshow( WINDOW_ORIGINAL_FRAME, _frame );

    }

    calibcv::SVideoHandler::release();
    _videoHandler = NULL;

    return 0;

}