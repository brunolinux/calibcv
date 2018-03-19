
#include <SVideoHandler.h>
#include <calibrationInterface.h>
#include <calibrationViz.h>
#include <calibrationTool.h>

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

#ifndef RESOURCES_PATH
#define RESOURCES_PATH "../res/"
#endif

#define WINDOW_ORIGINAL_FRAME "wOriginalFrame"
#define WINDOW_CORRECTED_FRAME "wCorrectedFrame"
#define SAMPLE_TIME 33 // 30fps

using namespace std;

#define NUM_ITERATIONS 20

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

    int _currentIteration = 0;
    vector< float > _rmsErrors;
    vector< float > _colinearityErrors;

    string _iterationsFileName = _calibrationFile;
    _iterationsFileName += "_" + to_string( INITIAL_CALIBRATION_THRESHOLD_COUNT );
    _iterationsFileName += "_iterations_results.txt";

    ofstream _iterationsFile( _iterationsFileName.c_str() );

    if ( !_calibrator.canUseRefining() )
    {
        cout << "should have initial calibration data" << endl;
        return -1;
    }

    bool _shouldBeCalibrating = false;

    while( 1 )
    {

        int _key = cv::waitKey( SAMPLE_TIME ) & 0xff;

        // cout << "key: " << _key << endl;

        bool _pickCalibrationBucket = false;
        bool _requestRefinment = false;
        bool _requestRecalibration = false;

        if ( _key == KEY_SPACE )
        {
            _videoHandler->togglePause();
        }
        else if ( _key == KEY_P )
        {
            cout << "toggle roi picking" << endl;
            _videoHandler->togglePickingROI();
        }
        // else if ( _key == KEY_R )
        // {
        //     cout << "requesting refinment" << endl;
        //     _requestRefinment = true;
        // }
        // else if ( _key == KEY_ENTER )
        // {
        //     cout << "try to pick calibration bucket" << endl;
        //     _pickCalibrationBucket = true;
        // }
        // else if ( _key == KEY_C )
        // {
        //     cout << "requesting a recalibration with refined data" << endl;
        //     _requestRecalibration = true;
        // }
        else if ( _key == KEY_ESCAPE )
        {
            break;
        }

        cv::Mat _frame;

        _videoHandler->takeFrame( _frame );

        vector< cv::Point2f > _corners;

        calibration::DetectionInfo _detInfo;
        _detInfo.roi = _videoHandler->roi();

        if ( _currentIteration < NUM_ITERATIONS )
        {
            if ( !_calibrator.isCalibrating() )
            {
                // cout << "not claibrating: var -> " << _shouldBeCalibrating << endl;
                if ( _shouldBeCalibrating )
                {
                    _shouldBeCalibrating = false;
                    _currentIteration++;

                    cout << "finished iteration " << ( _currentIteration ) << endl;

                    _rmsErrors.push_back( _calibrator.getCurrentRMSerror() );
                    _colinearityErrors.push_back( _calibrator.getCurrentColinearityerror() );

                    if ( _currentIteration == NUM_ITERATIONS )
                    {
                        _iterationsFile << "iteration results: " << endl;
                        for ( int q = 0; q < _rmsErrors.size(); q++ )
                        {
                            _iterationsFile << "iteration " << ( q + 1 ) << "*******" << endl;
                            _iterationsFile << "rmsError: " << _rmsErrors[q] << endl;
                            _iterationsFile << "colinearityError: " << _colinearityErrors[q] << endl;
                            _iterationsFile << endl;
                        }

                        _iterationsFile.close();
                    }
                }

                if ( !calibration::isRefining( _patternInfo ) )
                {
                    // Grab necessary data from previous calibration
                    cv::Mat _cameraMatrix, _distortionCoefficients;

                    _calibrator.getCalibrationCameraMatrix( _cameraMatrix );
                    _calibrator.getCalibrationDistortionCoefficients( _distortionCoefficients );

                    // cout << "_cameraMatrix: " << _cameraMatrix << endl;
                    // cout << "_distortionCoefficients: " << _distortionCoefficients << endl;

                    // Request refinment to the detection interface
                    vector< cv::Mat > _batchImagesToRefine;
                    vector< vector< cv::Point2f > > _batchPointsToRefine;

                    _calibrator.getCalibrationBatch( _batchImagesToRefine,
                                                     _batchPointsToRefine );

                    calibration::requestBatchRefinment( _patternInfo,
                                                        _batchImagesToRefine,
                                                        _batchPointsToRefine,
                                                        _cameraMatrix,
                                                        _distortionCoefficients );
                }
                else
                {
                    // If refiner worker has finished, grab the bounty
                    // This should be self cleared in the method
                    if ( calibration::hasRefinationToPick( _patternInfo ) )
                    {
                        vector< cv::Mat > _refinedImages;
                        vector< vector< cv::Point2f > > _refinedPoints;

                        calibration::grabRefinationBatch( _patternInfo, _refinedImages, _refinedPoints );

                        // Send data back to calibrator to await next recalibration
                        _calibrator.addBatchRefinment( _refinedImages, _refinedPoints );

                        _calibrator.requestRefinedCalibration();

                        cout << "requested calibration" << endl;
                        _shouldBeCalibrating = true;
                    }
                }
            }
        }

        if ( calibration::getPatternCorners( _corners, _frame, _patternInfo, _detInfo ) )
        {
            calibration::drawPatternCorners( _corners, _frame, _patternInfo );
        }
            


        // Apply correction ********************************************************
        cv::Mat _frameCorrected;

        _calibrator.applyCalibrationCorrection( _frame, _frameCorrected );
        cv::imshow( WINDOW_CORRECTED_FRAME, _frameCorrected );

        calibration::update( _patternInfo );
        _calibrator.update();

        cv::imshow( WINDOW_ORIGINAL_FRAME, _frame );
        // *************************************************************************
    }

    calibcv::SVideoHandler::release();
    _videoHandler = NULL;
    
    return 0;

}