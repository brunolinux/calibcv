
#pragma once

#include "calibrationInterface.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <pthread.h>

#define INITIAL_CALIBRATION_THRESHOLD_COUNT 30
#define REFINED_CALIBRATION_THRESHOLD_COUNT 30
#define CALIBRATION_FOLDER "calib_default"

using namespace std;

namespace calibration
{


    class Calibrator
    {

        private :

	    enum
	    {
	    	CALIB_STATE_UNCALIBRATED,
	    	CALIB_STATE_CALIBRATING,
	        CALIB_STATE_CALIBRATED_SIMPLE,
	        CALIB_STATE_CALIBRATED_REFINED
	    };

        enum
        {
            USE_MODE_NONE,
            USE_MODE_INITIAL,
            USE_MODE_REFINED
        };

        // TODO: Refactor this part ************************************************
        // Info from each calibration bucket
        vector< cv::Mat > m_calibrationImagesInitial;
        vector< cv::Mat > m_calibrationRotMatricesInitial;
        vector< cv::Mat > m_calibrationTranMatricesInitial;
        vector< vector< cv::Point2f > > m_pointsInImageInitial;
        vector< vector< cv::Point3f > > m_pointsInWorldInitial;

        cv::Mat m_cameraMatrixInitial;
        cv::Mat m_distortionCoefficientsInitial;
        cv::Mat m_transformationMap1Initial;
        cv::Mat m_transformationMap2Initial;

        // Info from each calibration bucket in the refining step
        vector< cv::Mat > m_calibrationImagesRefining;
        vector< cv::Mat > m_calibrationRotMatricesRefining;
        vector< cv::Mat > m_calibrationTranMatricesRefining;
        vector< vector< cv::Point2f > > m_pointsInImageRefining;
        vector< vector< cv::Point3f > > m_pointsInWorldRefining;

        cv::Mat m_cameraMatrixRefined;
        cv::Mat m_distortionCoefficientsRefined;
        cv::Mat m_transformationMap1Refined;
        cv::Mat m_transformationMap2Refined;

        // Use these working structures in the calibration thread
        cv::Mat m_transformationMap1Working;
        cv::Mat m_transformationMap2Working;
        cv::Mat m_cameraMatrixWorking;
        cv::Mat m_distortionCoefficientsWorking;        
        vector< cv::Mat > m_calibrationRotMatricesWorking;
        vector< cv::Mat > m_calibrationTranMatricesWorking;
        // *************************************************************************

        // State the calibration is in
        int m_calibState;
        int m_calibStateOld;
        // Used to know if the calibration thread is working
        bool m_isCalibrating;
        // To check which calibration mode to use in when applying correction
        int m_useMode;
        // Handle to the thread to work with
        pthread_t m_threadHandle;

        cv::Size m_frameSize;
        PatternInfo m_patternInfo;
        int m_calibInitThresholdCount;
        int m_calibRefiningThresholdCount;

        string m_calibFolder;
        string m_calibSaveFile;
        string m_calibSaveFileRefined;

        float m_calibrationRMSerror;
        float m_calibrationReprojectionError;
        float m_calibrationNewColinearityError;
        float m_calibrationOldColinearityError;

        vector< float > m_perViewErrors;

        DistributionVisualizer* m_visualizer;

        public :

        Calibrator( const cv::Size& frameSize, const PatternInfo& patternInfo,
        			string calibSaveFile, string calibSaveFileRefined, string calibFolder = CALIBRATION_FOLDER,
                    int calibrationInitialThresholdCount = INITIAL_CALIBRATION_THRESHOLD_COUNT,
                    int calibrationRefiningThresholdCount = REFINED_CALIBRATION_THRESHOLD_COUNT )
        {
            m_frameSize = frameSize;
            m_patternInfo = patternInfo;
            m_calibFolder = calibFolder;
            m_calibSaveFile = calibSaveFile;
            m_calibSaveFileRefined = calibSaveFileRefined;
            m_calibInitThresholdCount = calibrationInitialThresholdCount;
            m_calibRefiningThresholdCount = calibrationRefiningThresholdCount;

            m_calibState = CALIB_STATE_UNCALIBRATED;
            m_calibStateOld = CALIB_STATE_UNCALIBRATED;
            m_isCalibrating = false;
            m_useMode = USE_MODE_NONE;

            m_visualizer = new DistributionVisualizer( "viz",
                                                       m_patternInfo.size,
                                                       m_frameSize.width, m_frameSize.height );

            init();
        }

        ~Calibrator()
        {
            m_calibrationImagesInitial.clear();
            m_pointsInImageInitial.clear();
            m_pointsInWorldInitial.clear();

            m_calibrationImagesRefining.clear();
            m_pointsInImageRefining.clear();
            m_pointsInWorldRefining.clear();
        }

        void init()
        {
            m_calibrationImagesInitial.clear();
            m_pointsInImageInitial.clear();
            m_pointsInWorldInitial.clear();
            m_calibrationRotMatricesInitial.clear();
            m_calibrationTranMatricesInitial.clear();

            m_calibrationImagesRefining.clear();
            m_pointsInImageRefining.clear();
            m_pointsInWorldRefining.clear();
            m_calibrationRotMatricesRefining.clear();
            m_calibrationTranMatricesRefining.clear();

            m_perViewErrors.clear();

            m_calibState = CALIB_STATE_UNCALIBRATED;
            m_calibStateOld = CALIB_STATE_UNCALIBRATED;
            m_isCalibrating = false;
            m_useMode = USE_MODE_NONE;
        }

        void addCalibrationBucket( const cv::Mat& image,
                                   const vector< cv::Point2f >& corners2D )
        {
        	// For now, just accept frames and corners if able to calibrate ...
        	// ( not in calibrating state, because still working )

            // TODO: For now, no need for mutex, as only in the non-calibrating state ...
            // we can modify some of these vectors
            if ( m_calibState == CALIB_STATE_UNCALIBRATED )
            {
                m_calibrationImagesInitial.push_back( image );
                m_pointsInImageInitial.push_back( corners2D );

                m_visualizer->processCalibrationBucket( corners2D );
                m_visualizer->addFrameInitial( image );

                if ( m_calibrationImagesInitial.size() >= m_calibInitThresholdCount )
                {
                    cout << "Initial calibration started ........." << endl;

                    m_calibStateOld = m_calibState;
                    m_calibState = CALIB_STATE_CALIBRATING;
                    m_isCalibrating = true;

                    calibrateInitial();
                }
            }
            else if ( m_calibState == CALIB_STATE_CALIBRATED_SIMPLE ||
                      m_calibState == CALIB_STATE_CALIBRATED_REFINED )
            {
                m_calibrationImagesRefining.push_back( image );
                m_pointsInImageRefining.push_back( corners2D );

                m_visualizer->processCalibrationBucket( corners2D );
                m_visualizer->addFrameRefined( image );

                // The refined calibration will be sent by the application, not threshold base

                // if ( m_calibrationImagesRefining.size() >= m_calibRefiningThresholdCount )
                // {
                //     cout << "Refined calibration started ........." << endl;

                //     m_calibStateOld = m_calibState;
                //     m_calibState = CALIB_STATE_CALIBRATING;
                //     m_isCalibrating = true;
                    
                //     calibrateRefined();
                // }
            }
            else
            {
                cout << "something went wrong" << endl;
                cout << "state: " << m_calibState << endl;
                cout << "stateOld: " << m_calibStateOld << endl;
            }

        }

        void update()
        {
        	// check if still working
        	if ( m_calibState == CALIB_STATE_CALIBRATING )
        	{
        		if ( m_isCalibrating == false )
        		{
                    cout << "finished calibrating" << endl;
                    pthread_join( m_threadHandle, NULL );

                    // Get the maps back from the working-buffers
                    if ( m_calibStateOld == CALIB_STATE_UNCALIBRATED )
                    {
                        m_transformationMap1Initial      = m_transformationMap1Working.clone();
                        m_transformationMap2Initial      = m_transformationMap2Working.clone();
                        m_cameraMatrixInitial            = m_cameraMatrixWorking.clone();
                        m_distortionCoefficientsInitial  = m_distortionCoefficientsWorking.clone();
                        m_calibrationRotMatricesInitial  = m_calibrationRotMatricesWorking;
                        m_calibrationTranMatricesInitial = m_calibrationTranMatricesWorking;

                        m_visualizer->addCalibratedBucket( m_calibrationImagesInitial, m_perViewErrors, VIZ_CALIB_TYPE_SIMPLE );

                        // Keep this data as initial, as we are going to need it after when ...
                        // the application requests a batch to refine

                        // m_calibrationImagesInitial.clear();
                        // m_calibrationRotMatricesInitial.clear();
                        // m_calibrationTranMatricesInitial.clear();
                        // m_perViewErrors.clear();
                    }
                    else if ( m_calibStateOld == CALIB_STATE_CALIBRATED_SIMPLE ||
                              m_calibStateOld == CALIB_STATE_CALIBRATED_REFINED )
                    {
                        m_transformationMap1Refined      = m_transformationMap1Working.clone();
                        m_transformationMap2Refined      = m_transformationMap2Working.clone();
                        m_cameraMatrixRefined            = m_cameraMatrixWorking.clone();
                        m_distortionCoefficientsRefined  = m_distortionCoefficientsWorking.clone();
                        m_calibrationRotMatricesRefining  = m_calibrationRotMatricesWorking;
                        m_calibrationTranMatricesRefining = m_calibrationTranMatricesWorking;

                        m_visualizer->addCalibratedBucket( m_calibrationImagesRefining, m_perViewErrors, VIZ_CALIB_TYPE_REFINED );

                        // Set new calib state, refined data is now initial data
                        m_calibrationImagesInitial = m_calibrationImagesRefining;
                        m_calibrationRotMatricesInitial = m_calibrationRotMatricesRefining;
                        m_calibrationTranMatricesInitial = m_calibrationTranMatricesRefining;

                        // Clear now, as we are awaiting for new refining data
                        m_calibrationImagesRefining.clear();
                        m_calibrationRotMatricesRefining.clear();
                        m_calibrationTranMatricesRefining.clear();
                        m_perViewErrors.clear();

                    }

        			// Get to the next state, accordingly
        			if ( m_calibStateOld == CALIB_STATE_UNCALIBRATED )
        			{
        				m_calibState = m_calibStateOld = CALIB_STATE_CALIBRATED_SIMPLE;
                        m_useMode = USE_MODE_INITIAL;
        			}
        			else if ( m_calibStateOld == CALIB_STATE_CALIBRATED_SIMPLE )
        			{
        				m_calibState = m_calibStateOld = CALIB_STATE_CALIBRATED_REFINED;
                        m_useMode = USE_MODE_REFINED;
        			}
        			else if ( m_calibStateOld == CALIB_STATE_CALIBRATED_REFINED )
        			{
        				// Just stay here, this is the last state we should keep
        				m_calibState = m_calibStateOld = CALIB_STATE_CALIBRATED_REFINED;
                        m_useMode = USE_MODE_REFINED;
        			}
        		}
        	}

            m_visualizer->draw();
        }

        void calibrateInitial()
        {
            m_pointsInWorldInitial = vector< vector< cv::Point3f > >( 1 );
            getPatternKnownPlanePositions( m_pointsInWorldInitial[0], m_patternInfo );
            m_pointsInWorldInitial.resize( m_pointsInImageInitial.size(), m_pointsInWorldInitial[0] );

            pthread_create( &m_threadHandle, NULL, Calibrator::calibrateInitialWorker, ( void* ) this );
        }

        void calibrateRefined()
        {
            m_pointsInWorldRefining = vector< vector< cv::Point3f > >( 1 );
            getPatternKnownPlanePositions( m_pointsInWorldRefining[0], m_patternInfo );
            m_pointsInWorldRefining.resize( m_pointsInImageRefining.size(), m_pointsInWorldRefining[0] );            

            pthread_create( &m_threadHandle, NULL, Calibrator::calibrateRefinedWorker, ( void* ) this );
        }

        static void* calibrateInitialWorker( void* pCalibrator )
        {
        	Calibrator* _calibrator = ( Calibrator* ) pCalibrator;

            // TODO: Refactor. Quite dirty at the moment
            _calibrator->m_calibrationRMSerror = cv::calibrateCamera( _calibrator->m_pointsInWorldInitial,
                                                                      _calibrator->m_pointsInImageInitial,
                                                                      _calibrator->m_frameSize,
                                                                      _calibrator->m_cameraMatrixWorking,
                                                                      _calibrator->m_distortionCoefficientsWorking,
                                                                      _calibrator->m_calibrationRotMatricesWorking,
                                                                      _calibrator->m_calibrationTranMatricesWorking );

            cv::initUndistortRectifyMap( _calibrator->m_cameraMatrixWorking, 
                                         _calibrator->m_distortionCoefficientsWorking, 
                                         cv::Mat(), cv::Mat(), 
                                         _calibrator->m_frameSize,
                                         CV_32FC1, 
                                         _calibrator->m_transformationMap1Working, 
                                         _calibrator->m_transformationMap2Working );

            utils::computeReprojectionErrors( _calibrator->m_cameraMatrixWorking, 
                                              _calibrator->m_distortionCoefficientsWorking, 
                                              _calibrator->m_pointsInWorldInitial, 
                                              _calibrator->m_pointsInImageInitial, 
                                              _calibrator->m_calibrationRotMatricesWorking, 
                                              _calibrator->m_calibrationTranMatricesWorking, 
                                              _calibrator->m_calibrationReprojectionError,
                                              _calibrator->m_perViewErrors );

            utils::computeColinearityErrors( _calibrator->m_cameraMatrixWorking, 
                                             _calibrator->m_distortionCoefficientsWorking, 
                                             _calibrator->m_pointsInWorldInitial, 
                                             _calibrator->m_pointsInImageInitial, 
                                             _calibrator->m_calibrationRotMatricesWorking, 
                                             _calibrator->m_calibrationTranMatricesWorking, 
                                             _calibrator->m_calibrationOldColinearityError,
                                             _calibrator->m_calibrationNewColinearityError );

            _calibrator->saveCalibrationImages( _calibrator->m_calibrationImagesInitial, VIZ_CALIB_TYPE_SIMPLE );
            _calibrator->saveToFile( _calibrator->m_calibSaveFile, VIZ_CALIB_TYPE_SIMPLE );
            _calibrator->m_isCalibrating = false;
        }

        static void* calibrateRefinedWorker( void* pCalibrator )
        {
            Calibrator* _calibrator = ( Calibrator* ) pCalibrator;

            // TODO: Refactor. Quite dirty at the moment
            _calibrator->m_calibrationRMSerror = cv::calibrateCamera( _calibrator->m_pointsInWorldRefining,
                                                                      _calibrator->m_pointsInImageRefining,
                                                                      _calibrator->m_frameSize,
                                                                      _calibrator->m_cameraMatrixWorking,
                                                                      _calibrator->m_distortionCoefficientsWorking,
                                                                      _calibrator->m_calibrationRotMatricesWorking,
                                                                      _calibrator->m_calibrationTranMatricesWorking );
            // TODO: Check if change to undistort is necessary
            cv::initUndistortRectifyMap( _calibrator->m_cameraMatrixWorking, 
                                         _calibrator->m_distortionCoefficientsWorking, 
                                         cv::Mat(), cv::Mat(), 
                                         _calibrator->m_frameSize,
                                         CV_32FC1, 
                                         _calibrator->m_transformationMap1Working, 
                                         _calibrator->m_transformationMap2Working );

            utils::computeReprojectionErrors( _calibrator->m_cameraMatrixWorking, 
                                              _calibrator->m_distortionCoefficientsWorking, 
                                              _calibrator->m_pointsInWorldRefining, 
                                              _calibrator->m_pointsInImageRefining, 
                                              _calibrator->m_calibrationRotMatricesWorking, 
                                              _calibrator->m_calibrationTranMatricesWorking, 
                                              _calibrator->m_calibrationReprojectionError,
                                              _calibrator->m_perViewErrors );

            utils::computeColinearityErrors( _calibrator->m_cameraMatrixWorking, 
                                             _calibrator->m_distortionCoefficientsWorking, 
                                             _calibrator->m_pointsInWorldRefining, 
                                             _calibrator->m_pointsInImageRefining, 
                                             _calibrator->m_calibrationRotMatricesWorking, 
                                             _calibrator->m_calibrationTranMatricesWorking, 
                                             _calibrator->m_calibrationOldColinearityError,
                                             _calibrator->m_calibrationNewColinearityError );

            _calibrator->saveCalibrationImages( _calibrator->m_calibrationImagesRefining, VIZ_CALIB_TYPE_REFINED );
            _calibrator->saveToFile( _calibrator->m_calibSaveFileRefined, VIZ_CALIB_TYPE_REFINED );
            _calibrator->m_isCalibrating = false;
        }

        void applyCalibrationCorrection( const cv::Mat& src,
                                         cv::Mat& dst )
        {
            if ( m_calibState == CALIB_STATE_UNCALIBRATED ||
                 m_calibStateOld == CALIB_STATE_UNCALIBRATED )
            {
                // not calibrated yet, returning same image
                dst = src.clone();
                return;
            }

            if ( m_useMode == USE_MODE_NONE )
            {
                dst = src.clone();
            }
            else if ( m_useMode == USE_MODE_INITIAL )
            {
                // Use the initial calibration data
                cv::remap( src, dst, m_transformationMap1Initial, m_transformationMap2Initial, cv::INTER_LINEAR );
            }
            else if ( m_useMode == USE_MODE_REFINED )
            {
                if ( m_calibState == CALIB_STATE_CALIBRATED_REFINED ||
                     m_calibStateOld == CALIB_STATE_CALIBRATED_REFINED )
                {
                    cv::remap( src, dst, m_transformationMap1Refined, m_transformationMap2Refined, cv::INTER_LINEAR );
                }
                else
                {
                    cv::remap( src, dst, m_transformationMap1Initial, m_transformationMap2Initial, cv::INTER_LINEAR );
                }
            }
            
        }

        void saveToFile( string filename, int calibType )
        {
            cv::FileStorage _fs( filename, cv::FileStorage::WRITE );
 
            _fs << TAG_FRAME_WIDTH << m_frameSize.width;
            _fs << TAG_FRAME_HEIGHT << m_frameSize.height;
            _fs << TAG_CAMERA_MATRIX << m_cameraMatrixWorking;
            _fs << TAG_DISTORTION_COEFFICIENTS << m_distortionCoefficientsWorking;
            _fs << TAG_CALIBRATION_ERROR_RMS << m_calibrationRMSerror;
            _fs << TAG_CALIBRATION_ERROR_REPROJECTION << m_calibrationReprojectionError;
            _fs << TAG_CALIBRATION_ERROR_COLINEARITY_OLD << m_calibrationOldColinearityError;
            _fs << TAG_CALIBRATION_ERROR_COLINEARITY_NEW << m_calibrationNewColinearityError;
            
            _fs.release();

            m_visualizer->saveToFile();
        }

        bool loadFromFile( string filename )
        {
            cv::FileStorage _fs;
            
            _fs.open( filename, cv::FileStorage::READ );
            
            if ( !_fs.isOpened() )
            {
                std::cout << "couldn't find calibration file: " << filename << std::endl;
                return false;
            }

            init();
            
            _fs[ TAG_CAMERA_MATRIX ] >> m_cameraMatrixInitial;
            _fs[ TAG_DISTORTION_COEFFICIENTS ] >> m_distortionCoefficientsInitial;
            
            int _fw, _fh;
            _fs[ TAG_FRAME_WIDTH ] >> _fw;
            _fs[ TAG_FRAME_HEIGHT ] >> _fh;
            
            _fs.release();
            
            if ( m_frameSize.width != _fw ||
                 m_frameSize.height != _fh )
            {
                std::cout << "WARNING: Size from previous calibration does not match" << std::endl;
            }
            
            cv::initUndistortRectifyMap( m_cameraMatrixInitial, m_distortionCoefficientsInitial, 
                                         cv::Mat(), cv::Mat(), 
                                         m_frameSize,
                                         CV_32FC1, m_transformationMap1Initial, m_transformationMap2Initial );
            
            m_calibState = CALIB_STATE_CALIBRATED_SIMPLE;
            m_calibStateOld = CALIB_STATE_CALIBRATED_SIMPLE;
            m_useMode = USE_MODE_INITIAL;
            
            return true;
        }

        void saveCalibrationImages( const vector< cv::Mat >& images, int calibType )
        {
        	// create calibration directory **************************
        	string _pathSaveFolder = "./" + m_calibFolder;
            _pathSaveFolder += ( calibType == VIZ_CALIB_TYPE_SIMPLE ) ? "_simple" : "_refined";

        	mode_t _nMode = 0733;// permissions in unix
        	int _error = 0;

        #ifdef _WIN32
        	_error = _mkdir( _pathSaveFolder.c_str() );
        #else
        	_error = mkdir( _pathSaveFolder.c_str(), _nMode );
        #endif
        	if ( _error != 0 )
        	{
        		cout << "something went wrong while making the directory, maybe it already exists" << endl;
        	}
        	// *******************************************************

        	// save images to calibration directory ******************

        	for ( int q = 0; q < images.size(); q++ )
        	{
        		string _imgSavePath = _pathSaveFolder + "/img_";
        		_imgSavePath += to_string( q + 1 );
                _imgSavePath += ".jpg";

        		cv::imwrite( _imgSavePath, images[q] );
        	}
        }

        void getCalibrationBatch( vector< cv::Mat >& batchImagesToRefine,
                                  vector< vector< cv::Point2f > >& batchPointsToRefine )
        {
            batchImagesToRefine.clear();
            batchPointsToRefine.clear();

            for ( int q = 0; q < m_calibrationImagesInitial.size(); q++ )
            {
                batchImagesToRefine.push_back( m_calibrationImagesInitial[q] );
            }

            for ( int q = 0; q < m_pointsInImageInitial.size(); q++ )
            {
                batchPointsToRefine.push_back( m_pointsInImageInitial[q] );
            }

            assert( batchImagesToRefine.size() == batchPointsToRefine.size() );
        }

        void addBatchRefinment( vector< cv::Mat >& batchRefinedImages,
                                vector< vector< cv::Point2f > >& batchRefinedPoints )
        {
            assert( batchRefinedImages.size() == batchBuckets.size() );

            for ( int q = 0; q < batchRefinedImages.size(); q++ )
            {
                addCalibrationBucket( batchRefinedImages[q], batchRefinedPoints[q] );
            }
        }

        void requestRefinedCalibration()
        {
            if ( m_isCalibrating ||
                 m_calibState == CALIB_STATE_CALIBRATING ||
                 m_calibStateOld == CALIB_STATE_CALIBRATING )
            {
                return;
            }

            if ( m_calibrationImagesRefining.size() < m_calibRefiningThresholdCount )
            {
                cout << "not enough images for refined calibration" << endl;
                return;
            }

            m_calibStateOld = m_calibState;
            m_calibState = CALIB_STATE_CALIBRATING;
            m_isCalibrating = true;
                    
            calibrateRefined();            
        }

        int getCalibrationSize() 
        { 
            if ( m_calibState == CALIB_STATE_UNCALIBRATED ||
                 m_calibStateOld == CALIB_STATE_UNCALIBRATED )
            {            
                return m_calibrationImagesInitial.size(); 
            }
            else if ( m_calibState == CALIB_STATE_CALIBRATED_SIMPLE ||
                      m_calibStateOld == CALIB_STATE_CALIBRATED_SIMPLE ||
                      m_calibState == CALIB_STATE_CALIBRATED_REFINED ||
                      m_calibStateOld == CALIB_STATE_CALIBRATED_REFINED )
            {
                return m_calibrationImagesRefining.size();
            }

            return -1;
        }

        void getCalibrationCameraMatrix( cv::Mat& cameraMatrix ) 
        { 
            if ( m_calibState == CALIB_STATE_CALIBRATED_SIMPLE ||
                 m_calibStateOld == CALIB_STATE_CALIBRATED_SIMPLE )
            {
                cameraMatrix = m_cameraMatrixInitial.clone();
            }
            else if ( m_calibState == CALIB_STATE_CALIBRATED_REFINED ||
                      m_calibStateOld == CALIB_STATE_CALIBRATED_REFINED )
            {
                cameraMatrix = m_cameraMatrixRefined.clone();
            }
        }

        void getCalibrationDistortionCoefficients( cv::Mat& distortionCoefficients )
        {
            if ( m_calibState == CALIB_STATE_CALIBRATED_SIMPLE ||
                 m_calibStateOld == CALIB_STATE_CALIBRATED_SIMPLE )
            {
                distortionCoefficients = m_distortionCoefficientsInitial.clone();
            }
            else if ( m_calibState == CALIB_STATE_CALIBRATED_REFINED ||
                      m_calibStateOld == CALIB_STATE_CALIBRATED_REFINED )
            {
                distortionCoefficients = m_distortionCoefficientsRefined.clone();
            }
        }

        bool canUseRefining()
        {
            return ( m_calibState == CALIB_STATE_CALIBRATED_SIMPLE || m_calibStateOld == CALIB_STATE_CALIBRATED_SIMPLE ) ||
                   ( m_calibState == CALIB_STATE_CALIBRATED_REFINED || m_calibStateOld == CALIB_STATE_CALIBRATED_REFINED );
        }
    };

}