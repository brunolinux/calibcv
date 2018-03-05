
#pragma once

#include <sstream>
#include <ctime>

#include "calibrationCommon.h"

#include "calibrationPatternChessboard.h"
#include "calibrationPatternCircleGridSymmetric.h"
#include "calibrationPatternCircleGridAsymmetric.h"
#include "calibrationPatternConcentric.h"

using namespace std;

#define TAG_FRAME_WIDTH "FrameWidth"
#define TAG_FRAME_HEIGHT "FrameHeight"
#define TAG_CALIBRATION_ERROR_RMS "CalibrationRMSerror"
#define TAG_CALIBRATION_ERROR_REPROJECTION "CalibrationReprojectionError"
#define TAG_CAMERA_MATRIX "CameraMatrix"
#define TAG_DISTORTION_COEFFICIENTS "DistortionCoefficients"

#define DIST_VIS_X_DIV 100
#define DIST_VIS_Y_DIV 100

namespace calibration
{

    
    bool getPatternKnownPlanePositions( vector< cv::Point3f >& corners, const PatternInfo& pInfo )
    {
        bool success = true;
        
        switch ( pInfo.type )
        {
            case PATTERN_TYPE_CHESSBOARD :
                
                chessboard::getKnownPlanePositions( corners, pInfo );
                
                break;
                
            case PATTERN_TYPE_SYMMETRIC_CIRCLES :

                circleGridSymmetric::getKnownPlanePositions( corners, pInfo );

                break;

            case PATTERN_TYPE_ASYMMETRIC_CIRCLES :

                circleGridAsymmetric::getKnownPlanePositions( corners, pInfo );

                break;

            case PATTERN_TYPE_CONCENTRIC_CIRCLES :

                concentric::getKnownPlanePositions( corners, pInfo );

                break;

            default :
                
                cout << "pattern type: " << pInfo.type << " not found" << endl;
                success = false;
                
                break;
        }
        
        return success;
    }
  
    bool getPatternCorners( vector< cv::Point2f >& iCorners, const cv::Mat& image, const PatternInfo& pInfo, const DetectionInfo& dInfo )
    {

        cv::Mat _imgGray;
        cv::cvtColor( image, _imgGray, CV_BGR2GRAY );
        
        bool success = true;
        
        switch ( pInfo.type )
        {
            case PATTERN_TYPE_CHESSBOARD :
                
                success = chessboard::getCorners( iCorners, _imgGray, pInfo, dInfo );

                // refine only if chessboard pattern
                if ( success )
                {
                    cv::cornerSubPix( _imgGray, iCorners, cv::Size( 11,11 ), cv::Size( -1,-1 ), 
                                      cv::TermCriteria( cv::TermCriteria::EPS + 
                                                        cv::TermCriteria::COUNT, 30, 0.1 ) );
                }

                break;
                
            case PATTERN_TYPE_SYMMETRIC_CIRCLES :

                success = circleGridSymmetric::getCorners( iCorners, _imgGray, pInfo, dInfo );

                break;

            case PATTERN_TYPE_ASYMMETRIC_CIRCLES :

                success = circleGridAsymmetric::getCorners( iCorners, _imgGray, pInfo, dInfo );

                break;

            case PATTERN_TYPE_CONCENTRIC_CIRCLES :

                success = concentric::getCorners( iCorners, image, pInfo, dInfo );

                break;

            default :
                
                cout << "pattern type: " << pInfo.type << " not found" << endl;
                success = false;
                
                break;
        }

        return success;
    }

    void drawPatternCorners( const vector< cv::Point2f >& iCorners, cv::Mat& image, const PatternInfo& pInfo )
    {
        switch ( pInfo.type )
        {
            case PATTERN_TYPE_CHESSBOARD :
            case PATTERN_TYPE_SYMMETRIC_CIRCLES :
            case PATTERN_TYPE_ASYMMETRIC_CIRCLES :

                chessboard::drawPatternCorners( iCorners, image, pInfo );

            case PATTERN_TYPE_CONCENTRIC_CIRCLES :

                concentric::drawPatternCorners( iCorners, image, pInfo );

            break;
        }
    }

    // Heatmap like visualizer
    class DistributionVisualizer
    {

        private :

        string m_windowName;
        // if uses the average of the pattern points as the value to put in the heatmap
        bool m_usesAverage;

        int m_xDiv;
        int m_yDiv;

        int m_xRes;
        int m_yRes;

        int m_fWidth;
        int m_fHeight;

        cv::Size m_patternSize;

        vector< float > m_angles;// to draw a histogram of the angles
        vector< vector< float > > m_heatmap;// heatmap of resolution xresdiv by yresdiv

        cv::Mat m_patternDistribution;

        public :

        DistributionVisualizer( string windowName, cv::Size patternSize,
                                int frameWidth, int frameHeight,
                                bool useAverage = false, 
                                int xDiv = DIST_VIS_X_DIV, 
                                int yDiv = DIST_VIS_Y_DIV )
        {
            m_windowName = windowName;
            m_usesAverage = useAverage;

            cv::namedWindow( m_windowName );

            m_patternSize = patternSize;
            m_xDiv = xDiv;
            m_yDiv = yDiv;

            m_fWidth  = frameWidth;
            m_fHeight = frameHeight;

            m_xRes = m_fWidth / m_xDiv;
            m_yRes = m_fHeight / m_yDiv;

            m_heatmap = vector< vector< float > >( 1 );
            m_heatmap[0] = vector< float >( m_yDiv, 0.0f );
            m_heatmap.resize( m_xDiv, m_heatmap[0] );

            m_patternDistribution = cv::Mat::zeros( m_fWidth, m_fHeight, CV_8UC3 );

            m_angles = vector< float >( 360 );
        }

        ~DistributionVisualizer()
        {
            m_angles.clear();
            m_heatmap.clear();
        }

        void processCalibrationBucket( const vector< cv::Point2f >& patternPoints )
        {

            // process heatmap
            if ( m_usesAverage )
            {
                float _xAvg = 0.0f;
                float _yAvg = 0.0f;

                for ( int q = 0; q < patternPoints.size(); q++ )
                {
                    _xAvg += patternPoints[q].x;
                    _yAvg += patternPoints[q].y;
                }

                _xAvg = _xAvg / patternPoints.size();
                _yAvg = _yAvg / patternPoints.size();

                int _xBin = min( max( round( _xAvg / m_xRes ), 0 ), m_xDiv - 1 );
                int _yBin = min( max( round( _yAvg / m_yRes ), 0 ), m_yDiv - 1 );

                m_heatmap[ _xBin ][ _yBin ] += 1.0f;
            }
            else
            {
                for ( int q = 0; q < patternPoints.size(); q++ )
                {
                    int _xBin = min( max( round( patternPoints[q].x / m_xRes ), 0 ), m_xDiv - 1 );
                    int _yBin = min( max( round( patternPoints[q].y / m_yRes ), 0 ), m_yDiv - 1 );

                    m_heatmap[ _xBin ][ _yBin ] += 1.0f;
                }
            }

            // process angles
            cv::Point2f _p0 = patternPoints[ 0 ];
            cv::Point2f _p1 = patternPoints[ m_patternSize.width - 1 ];
            float _angle = ( atan2( _p1.y - _p0.y, _p1.x - _p0.x ) / 3.14159265 + 1 ) * 180.0;
            int _angBin = min( max( round( _angle ), 0 ), 360 );

            // process into distribution


        }

        void _drawAngles()
        {

        }

        void _drawHeatmap()
        {

        }

        void draw()
        {

        }

    };

    class Calibrator
    {

        private :

        // Info from each calibration bucket
        vector< cv::Mat > m_calibrationImages;
        vector< cv::Mat > m_calibrationRotMatrices;
        vector< cv::Mat > m_calibrationTranMatrices;
        vector< vector< cv::Point2f > > m_pointsInImage;
        vector< vector< cv::Point3f > > m_pointsInWorld;

        cv::Mat m_cameraMatrix;
        cv::Mat m_distortionCoefficients;

        cv::Mat m_transformationMap1;
        cv::Mat m_transformationMap2;

        bool m_isCalibrated;

        cv::Size m_frameSize;

        PatternInfo m_patternInfo;
        float m_calibrationRMSerror;
        float m_calibrationReprojectionError;

        public :

        Calibrator( const cv::Size& frameSize, const PatternInfo& pInfo )
        {
            m_isCalibrated = false;

            m_frameSize = frameSize;

            m_patternInfo = pInfo;
        }

        ~Calibrator()
        {
            m_calibrationImages.clear();
            m_pointsInImage.clear();
            m_pointsInWorld.clear();
        }

        void setSize( const cv::Size& frameSize )
        {
            m_frameSize = frameSize;
        }

        void init()
        {
            m_calibrationImages.clear();
            m_pointsInImage.clear();
            m_pointsInWorld.clear();

            m_isCalibrated = false;
        }

        void addCalibrationBucket( const cv::Mat& image,
                                   const vector< cv::Point2f >& corners2D )
        {
            m_calibrationImages.push_back( image );
            m_pointsInImage.push_back( corners2D );
        }

        void calibrate()
        {
            m_pointsInWorld = vector< vector< cv::Point3f > >( 1 );
            getPatternKnownPlanePositions( m_pointsInWorld[0], m_patternInfo );
            m_pointsInWorld.resize( m_pointsInImage.size(), m_pointsInWorld[0] );

            m_calibrationRMSerror = cv::calibrateCamera( m_pointsInWorld,
                                                         m_pointsInImage,
                                                         m_frameSize,
                                                         m_cameraMatrix,
                                                         m_distortionCoefficients,
                                                         m_calibrationRotMatrices,
                                                         m_calibrationTranMatrices );

            cv::initUndistortRectifyMap( m_cameraMatrix, m_distortionCoefficients, 
                                         cv::Mat(), cv::Mat(), 
                                         m_frameSize,
                                         CV_32FC1, m_transformationMap1, m_transformationMap2 );

            m_isCalibrated = true;

            vector< float > _perViewErrors;

            m_calibrationReprojectionError = computeReprojectionErrors( m_pointsInWorld, 
                                                                        m_pointsInImage, 
                                                                        m_calibrationRotMatrices, 
                                                                        m_calibrationTranMatrices, 
                                                                        _perViewErrors );

            std::cout << "********************************************" << std::endl;
            std::cout << "camera matrix: " << std::endl;
            std::cout << m_cameraMatrix << std::endl;
            std::cout << "distortion coefficients: " << std::endl;
            std::cout << m_distortionCoefficients << std::endl;
            std::cout << "calibration rms error: " << std::endl;
            std::cout << m_calibrationRMSerror << std::endl;
            std::cout << "calibration reprojection error: " << std::endl;
            std::cout << m_calibrationReprojectionError << std::endl;
            std::cout << "********************************************" << std::endl;
        }

        void applyCalibrationCorrection( const cv::Mat& src,
                                         cv::Mat& dst )
        {
            if ( !m_isCalibrated )
            {
                std::cout << "Calibrator::applyCalibrationCorrection> not calibrated yet" << std::endl;
                dst = src.clone();
                return;
            }

            cv::remap( src, dst, m_transformationMap1, m_transformationMap2, cv::INTER_LINEAR );
            //cv::undistort( src, dst, m_cameraMatrix, m_distortionCoefficients );
        }

        void saveToFile( string filename )
        {
            if ( !m_isCalibrated )
            {
                std::cout << "not calibrated yet, can't save" << std::endl;
                return;
            }
            
            cv::FileStorage _fs( filename, cv::FileStorage::WRITE );
 
            _fs << TAG_FRAME_WIDTH << m_frameSize.width;
            _fs << TAG_FRAME_HEIGHT << m_frameSize.height;
            _fs << TAG_CAMERA_MATRIX << m_cameraMatrix;
            _fs << TAG_DISTORTION_COEFFICIENTS << m_distortionCoefficients;
            _fs << TAG_CALIBRATION_ERROR_RMS << m_calibrationRMSerror;
            _fs << TAG_CALIBRATION_ERROR_REPROJECTION << m_calibrationReprojectionError;
            
            _fs.release();
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
            
            _fs[ TAG_CAMERA_MATRIX ] >> m_cameraMatrix;
            _fs[ TAG_DISTORTION_COEFFICIENTS ] >> m_distortionCoefficients;
            
            int _fw, _fh;
            _fs[ TAG_FRAME_WIDTH ] >> _fw;
            _fs[ TAG_FRAME_HEIGHT ] >> _fh;
            
            _fs.release();
            
            if ( m_frameSize.width != _fw ||
                 m_frameSize.height != _fh )
            {
                std::cout << "WARNING: Size from previous calibration does not match" << std::endl;
            }
            
            cv::initUndistortRectifyMap( m_cameraMatrix, m_distortionCoefficients, 
                                         cv::Mat(), cv::Mat(), 
                                         m_frameSize,
                                         CV_32FC1, m_transformationMap1, m_transformationMap2 );
            
            m_isCalibrated = true;
            
            return true;
        }

        float computeReprojectionErrors( const vector< vector< cv::Point3f > >& worldPoints,
                                         const vector< vector< cv::Point2f > >& imagePoints,
                                         const vector< cv::Mat >& rvecs, const vector< cv::Mat >& tvecs,
                                         vector<float>& perViewErrors )
        {
            if ( !m_isCalibrated )
            {
                std::cout << "camera not calibrated yet" << std::endl;
                return -1;
            }
            
            vector< cv::Point2f > _reprojectedPoints;
            size_t _totalPoints = 0;
            
            float _totalErr = 0;
            float _err = 0;
            
            perViewErrors.resize( worldPoints.size() );

            for( size_t i = 0; i < worldPoints.size(); ++i )
            {
                cv::projectPoints( worldPoints[i], rvecs[i], tvecs[i], m_cameraMatrix, m_distortionCoefficients, _reprojectedPoints );
                _err = cv::norm( imagePoints[i], _reprojectedPoints, cv::NORM_L2 );

                size_t _n           = worldPoints[i].size();
                perViewErrors[i]    = ( float ) std::sqrt( _err * _err / _n );
                _totalErr           += _err * _err;
                _totalPoints        += _n;
            }

            return std::sqrt( _totalErr / _totalPoints );
        }
        
        bool isCalibrated() { return m_isCalibrated; }

        int getCalibrationSize() { return m_calibrationImages.size(); }

        float getCalibrationRMSerror() { return m_calibrationRMSerror; }

        float getCalibrationColinearityError() { return m_calibrationColinearityError; }
    };


}