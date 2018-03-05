
#pragma once

#include <sstream>
#include <cmath>
#include <ctime>

#include "calibrationCommon.h"
#include "calibrationPatternConcentricUtils.h"

#include "calibrationPatternChessboard.h"
#include "calibrationPatternCircleGridSymmetric.h"
#include "calibrationPatternCircleGridAsymmetric.h"
#include "calibrationPatternConcentric.h"

using namespace std;

#define TAG_FRAME_WIDTH "FrameWidth"
#define TAG_FRAME_HEIGHT "FrameHeight"
#define TAG_CALIBRATION_ERROR_RMS "CalibrationRMSerror"
#define TAG_CALIBRATION_ERROR_REPROJECTION "CalibrationReprojectionError"
#define TAG_CALIBRATION_ERROR_COLINEARITY_OLD "CalibrationColinearityErrorOld"
#define TAG_CALIBRATION_ERROR_COLINEARITY_NEW "CalibrationColinearityErrorNew"
#define TAG_CAMERA_MATRIX "CameraMatrix"
#define TAG_DISTORTION_COEFFICIENTS "DistortionCoefficients"

#define DIST_VIS_X_DIV 20
#define DIST_VIS_Y_DIV 20

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

        string m_windowBaseName;
        string m_windowHistogram;
        string m_windowHeatmap;
        string m_windowDistribution;
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

        cv::Mat m_patternHeatmap;
        cv::Mat m_patternDistribution;
        cv::Mat m_patternHistogram;

        public :

        DistributionVisualizer( string windowBaseName, cv::Size patternSize,
                                int frameWidth, int frameHeight,
                                bool useAverage = false, 
                                int xDiv = DIST_VIS_X_DIV, 
                                int yDiv = DIST_VIS_Y_DIV )
        {
            m_windowBaseName = windowBaseName;
            m_usesAverage = useAverage;


            m_windowHistogram = m_windowHeatmap = m_windowDistribution = m_windowBaseName;

            m_windowHistogram    += "_histogram";
            m_windowHeatmap      += "_heatmap";
            m_windowDistribution += "_distribution";

            cv::namedWindow( m_windowHistogram.c_str() );
            cv::namedWindow( m_windowHeatmap.c_str() );
            cv::namedWindow( m_windowDistribution.c_str() );

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

            m_patternHeatmap        = cv::Mat::zeros( m_fHeight, m_fWidth, CV_8UC3 );
            m_patternDistribution   = cv::Mat::zeros( m_fHeight, m_fWidth, CV_8UC3 );
            m_patternHistogram      = cv::Mat::zeros( m_fHeight, m_fWidth, CV_8UC3 );

            m_angles = vector< float >( 360, 0.0f );
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

                int _xBin = min( max( round( _xAvg / m_xRes ), 0.0f ), m_xDiv - 1.0f );
                int _yBin = min( max( round( _yAvg / m_yRes ), 0.0f ), m_yDiv - 1.0f );

                m_heatmap[ _xBin ][ _yBin ] += 1.0f;
            }
            else
            {
                for ( int q = 0; q < patternPoints.size(); q++ )
                {
                    int _xBin = min( max( round( patternPoints[q].x / m_xRes ), 0.0f ), m_xDiv - 1.0f );
                    int _yBin = min( max( round( patternPoints[q].y / m_yRes ), 0.0f ), m_yDiv - 1.0f );

                    m_heatmap[ _xBin ][ _yBin ] += 1.0f;
                }
            }

            // process angles
            cv::Point2f _p0 = patternPoints[ 0 ];
            cv::Point2f _p1 = patternPoints[ m_patternSize.width - 1 ];
            float _angle = ( atan2( _p1.y - _p0.y, _p1.x - _p0.x ) / 3.14159265 + 1 ) * 180.0;
            int _angBin = min( max( round( _angle ), 0.0f ), 360.0f );
            m_angles[_angBin] += 1.0f;

            // process into distribution

            for ( int q = 0; q < patternPoints.size(); q++ )
            {
                cv::circle( m_patternDistribution, patternPoints[q], 2, cv::Scalar( 0, 0, 255 ), 1 );
            }
        }

        void _drawHistogram()
        {
            m_patternHistogram.setTo( cv::Scalar( 0, 0, 0 ) );

            float _min = m_angles[0], _max = m_angles[0];
            for ( int q = 1; q < m_angles.size(); q++ )
            {
                _min = min( m_angles[q], _min );
                _max = max( m_angles[q], _max );
            }

            for ( int q = 0; q < m_angles.size() - 1; q++ )
            {
                float _x = m_fWidth * ( (float)q ) / m_angles.size();
                float _y = m_fHeight * ( 1.0f - ( m_angles[q] / _max ) );

                float _xNext = m_fWidth * ( (float)( q + 1 ) ) / m_angles.size();
                float _yNext = m_fHeight * ( 1.0f - ( m_angles[q + 1] / _max ) );

                _x = min( max( _x, 0.0f ), ( float ) m_fWidth );
                _y = min( max( _y, 0.0f ), ( float ) m_fHeight );

                _xNext = min( max( _xNext, 0.0f ), ( float ) m_fWidth );
                _yNext = min( max( _yNext, 0.0f ), ( float ) m_fHeight );                

                cv::line( m_patternHistogram, 
                          cv::Point2f( _x, _y ),
                          cv::Point2f( _xNext, _yNext ),
                          cv::Scalar( 255, 0, 0 ), 4 );
            }

            cv::imshow( m_windowHistogram.c_str(), m_patternHistogram );
        }

        void _drawHeatmap()
        {
            m_patternHeatmap.setTo( cv::Scalar( 0, 0, 0 ) );

            float _max = -100000.0f;
            for ( int x = 0; x < m_xDiv; x++ )
            {
                for ( int y = 0; y < m_yDiv; y++ )
                {
                    _max = max( _max, m_heatmap[ x ][ y ] );
                }
            }

            for ( int x = 0; x < m_xDiv; x++ )
            {
                for ( int y = 0; y < m_yDiv; y++ )
                {
                    int _intensity = 255.0f * m_heatmap[x][y] / _max;
                    _intensity = min( max( _intensity, 0 ), 255 );

                    cv::Scalar _color( _intensity, _intensity, _intensity );

                    float _x = x * m_xRes;
                    float _y = y * m_yRes;

                    float _xn = ( x + 1 ) * m_xRes;
                    float _yn = ( y + 1 ) * m_yRes;

                    _x = min( max( _x, 0.0f ), ( float ) m_fWidth );
                    _y = min( max( _y, 0.0f ), ( float ) m_fHeight );

                    _xn = min( max( _xn, 0.0f ), ( float ) m_fWidth );
                    _yn = min( max( _yn, 0.0f ), ( float ) m_fHeight );                

                    cv::rectangle( m_patternHeatmap,
                                   cv::Point2f( _x, _y ), cv::Point2f( _xn, _yn ),
                                   _color, CV_FILLED );
                }
            }

            cv::imshow( m_windowHeatmap.c_str(), m_patternHeatmap );

        }

        void _drawDistribution()
        {
            cv::imshow( m_windowDistribution.c_str(), m_patternDistribution );
        }

        void draw()
        {
            _drawHistogram();
            _drawHeatmap();
            _drawDistribution();
        }

        float getMeanAngle()
        {
            float _sum = 0.0f;
            for ( int q = 0; q < m_angles.size(); q++ )
            {
                _sum += m_angles[q];
            }

            return 180.0f * _sum / ( m_angles.size() * 3.1415926 );
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
        float m_calibrationNewColinearityError;
        float m_calibrationOldColinearityError;

        DistributionVisualizer* m_visualizer;

        public :

        Calibrator( const cv::Size& frameSize, const PatternInfo& pInfo )
        {
            m_isCalibrated = false;

            m_frameSize = frameSize;

            m_patternInfo = pInfo;

            m_visualizer = new DistributionVisualizer( "CalibrationViz",
                                                       m_patternInfo.cb_size,
                                                       m_frameSize.width, m_frameSize.height );
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

            m_visualizer->processCalibrationBucket( corners2D );
        }

        void update()
        {
            m_visualizer->draw();
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

            computeColinearityErrors( m_pointsInWorld, 
                                      m_pointsInImage, 
                                      m_calibrationRotMatrices, 
                                      m_calibrationTranMatrices, 
                                      m_calibrationOldColinearityError,
                                      m_calibrationNewColinearityError );

            std::cout << "********************************************" << std::endl;
            std::cout << "camera matrix: " << std::endl;
            std::cout << m_cameraMatrix << std::endl;
            std::cout << "distortion coefficients: " << std::endl;
            std::cout << m_distortionCoefficients << std::endl;
            std::cout << "calibration rms error: " << std::endl;
            std::cout << m_calibrationRMSerror << std::endl;
            std::cout << "calibration reprojection error: " << std::endl;
            std::cout << m_calibrationReprojectionError << std::endl;
            std::cout << "calibration colinearity error old: " << std::endl;
            std::cout << m_calibrationOldColinearityError << std::endl;
            std::cout << "calibration colinearity error new: " << std::endl;
            std::cout << m_calibrationNewColinearityError << std::endl;
            std::cout << "mean pattern angle" << std::endl;
            std::cout << m_visualizer->getMeanAngle() << std::endl;
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
            _fs << TAG_CALIBRATION_ERROR_COLINEARITY_OLD << m_calibrationOldColinearityError;
            _fs << TAG_CALIBRATION_ERROR_COLINEARITY_NEW << m_calibrationNewColinearityError;
            
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

        void computeColinearityErrors( const vector< vector< cv::Point3f > >& worldPoints,
                                       const vector< vector< cv::Point2f > >& imagePoints,
                                       const vector< cv::Mat >& rvecs, const vector< cv::Mat >& tvecs,
                                       float& oldColinearity, float& newColinearity )
        {
            if ( !m_isCalibrated )
            {
                std::cout << "camera not calibrated yet" << std::endl;
            }
            
            vector< cv::Point2f > _reprojectedPoints;
            size_t _totalPoints = 0;
            
            float _totalErrOld = 0;
            float _totalErrNew = 0;
            float _err = 0;        

            for( size_t i = 0; i < worldPoints.size(); ++i )
            {
                cv::projectPoints( worldPoints[i], rvecs[i], tvecs[i], m_cameraMatrix, m_distortionCoefficients, _reprojectedPoints );
                
                _err = utils::checkEnd2EndColinearity( _reprojectedPoints );
                _totalErrNew           += _err * _err;
                _err = utils::checkEnd2EndColinearity( imagePoints[i] );
                _totalErrOld           += _err * _err;

                size_t _n           = worldPoints[i].size();
                _totalPoints        += _n;
            }

            oldColinearity = std::sqrt( _totalErrOld / _totalPoints );
            newColinearity = std::sqrt( _totalErrNew / _totalPoints );
        }

        bool isCalibrated() { return m_isCalibrated; }

        int getCalibrationSize() { return m_calibrationImages.size(); }

        float getCalibrationRMSerror() { return m_calibrationRMSerror; }

        float getCalibrationColinearityError() { return m_calibrationNewColinearityError; }
    };


}