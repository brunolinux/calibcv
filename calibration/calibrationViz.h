
#pragma once

#include "calibrationCommon.h"

using namespace std;

#define PLOT_SPACING_TOP_PERCENT 0.2 

namespace calibration
{

    // Heatmap like visualizer
    class DistributionVisualizer
    {

        private :

        string m_windowBaseName;
        string m_windowHistogram;
        string m_windowHeatmap;
        string m_windowDistribution;
        string m_windowCalibrationFrame;
        string m_windowCalibrationErrors;
        // if uses the average of the pattern points as the value to put in the heatmap
        bool m_usesAverage;

        int m_xDiv;
        int m_yDiv;

        int m_xRes;
        int m_yRes;

        int m_fWidth;
        int m_fHeight;

        cv::Size m_patternSize;

        int m_currentCalibrationFrameIndx;
        vector< cv::Mat > m_calibrationImages;
        vector< float > m_calibrationPerViewErrors;
        vector< int > m_calibrationTypes;
        vector< float > m_angles;// to draw a histogram of the angles
        vector< vector< float > > m_heatmap;// heatmap of resolution xresdiv by yresdiv

        cv::Mat m_patternHeatmap;
        cv::Mat m_patternDistribution;
        cv::Mat m_patternHistogram;
        cv::Mat m_patternErrors;

        public :

        DistributionVisualizer( string windowBaseName, cv::Size patternSize,
                                int frameWidth, int frameHeight,
                                bool useAverage = false, 
                                int xDiv = DIST_VIS_X_DIV, 
                                int yDiv = DIST_VIS_Y_DIV )
        {
            m_windowBaseName = windowBaseName;
            m_usesAverage = useAverage;


            m_windowCalibrationFrame = m_windowHistogram = m_windowHeatmap = m_windowDistribution = m_windowBaseName;

            m_windowHistogram    += "_histogram";
            m_windowHeatmap      += "_heatmap";
            m_windowDistribution += "_distribution";
            m_windowCalibrationFrame += "_calibration_frames";
            m_windowCalibrationErrors += "_calibration_errors";

            cv::namedWindow( m_windowHistogram.c_str() );
            cv::namedWindow( m_windowHeatmap.c_str() );
            cv::namedWindow( m_windowDistribution.c_str() );
            cv::namedWindow( m_windowCalibrationFrame.c_str() );
            cv::namedWindow( m_windowCalibrationErrors.c_str() );

            cv::createTrackbar( "calibrationIndex", m_windowCalibrationFrame.c_str(),
                                &m_currentCalibrationFrameIndx,
                                1,
                                DistributionVisualizer::onSelectCalibrationImage, this );

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
            m_patternErrors         = cv::Mat::zeros( m_fHeight, m_fWidth, CV_8UC3 );

            m_angles = vector< float >( 360, 0.0f );

            m_currentCalibrationFrameIndx = -1;
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

        void addCalibratedBucket( const vector< cv::Mat >& frames4calibration, const vector< float >& perViewErrors, int calibrationType )
        {
            assert( frames4calibration.size() == perViewErrors.size() );

            for ( int q = 0; q < frames4calibration.size(); q++ )
            {
                m_calibrationImages.push_back( frames4calibration[q].clone() );
                m_calibrationPerViewErrors.push_back( perViewErrors[q] );
                m_calibrationTypes.push_back( calibrationType );
            }

            if ( m_currentCalibrationFrameIndx == -1 )
            {
                // Just at initialization
                m_currentCalibrationFrameIndx = m_calibrationImages.size() - 1;
            }

            cv::setTrackbarMax( "calibrationIndex", m_windowCalibrationFrame.c_str(), m_calibrationImages.size() - 1 );
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

        void _drawCalibrationFrame()
        {
            if ( m_calibrationImages.size() < 1 )
            {
                // no frames yet
                return;
            }

            if ( 0 <= m_currentCalibrationFrameIndx && 
                 m_currentCalibrationFrameIndx < m_calibrationImages.size() )
            {
                cv::imshow( m_windowCalibrationFrame.c_str(), m_calibrationImages[ m_currentCalibrationFrameIndx ] );
            }
        }

        void _drawCalibrationErrors()
        {
            if ( m_calibrationImages.size() < 1 )
            {
                return;
            }

            m_patternErrors.setTo( cv::Scalar( 0, 0, 0 ) );

            float _min = m_calibrationPerViewErrors[0], _max = m_calibrationPerViewErrors[0];
            for ( int q = 1; q < m_calibrationPerViewErrors.size(); q++ )
            {
                _min = min( m_calibrationPerViewErrors[q], _min );
                _max = max( m_calibrationPerViewErrors[q], _max );
            }

            float _scaleFactor = ( 1.0f - PLOT_SPACING_TOP_PERCENT );

            for ( int q = 0; q < m_calibrationPerViewErrors.size() - 1; q++ )
            {
                float _x = m_fWidth * ( (float)q ) / m_calibrationPerViewErrors.size();
                float _y = m_fHeight * ( 1.0f - _scaleFactor * ( m_calibrationPerViewErrors[q] / _max ) );

                float _xNext = m_fWidth * ( (float)( q + 1 ) ) / m_calibrationPerViewErrors.size();
                float _yNext = m_fHeight * ( 1.0f - _scaleFactor * ( m_calibrationPerViewErrors[q + 1] / _max ) );

                _x = min( max( _x, 0.0f ), ( float ) m_fWidth );
                _y = min( max( _y, 0.0f ), ( float ) m_fHeight );

                _xNext = min( max( _xNext, 0.0f ), ( float ) m_fWidth );
                _yNext = min( max( _yNext, 0.0f ), ( float ) m_fHeight );                

                cv::line( m_patternErrors, 
                          cv::Point2f( _x, _y ),
                          cv::Point2f( _xNext, _yNext ),
                          cv::Scalar( 255, 255, 0 ), 4 );
            }

            // draw the current error given by the current frameindx
            float _xCurrent = m_fWidth * ( (float)m_currentCalibrationFrameIndx ) / m_calibrationPerViewErrors.size();
            float _yCurrent = m_fHeight * ( 1.0f - _scaleFactor * ( m_calibrationPerViewErrors[m_currentCalibrationFrameIndx] / _max ) );

            cv::line( m_patternErrors,
                      cv::Point2f( _xCurrent, m_fHeight - 1 ),
                      cv::Point2f( _xCurrent, _yCurrent ),
                      cv::Scalar( 255, 255, 255 ), 4 );


            string _errText = "reprojection error: ";
            _errText += to_string( m_calibrationPerViewErrors[m_currentCalibrationFrameIndx] );
            cv::putText( m_patternErrors, _errText, 
                         cv::Point2f( 20, 20 ), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 255, 255, 0 ), 2 );

            string _calibType = "calibrationType: ";
            _calibType += ( m_calibrationTypes[m_currentCalibrationFrameIndx] == VIZ_CALIB_TYPE_SIMPLE ) ? "simple" : "refined";
            cv::putText( m_patternErrors, _calibType, 
                         cv::Point2f( 60, 20 ), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 255, 255, 0 ), 2 );

            cv::imshow( m_windowCalibrationErrors.c_str(), m_patternErrors );
        }

        void draw()
        {
            _drawHistogram();
            _drawHeatmap();
            _drawDistribution();
            _drawCalibrationFrame();
            _drawCalibrationErrors();
        }


        void saveToFile()
        {
            auto _strHist = m_windowHistogram;
            _strHist += ".jpg";

            auto _strHeatmap = m_windowHeatmap;
            _strHeatmap += ".jpg";

            auto _strDistribution = m_windowDistribution;
            _strDistribution += ".jpg";

            cv::imwrite( _strHist, m_patternHistogram );
            cv::imwrite( _strHeatmap, m_patternHeatmap );
            cv::imwrite( _strDistribution, m_patternDistribution );
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

        static void onSelectCalibrationImage( int v, void* pVisualizer )
        {
            DistributionVisualizer* _visualizer = ( DistributionVisualizer* ) pVisualizer;

            if ( _visualizer->m_calibrationImages.size() < 1 )
            {
                _visualizer->m_currentCalibrationFrameIndx = -1;
            }
            else
            {
                _visualizer->m_currentCalibrationFrameIndx = min( max( v, 0 ), (int)( _visualizer->m_calibrationImages.size() - 1 ) );
            }
                
        }

    };




}