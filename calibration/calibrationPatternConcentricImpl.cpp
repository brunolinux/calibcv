
#include "calibrationPatternConcentricImpl.h"

using namespace std;

namespace calibcv
{

    void drawConcentricPatternCorners( const vector< cv::Point2f >& iCorners, cv::Mat& image, const PatternInfo& pInfo )
    {
        vector< cv::Scalar > _colors = { cv::Scalar( 0, 0, 255 ),
                                         cv::Scalar( 0, 128, 255 ),
                                         cv::Scalar( 0, 200, 200 ),
                                         cv::Scalar( 0, 255, 0 ),
                                         cv::Scalar( 200, 200, 0 ),
                                         cv::Scalar( 255, 0, 0 ),
                                         cv::Scalar( 255, 0, 255 ) };

        int _currentColorIndx = 0;
        for ( int q = 0; q < iCorners.size() - 1; q++ )
        {
            int _offset = q == ( pInfo.cb_size.width - 1 ) ? 0 : 1;
            _currentColorIndx = ( ( q + _offset ) / pInfo.cb_size.width ) % _colors.size();

            cv::Scalar _color = _colors[ _currentColorIndx ];

            cv::line( iCorners[q], iCorners[q + 1], _color, 2 );
        }
    }

    bool findConcentricGrid( const cv::Mat& image, const cv::Size pSize, vector< cv::Point2f >& iCorners )
    {
        return false;
    }

    namespace detection
    {

        Detector::Detector( const cv::Size& size )
        {
            m_numPoints      = size.width * size.height;
            m_size           = size;
            m_trackingPoints = vector< TrackingPoint >( m_numPoints );

            m_stageFrameResults = vector< cv::Mat >( 4 );

            cv::SimpleBlobDetector::Params _detectorCreationParams;
            _detectorCreationParams.filterByArea = true;
            _detectorCreationParams.minArea = 100;
            _detectorCreationParams.maxArea = 1500;
            _detectorCreationParams.filterByColor = true;
            _detectorCreationParams.blobColor = 0;
            _detectorCreationParams.filterByConvexity = true;
            _detectorCreationParams.minConvexity = 0.9;
            _detectorCreationParams.maxConvexity = 1;

            m_blobsDetector = cv::SimpleBlobDetector::create( _detectorCreationParams );

            m_mode = MODE_FINDING_PATTERN;

            m_initialROI = cv::Rect2i( 0, 0, 1, 1 );
        }

        Detector::~Detector()
        {

        }

        bool Detector::run( const cv::Mat& input )
        {
            bool _ret = false;

            switch ( m_mode )
            {
                case MODE_FINDING_PATTERN :

                _ret = runInitialDetectionMode( input );

                break;

                case MODE_TRACKING :

                _ret = runTrackingMode( input );

                break;

                case MODE_RECOVERING :

                _ret = runRecoveringMode( input );

                break;
            }

            return _ret;
        }

        bool Detector::runInitialDetectionMode( const cv::Mat& input )
        {
            if ( m_initialROI.width == 1 ||
                 m_initialROI.height == 1 )
            {
                cout << "Must initialize with a given region of interest" << endl;
                return false;
            }

            _pipeline( input );

            if ( m_candidatePoints.size() >= m_numPoints )
            {
                int _countInROI = 0;
                bool _foundEnoughInRegion = false;

                for ( cv::Point2f& _candidatePoint : m_candidatePoints )
                {
                    if ( m_initialROI.contains( _candidatePoint ) )
                    {
                        _countInROI++;
                        if ( _countInROI == m_numPoints )
                        {
                            _foundEnoughInRegion = true;
                            break;
                        }
                    }
                }

                if ( _foundEnoughInRegion )
                {
                    if ( _computeInitialPattern( m_candidatePoints, m_matchedPoints ) )
                    {
                        for ( int q = 0; q < m_matchedPoints.size(); q++ )
                        {
                            TrackingPoint _tpt;
                            _tpt.pos    = m_matchedPoints[q];
                            _tpt.vel    = cv::Point2f( 0.0f, 0.0f );
                            _tpt.found  = true;
                        }

                        m_mode = MODE_TRACKING;
                        return true;
                    }
                }
            }

            return false;
        }

        bool Detector::_computeInitialPattern( const vector< cv::Point2f >& candidatePatternPoints
                                               vector< cv::Point2f >& matchedPoints )
        {
            vector< cv::Point2f > _orderedKeypoints( m_numPoints );

            // Get bounding region to get the corners

            int _leftIndx   = 0;
            int _rightIndx  = 0;
            int _topIndx    = 0;
            int _bottomIndx = 0;

            for ( int q = 1; q < candidatePatternPoints.size(); q++ )
            {
                if ( candidatePatternPoints[q].x < candidatePatternPoints[_leftIndx].x )
                {
                    _leftIndx = q;
                }
                if ( candidatePatternPoints[q].x > candidatePatternPoints[_rightIndx].x )
                {
                    _rightIndx = q;
                }
                if ( candidatePatternPoints[q].y < candidatePatternPoints[_topIndx].y )
                {
                    _topIndx = q;
                }
                if ( candidatePatternPoints[q].y > candidatePatternPoints[_bottomIndx].y )
                {
                    _bottomIndx = q;
                }
            }
            
            vector< int > _cornerOptions = { _topIndx, _rightIndx, _bottomIndx, _leftIndx };
            bool _isFit = false;

            for ( int q = 0; q < _cornerOptions.size(); q++ )
            {
                matchedPoints.clear();

                _isFit = utils::isGridPatternFit( candidatePatternPoints, matchedPoints, m_size, 
                                                  _cornerOptions[( 0 + q ) % 4], _cornerOptions[( 1 + q ) % 4],
                                                  _cornerOptions[( 2 + q ) % 4], _cornerOptions[( 3 + q ) % 4] );
                if ( _isFit )
                {
                    return true;
                }
            }

            return _isFit;
        }

        bool Detector::runTrackingMode( const cv::Mat& input )
        {
            _pipeline( input );

            for ( int q = 0; q < m_trackingPoints.size(); q++ )
            {
                if ( m_trackingPoints[q].found == false )
                {
                    m_mode = MODE_RECOVERING;

                    return false;
                }
            }
        }

        bool Detector::runRecoveringMode( const cv::Mat& input )
        {
            return false;
        }

        void Detector::_pipeline( const cv::Mat& input )
        {
            m_stageFrameResults.clear();

            // thresholding step
            _runMaskGenerator( input, 
                               m_stageFrameResults[ STAGE_THRESHOLDING ] );
            // edge detection step
            _runEdgesGenerator( m_stageFrameResults[ STAGE_THRESHOLDING ], 
                                m_stageFrameResults[ STAGE_EDGE_DETECTION ] );
            // features extractor step
            _runFeaturesExtractor( m_stageFrameResults[ STAGE_EDGE_DETECTION ],
                                   m_stageFrameResults[ STAGE_FEATURES_EXTRACTION ] );
            // tracking step
            if ( m_mode == MODE_TRACKING )
            {
                _runTracking( m_stageFrameResults[ STAGE_FEATURES_EXTRACTION ],
                              m_stageFrameResults[ STAGE_KEYPOINTS_TRACKING ] );
            }
        }

        void Detector::_runMaskGenerator( const cv::Mat& input, cv::Mat& output )
        {
            cv::Mat _grayScale;

            cv::cvtColor( input, _grayScale, CV_RGB2GRAY );
            cv::adaptiveThreshold( _grayScale, output, MASKING_STAGE_MAX_VALUE,
                                   cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV,
                                   MASKING_STAGE_BLOCKSIZE, MASKING_STAGE_C );

            // m_stageFrameResults[ STAGE_THRESHOLDING ] = output;
        }

        void Detector::_runEdgesGenerator( const cv::Mat& input, cv::Mat& output )
        {
            cv::Mat _gradX, _gradY, _gradAbsX, _gradAbsY;

            cv::Scharr( input, _gradX, CV_16S, 1, 0,
                        EDGES_STAGE_SCALE, EDGES_STAGE_DELTA, cv::BORDER_DEFAULT );
            cv::convertScaleAbs( _gradX, _gradAbsX );

            cv::Scharr( input, _gradY, CV_16S, 0, 1,
                        EDGES_STAGE_SCALE, EDGES_STAGE_DELTA, cv::BORDER_DEFAULT );
            cv::convertScaleAbs( _gradY, _gradAbsY );

            cv::addWeighted( _gradAbsX, 0.5, _gradAbsY, 0.5, 0, output );

            // m_stageFrameResults[ STAGE_EDGE_DETECTION ] = output;
        }

        void Detector::_runFeaturesExtractor( const cv::Mat& input, cv::Mat& output )
        {
            vector< cv::KeyPoint > _keypoints;
            m_blobsDetector->detect( input, _keypoints );

            output = input.clone();

            for ( cv::KeyPoint& _keypoint : _keypoints )
            {
                cv::circle( output, _keypoint.pt, 4, cv::Scalar( 255, 0, 255 ), 2 );
            }

            m_candidatePoints.clear();
            for ( cv::KeyPoint& _keypoint : _keypoints )
            {
                m_candidatePoints.push_back( _keypoint.pt );
            }

            // m_stageFrameResults[ STAGE_FEATURES_EXTRACTION ] = output;
        }

        void Detector::_runTracking( const cv::Mat& input, cv::Mat& output )
        {

        }


    }




}