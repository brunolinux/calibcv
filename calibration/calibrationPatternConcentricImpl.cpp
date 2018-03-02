
#include "calibrationPatternConcentricImpl.h"
#include "calibrationPatternConcentricUtils.h"

using namespace std;

namespace calibration { namespace concentric {

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

            cv::line( image, iCorners[q], iCorners[q + 1], _color, 2 );
        }
    }

    bool findConcentricGrid( const cv::Mat& image, const cv::Size pSize, 
                             const vector< cv::Point2f >& roi,
                             vector< cv::Point2f >& iCorners )
    {
        detection::Detector* _detector = detection::Detector::create( pSize );

        bool _found = _detector->run( image, roi );
        if ( _found )
        {
            _detector->getDetectedPoints( iCorners );
        }

        return _found;
    }

    namespace detection
    {

        Detector* Detector::INSTANCE = NULL;

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

            m_pipelinePanel = calibcv::SPatternDetectorPanel::create();
        }

        Detector::~Detector()
        {
            m_pipelinePanel = NULL;
            calibcv::SPatternDetectorPanel::release();
        }

        Detector* Detector::create( const cv::Size& size )
        {
            if ( Detector::INSTANCE == NULL )
            {
                std::cout << "created new detector instance" << std::endl;
                Detector::INSTANCE = new Detector( size );
            }
                
            return Detector::INSTANCE;
        }

        void Detector::release()
        {
            if ( Detector::INSTANCE != NULL )
            {
                delete Detector::INSTANCE;
                Detector::INSTANCE = NULL;
            }
        }

        void Detector::setPatternSize( const cv::Size& size )
        {
            m_numPoints      = size.width * size.height;
            m_size           = size;
            m_trackingPoints = vector< TrackingPoint >( m_numPoints );
        }

        bool Detector::run( const cv::Mat& input, const vector< cv::Point2f >& roi )
        {
            setInitialROI( roi );

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

            //m_pipelinePanel->showBase( input );
            m_pipelinePanel->showMask( m_stageFrameResults[ STAGE_THRESHOLDING ] );
            m_pipelinePanel->showEdges( m_stageFrameResults[ STAGE_EDGE_DETECTION ] );
            m_pipelinePanel->showBlobs( m_stageFrameResults[ STAGE_FEATURES_EXTRACTION ] );
            m_pipelinePanel->showTracking( m_stageFrameResults[ STAGE_KEYPOINTS_TRACKING ] );

            m_pipelinePanel->cleanInfo();

            // string _logString = "";
            // _logString += "log ******************\n\r";

            // _logString += "numPoints: "; _logString += std::to_string( m_candidatePoints.size() ); _logString += "\n\r";
            // _logString += "state: "; _logString += getCurrentDetectionMode(); _logString += "\n\r";

            // _logString += "**********************\n\r";

            // cout << "numCandidatePoints: " << m_candidatePoints.size() << endl;

            m_pipelinePanel->setLogInfo( getCurrentDetectionMode() );
            

            return _ret;
        }

        bool Detector::runInitialDetectionMode( const cv::Mat& input )
        {
            _pipeline( input );

            if ( m_initialROI.size() != 4 )
            {
                // cout << "Must initialize with a given region of interest" << endl;
                return false;
            }

            if ( m_candidatePoints.size() >= m_numPoints )
            {
                int _countInROI = 0;
                bool _foundEnoughInRegion = false;

                for ( cv::Point2f& _candidatePoint : m_candidatePoints )
                {
                    if ( cv::pointPolygonTest( m_initialROI, _candidatePoint, false ) > 0 )
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

                            m_trackingPoints[q] = _tpt;
                        }

                        m_mode = MODE_TRACKING;
                        return true;
                    }
                }
            }

            return false;
        }

        bool Detector::_computeInitialPattern( const vector< cv::Point2f >& candidatePatternPoints,
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
                    float _xmin = candidatePatternPoints[ _leftIndx ].x;
                    float _xmax = candidatePatternPoints[ _rightIndx ].x;
                    float _ymin = candidatePatternPoints[ _topIndx ].y;
                    float _ymax = candidatePatternPoints[ _bottomIndx ].y;

                    m_cropROI = cv::Rect2f( cv::Point2f( _xmin, _ymin ), cv::Point2f( _xmax, _ymax ) );
                    m_cropOrigin = cv::Point2f( _xmin, _ymin );

                    // cout << "_xmin = " << candidatePatternPoints[ _leftIndx ].x << endl;
                    // cout << "_xmax = " << candidatePatternPoints[ _rightIndx ].x << endl;
                    // cout << "_ymin = " << candidatePatternPoints[ _topIndx ].y << endl;
                    // cout << "_ymax = " << candidatePatternPoints[ _bottomIndx ].y << endl;

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

            return true;
        }

        bool Detector::runRecoveringMode( const cv::Mat& input )
        {
            _pipeline( input );

            if ( m_candidatePoints.size() >= m_numPoints )
            {
                if ( _computeInitialPattern( m_candidatePoints, m_matchedPoints ) )
                {
                    for ( int q = 0; q < m_matchedPoints.size(); q++ )
                    {
                        TrackingPoint _tpt;
                        _tpt.pos    = m_matchedPoints[q];
                        _tpt.vel    = cv::Point2f( 0.0f, 0.0f );
                        _tpt.found  = true;

                        m_trackingPoints[q] = _tpt;
                    }

                    m_mode = MODE_TRACKING;
                    return true;
                }
            }

            return false;
        }

        void Detector::_pipeline( const cv::Mat& input )
        {
            m_stageFrameResults.clear();
            m_frame = input;

            // if ( m_mode == MODE_TRACKING )
            // {
            //     m_workingInput = input( m_cropROI ).clone();
            // }
            // else
            // {
                m_workingInput = input;
            // }

            // thresholding step
            _runMaskGenerator( m_workingInput, 
                               m_stageFrameResults[ STAGE_THRESHOLDING ] );
            // edge detection step
            _runEdgesGenerator( m_stageFrameResults[ STAGE_THRESHOLDING ], 
                                m_stageFrameResults[ STAGE_EDGE_DETECTION ] );
            // features extractor step
            _runFeaturesExtractor( m_stageFrameResults[ STAGE_EDGE_DETECTION ],
                                   m_stageFrameResults[ STAGE_FEATURES_EXTRACTION ] );
            // tracking step
            _runTracking( m_stageFrameResults[ STAGE_FEATURES_EXTRACTION ],
                          m_stageFrameResults[ STAGE_KEYPOINTS_TRACKING ] );


        }

        void Detector::_runMaskGenerator( const cv::Mat& input, cv::Mat& output )
        {
            cv::Mat _grayScale;

            cv::cvtColor( input, _grayScale, CV_RGB2GRAY );
            cv::adaptiveThreshold( _grayScale, output, PIPELINE_MASKING_STAGE_MAX_VALUE,
                                   cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV,
                                   PIPELINE_MASKING_STAGE_BLOCKSIZE, PIPELINE_MASKING_STAGE_C );

            // m_stageFrameResults[ STAGE_THRESHOLDING ] = output;
        }

        void Detector::_runEdgesGenerator( const cv::Mat& input, cv::Mat& output )
        {
            cv::Mat _gradX, _gradY, _gradAbsX, _gradAbsY;

            cv::Scharr( input, _gradX, CV_16S, 1, 0,
                        PIPELINE_EDGES_STAGE_SCALE, PIPELINE_EDGES_STAGE_DELTA, cv::BORDER_DEFAULT );
            cv::convertScaleAbs( _gradX, _gradAbsX );

            cv::Scharr( input, _gradY, CV_16S, 0, 1,
                        PIPELINE_EDGES_STAGE_SCALE, PIPELINE_EDGES_STAGE_DELTA, cv::BORDER_DEFAULT );
            cv::convertScaleAbs( _gradY, _gradAbsY );

            cv::addWeighted( _gradAbsX, 0.5, _gradAbsY, 0.5, 0, output );

            // m_stageFrameResults[ STAGE_EDGE_DETECTION ] = output;
        }

        void Detector::_runFeaturesExtractor( const cv::Mat& input, cv::Mat& output )
        {
            vector< cv::KeyPoint > _keypoints;
            m_blobsDetector->detect( input, _keypoints );

            output = m_workingInput.clone();

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
            if ( m_mode == MODE_TRACKING )
            {
                vector< bool > _assigned( m_candidatePoints.size(), false );

                for ( int t = 0; t < m_trackingPoints.size(); t++ )
                {
                    m_trackingPoints[t].found = false;

                    for ( int q = 0; q < m_candidatePoints.size(); q++ )
                    {
                        cv::Point2f _candidate = m_candidatePoints[q];

                        if ( !_assigned[q] && utils::dist( m_trackingPoints[t].pos, _candidate/* + m_cropOrigin*/ ) < 20 )
                        {
                            m_trackingPoints[t].vel = _candidate/* + m_cropOrigin*/ - m_trackingPoints[t].pos;
                            m_trackingPoints[t].pos = _candidate/* + m_cropOrigin*/;
                            m_trackingPoints[t].found = true;
                            _assigned[q] = true;
                            break;
                        }
                    }
                }

                vector<cv::Point2f> corners = { m_trackingPoints[0].pos,
                                                m_trackingPoints[ m_size.width - 1 ].pos,
                                                m_trackingPoints[ m_size.width * ( m_size.height - 1 ) ].pos,
                                                m_trackingPoints[ m_size.width * m_size.height - 1 ].pos };
                float minX = 1000, maxX = -1000, minY = 1000, maxY = -1000;

                for( cv::Point2f& tPoint : corners )
                {
                    minX = min( tPoint.x, minX );
                    maxX = max( tPoint.x, maxX );
                    minY = min( tPoint.y, minY );
                    maxY = max( tPoint.y, maxY );
                }
                minX = max( minX - ROI_MARGIN, 0.0f );
                minY = max( minY - ROI_MARGIN, 0.0f );
                maxX = min( maxX + ROI_MARGIN, (float) m_frame.cols );
                maxY = min( maxY + ROI_MARGIN, (float) m_frame.rows );

                m_cropOrigin.x = minX;
                m_cropOrigin.y = minY;

                m_cropROI = cv::Rect2f( cv::Point2f( minX , minY ), cv::Point2f( maxX, maxY ) );
            }
            else
            {
                output = input.clone();
            }
        }


        void Detector::getDetectedPoints( vector< cv::Point2f >& iPoints )
        {
            for ( int q = 0; q < m_trackingPoints.size(); q++ )
            {
                iPoints.push_back( m_trackingPoints[q].pos );
            }
        }

        void Detector::getTimeCosts( vector< float >& timeCosts )
        {

        }

        void Detector::getStageFrameResults( vector< cv::Mat >& vStageResults )
        {
            for ( int q = 0; q < m_stageFrameResults.size(); q++ )
            {
                vStageResults.push_back( m_stageFrameResults[q] );
            }
        }

        string Detector::getCurrentDetectionMode()
        {
            string _modeStr = "None";

            switch ( m_mode )
            {
                case MODE_FINDING_PATTERN :

                    _modeStr = "Finding Pattern";

                break;

                case MODE_TRACKING :

                    _modeStr = "Tracking pattern";

                break;

                case MODE_RECOVERING :

                    _modeStr = "Recovering pattern";

                break;
            }

            return _modeStr;
        }

    }

}}