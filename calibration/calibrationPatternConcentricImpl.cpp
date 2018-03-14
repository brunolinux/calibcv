
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

        int _w = pInfo.cb_size.width;
        int _h = pInfo.cb_size.height;

        string _cornerText = "p";
        cv::putText( image, _cornerText + std::to_string( 0 ),
                     iCorners[ 0 ], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 255, 255, 0 ), 2 );
        cv::putText( image, _cornerText + std::to_string( 1 ),
                     iCorners[ _w - 1 ], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 255, 255, 0 ), 2 );
        cv::putText( image, _cornerText + std::to_string( 2 ),
                     iCorners[ _w * _h - 1 ], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 255, 255, 0 ), 2 );
        cv::putText( image, _cornerText + std::to_string( 3 ),
                     iCorners[ _w * ( _h - 1 ) ], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 255, 255, 0 ), 2 );
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

    bool findConcentricGrid( const cv::Mat& image, const cv::Size pSize,
                             const vector< cv::Point2f >& roi,
                             vector< cv::Point2f >& iCorners,
                             const bool& isCalibrated,
                             const cv::Mat& cameraMatrix,
                             const cv::Mat& distortionCoefficients )
    {
        detection::Detector* _detector = detection::Detector::create( pSize );
        _detector->m_isCalibrated = isCalibrated;
        if( isCalibrated )
        {
            _detector->setUndistortionParams( cameraMatrix, distortionCoefficients );
        }

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

            m_stageFrameResults = vector< cv::Mat >( 5 );

            cv::SimpleBlobDetector::Params _detectorCreationParams;
            _detectorCreationParams.filterByArea = true;
            _detectorCreationParams.minArea = 100;
            _detectorCreationParams.maxArea = 1550;
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
            m_pipelinePanel->showPreprocessing( m_stageFrameResults[ STAGE_KEYPOINTS_TRANSFORMATION ] );

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

                // Make sure the orientation is the base one ( nor p0 or p1 are bottom )
                if ( q == 1 || q == 2 )
                {
                    continue;
                }

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
            if( m_isCalibrated )
            {
                cv::Mat undistorted;
                cv::undistort( input, undistorted, m_cameraMatrix, m_distortionCoefficients );
                //cv::remap( input, undistorted, m_transformationMap1, m_transformationMap2, cv::INTER_LINEAR );
                m_undistortedPoints.clear();
                vector< cv::Point2f > tPoints;
                cv::Point2f undistortedP;
                for( auto& trackPoint : m_trackingPoints )
                {
                    /*undistortedP.x = m_transformationMap1.at<float>( trackPoint.pos );
                    undistortedP.y = m_transformationMap2.at<float>( trackPoint.pos );
                    m_undistortedPoints.push_back(undistortedP);*/
                    tPoints.push_back( trackPoint.pos );
                }
                cv::undistortPoints(tPoints, m_undistortedPoints, m_cameraMatrix, m_distortionCoefficients, cv::noArray(), m_cameraMatrix);
                _pipeline2( undistorted, m_stageFrameResults[ STAGE_KEYPOINTS_TRANSFORMATION ] );
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

        void Detector::_pipeline2( const cv::Mat& input, cv::Mat& fronto_parallel )
        {

            cv::Point2f topLeft = 2 * m_undistortedPoints[0] -
                                  m_undistortedPoints[ m_size.width + 1 ];

            cv::Point2f topRight = 2 * m_undistortedPoints[ m_size.width - 1 ] -
                                   m_undistortedPoints[ m_size.width * 2 - 2 ];

            cv::Point2f botLeft = 2 * m_undistortedPoints[ m_size.width * ( m_size.height - 1 ) ] -
                                  m_undistortedPoints[ m_size.width * ( m_size.height - 2 ) + 1 ];

            cv::Point2f botRight = 2 * m_undistortedPoints[ m_size.width * m_size.height - 1 ] -
                                   m_undistortedPoints[ m_size.width * ( m_size.height - 1 )  - 2 ];

            /*cv::Point2f topLeft = 2 * m_trackingPoints[0].pos -
                                  m_trackingPoints[ m_size.width + 1 ].pos;

            cv::Point2f topRight = 2 * m_trackingPoints[ m_size.width - 1 ].pos -
                                   m_trackingPoints[ m_size.width * 2 - 2 ].pos;

            cv::Point2f botLeft = 2 * m_trackingPoints[ m_size.width * ( m_size.height - 1 ) ].pos -
                                  m_trackingPoints[ m_size.width * ( m_size.height - 2 ) + 1 ].pos;

            cv::Point2f botRight = 2 * m_trackingPoints[ m_size.width * m_size.height - 1 ].pos -
                                   m_trackingPoints[ m_size.width * ( m_size.height - 1 )  - 2 ].pos;*/

            vector<cv::Point2f> src = { topLeft, topRight, botLeft, botRight };
            vector<cv::Point2f> dst = { cv::Point2f(0, 0), cv::Point2f(400, 0), cv::Point2f(0, 320), cv::Point2f(400, 320) };
            cv::Mat perspectiveTransform = cv::getPerspectiveTransform(src, dst);
            //cv::Mat perspectiveTransform = cv::findHomography( src, dst, cv::RANSAC);

            cv::warpPerspective( input, fronto_parallel, perspectiveTransform, cv::Size( 400, 320 ) );

            cv::Mat invPerspectiveTransform = perspectiveTransform.inv();
            invPerspectiveTransform.convertTo(invPerspectiveTransform, CV_32FC2);

            cv::Mat newMask, newEdges, newFeatures;
            vector< cv::Point2f > candidatePoints;
            vector< cv::Point2f > distortedPoints;

            _runMaskGenerator( fronto_parallel,
                               newMask );
            // edge detection step
            _runEdgesGeneratorCanny( newMask,
                                     newEdges );
            // features extractor step
            _runFeaturesExtractor( newEdges,
                                   candidatePoints );

            for ( cv::Point2f& point : candidatePoints )
            {
                cv::circle( fronto_parallel, point, 1, cv::Scalar( 255, 255, 0 ), 2 );
            }


            if ( !perspectiveTransform.empty() )
            {
                m_perspectivePoints.clear();
                //m_perspectivePoints.reserve( candidatePoints.size() );
                for ( cv::Point2f& point : candidatePoints )
                {
                    cv::Mat2f dstPoint;
                    cv::perspectiveTransform( cv::Mat2f(point), dstPoint, perspectiveTransform.inv() );
                    m_perspectivePoints.push_back( cv::Point2f( dstPoint(0) ) );
                }
            }

            distortedPoints = distortPoints( m_perspectivePoints );
            //m_stageFrameResults[ STAGE_FEATURES_EXTRACTION ] = input.clone();

            for ( auto& point : distortedPoints )
            {
                cv::circle( m_stageFrameResults[ STAGE_FEATURES_EXTRACTION ], point, 1, cv::Scalar( 255, 255, 0 ), 2 );
            }
            // tracking step
            /*_runTracking( m_stageFrameResults[ STAGE_FEATURES_EXTRACTION ],
                          m_stageFrameResults[ STAGE_KEYPOINTS_TRACKING ] );*/
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

        void Detector::_runEdgesGeneratorCanny( const cv::Mat& input, cv::Mat& output )
        {
            cv::blur( input, output, cv::Size( 3, 3 ) );

            cv::Canny( output, output,
                       50,
                       150,
                       3 );

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

        void Detector::_runFeaturesExtractor( const cv::Mat& input, vector< cv::Point2f >& candidatePoints )
        {
            vector< vector< cv::Point > > _contours;
            vector< cv::Vec4i > _hierarchy;
            vector< cv::RotatedRect > _ellipsesBB;

            cv::findContours( input, _contours, _hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE );

            for ( auto _contour : _contours )
            {
                if ( _contour.size() > 5 )
                {
                    _ellipsesBB.push_back( cv::fitEllipse( cv::Mat( _contour ) ) );
                }
            }

            vector< cv::RotatedRect > _filteredEllipses;

            for ( int q = 0; q < _ellipsesBB.size(); q++ )
            {
                float _a = _ellipsesBB[q].size.width;
                float _b = _ellipsesBB[q].size.height;
                float _size = sqrt( _a * _a + _b * _b );
                float _ratio = _a / _b;

                if ( ( ELLIPSE_MIN_SIZE < _size && _size < ELLIPSE_MAX_SIZE ) &&
                     ( ELLIPSE_MIN_RATIO < _ratio && _ratio < ELLIPSE_MAX_RATIO ) )
                {
                    _filteredEllipses.push_back( _ellipsesBB[q] );
                }

            }

            candidatePoints.clear();
            for ( cv::RotatedRect _rect : _filteredEllipses )
            {
                candidatePoints.push_back( _rect.center );
            }
        }

        void Detector::_runTracking( const cv::Mat& input, cv::Mat& output )
        {
            if ( m_mode == MODE_TRACKING )
            {
                vector< bool > _assigned( m_candidatePoints.size(), false );
                vector< bool > _assignedPerspective( m_perspectivePoints.size(), false );

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

                    /*for ( int q = 0; q < m_perspectivePoints.size(); q++ )
                    {
                        cv::Point2f _candidate = m_perspectivePoints[q];

                        if ( !_assignedPerspective[q] && utils::dist( m_trackingPoints[t].pos, _candidate/* + m_cropOrigin*/ //) < 20 )
                        //{
                        //    m_trackingPoints[t].vel = _candidate/* + m_cropOrigin*/ - m_trackingPoints[t].pos;
                        //    m_trackingPoints[t].pos = m_trackingPoints[t].pos + _candidate / 2 /* + m_cropOrigin*/;
                        //    m_trackingPoints[t].found = true;
                        //    _assignedPerspective[q] = true;
                        //    break;
                        //}
                    //}
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

        vector< cv::Point2f > Detector::distortPoints( vector< cv::Point2f >& undistortedPoints )
        {
            double fx = m_cameraMatrix.at<double>(0);
            double fy = m_cameraMatrix.at<double>(4);
            double cx = m_cameraMatrix.at<double>(2);
            double cy = m_cameraMatrix.at<double>(5);

            double k1 = m_distortionCoefficients.at<double>(0);
            double k2 = m_distortionCoefficients.at<double>(1);
            double p1 = m_distortionCoefficients.at<double>(2);
            double p2 = m_distortionCoefficients.at<double>(3);
            double k3 = m_distortionCoefficients.at<double>(4);

            vector< cv::Point2f > distortedPoints;

            for( cv::Point2f& point : undistortedPoints )
            {
                double x = ( point.x - cx ) / fx;
                double y = ( point.y - cy ) / fy;

                double r2 = x*x + y*y;

                double xDistort = x * ( 1 + k1 * r2 + k2 * r2 * r2  + k3 * r2 * r2 * r2 );
                double yDistort = y * ( 1 + k1 * r2 + k2 * r2 * r2  + k3 * r2 * r2 * r2 );

                xDistort += ( 2 * p1 * x * y + p2 * ( r2 + 2 * x * x ) );
                yDistort += ( p1 * ( r2 + 2 * y * y ) + 2 * p2 * x * y );

                xDistort = xDistort * fx + cx;
                yDistort = yDistort * fy + cy;

                distortedPoints.push_back( cv::Point2f( (float)xDistort, (float)yDistort ) );
            }

            return distortedPoints;
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
