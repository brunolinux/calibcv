
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
            int _offset = q == ( pInfo.size.width - 1 ) ? 0 : 1;
            _currentColorIndx = ( ( q + _offset ) / pInfo.size.width ) % _colors.size();

            cv::Scalar _color = _colors[ _currentColorIndx ];

            cv::line( image, iCorners[q], iCorners[q + 1], _color, 2 );
        }

        int _w = pInfo.size.width;
        int _h = pInfo.size.height;

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
                             const DetectionInfo& detInfo,
                             vector< cv::Point2f >& iCorners )
    {
        detection::Detector* _detector = detection::Detector::create( pSize );

        bool _found = _detector->run( image, detInfo );
        if ( _found )
        {
            // if ( detInfo.useRefining )
            // {
            //     if ( _detector->hasRefinedPoints() )
            //     {
            //         _detector->getRefinedPoints( iCorners );
            //     }
            //     else
            //     {
            //         _found = false;
            //     }
            // }
            // else
            // {
            //     _detector->getDetectedPoints( iCorners );
            // }

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

            m_stageFrameResults = vector< cv::Mat >( PIPELINE_MAX_STAGES );

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
            m_hasRefinedPoints = false;

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

        bool Detector::run( const cv::Mat& input, const DetectionInfo& detInfo )
        {
            setInitialROI( detInfo.roi );

            m_frameSize = cv::Size( input.cols, input.rows );

            bool _ret = false;
            m_hasRefinedPoints = false;

            switch ( m_mode )
            {
                case MODE_FINDING_PATTERN :

                _ret = runInitialDetectionMode( input );

                break;

                case MODE_TRACKING :

                _ret = runTrackingMode( input, detInfo );

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
                                               vector< cv::Point2f >& matchedPoints, bool isFronto )
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

        bool Detector::runTrackingMode( const cv::Mat& input, const DetectionInfo& detInfo )
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

            // If found the points, refine them if requested
            bool _useRefining = detInfo.useRefining;
            m_refinedPoints.clear();

            if ( _useRefining )
            {
                bool _success = _refining( input, 
                                           detInfo.cameraMatrix, detInfo.distortionCoefficients,
                                           m_trackingPoints, m_refinedPoints );

                if ( _success )
                {
                    // Update tracking points with refined points
                    m_hasRefinedPoints = true;
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

        bool Detector::_refining( const cv::Mat& input, 
                                  const cv::Mat& cameraMatrix, const cv::Mat& distortionCoefficients, 
                                  const vector< TrackingPoint >& trackingPatternPoints,
                                  vector< cv::Point2f >& refinedPoints )
        {
            bool _success = true;

            vector< cv::Point2f > _patternPoints;
            for ( int q = 0; q < trackingPatternPoints.size(); q++ )
            {
                _patternPoints.push_back( trackingPatternPoints[q].pos );
            }

            vector< cv::Point2f > _patternPointsUndistorted;
            vector< cv::Point2f > _patternPointsFronto;
            vector< cv::Point2f > _patternPointsProjected;
            cv::Mat _frontoTransform;

            // Undistort the image
            _refiningUndistortion( input,
                                   m_stageFrameResults[ STAGE_REFINING_UNDISTORTED ], 
                                   cameraMatrix, distortionCoefficients,
                                   _patternPoints, _patternPointsUndistorted );

            // Convert to fronto parallel
            _refiningFronto( m_stageFrameResults[ STAGE_REFINING_UNDISTORTED ],
                             m_stageFrameResults[ STAGE_REFINING_FRONTO ],
                             _patternPointsUndistorted,
                             _frontoTransform );

            m_pipelinePanel->showRefUndistorted( m_stageFrameResults[ STAGE_REFINING_UNDISTORTED ] );
            m_pipelinePanel->showRefFronto( m_stageFrameResults[ STAGE_REFINING_FRONTO ] );

            if ( _frontoTransform.empty() )
            {
                return false;
            }

            // Refine in this view

            // Adaptive thresholding
            _refiningMask( m_stageFrameResults[ STAGE_REFINING_FRONTO ],
                           m_stageFrameResults[ STAGE_REFINING_MASK ] );

            // Canny edges
            _refiningEdges( m_stageFrameResults[ STAGE_REFINING_MASK ],
                            m_stageFrameResults[ STAGE_REFINING_EDGES ] );

            // Ellipse finding
            bool _foundRefined = _refiningFeatures( m_stageFrameResults[ STAGE_REFINING_EDGES ],
                                                    m_stageFrameResults[ STAGE_REFINING_FRONTO ],
                                                    m_stageFrameResults[ STAGE_REFINING_FEATURES ],
                                                    _patternPointsFronto );

            m_pipelinePanel->showRefMask( m_stageFrameResults[ STAGE_REFINING_MASK ] );
            m_pipelinePanel->showRefEdges( m_stageFrameResults[ STAGE_REFINING_EDGES ] );
            m_pipelinePanel->showRefFeatures( m_stageFrameResults[ STAGE_REFINING_FEATURES ] );

            // Let all refined points pass for now, will discard them in normal view

            // if ( !_foundRefined )
            // {
            //     cout << "couldnt refine features" << endl;
            //     return false;
            // }

            // Project back points to original view
            _refiningProjected( m_stageFrameResults[ STAGE_REFINING_UNDISTORTED ],// work on copy of undistorted
                                m_stageFrameResults[ STAGE_REFINING_PROJECTED ],
                                _frontoTransform.inv(),
                                _patternPointsFronto, 
                                _patternPointsProjected, 
                                _patternPointsUndistorted );// last points just for comparison

            // Distort back to original type of view
            _refiningDistortion( input,
                                 m_stageFrameResults[ STAGE_REFINING_DISTORTED ],
                                 cameraMatrix, distortionCoefficients,
                                 _patternPointsProjected,
                                 refinedPoints,
                                 _patternPoints );// last points just for comparison

            m_pipelinePanel->showRefProjected( m_stageFrameResults[ STAGE_REFINING_PROJECTED ] );
            m_pipelinePanel->showRefDistorted( m_stageFrameResults[ STAGE_REFINING_DISTORTED ] );

            // Check if found enough refined points
            int _numMatched = 0;

            for ( int q = 0; q < _patternPoints.size(); q++ )
            {
                for ( int p = 0; p < refinedPoints.size(); p++ )
                {
                    float _dist = utils::dist( _patternPoints[q], refinedPoints[p] );

                    if ( _dist < 10 )
                    {
                        _patternPoints[q] = refinedPoints[p];
                        _numMatched++;
                        break;
                    }
                }
            }

            refinedPoints = _patternPoints;

            return _numMatched == 20;
        }

        void Detector::_refiningUndistortion( const cv::Mat& input, cv::Mat& output,
                                              const cv::Mat& cameraMatrix, const cv::Mat& distortionCoefficients,
                                              const vector< cv::Point2f >& patternPoints,
                                              vector< cv::Point2f >& undistortedPatternPoints )
        {
            // transform the iamge to undistorted view
            cv::undistort( input, output, cameraMatrix, distortionCoefficients );

            // transform also the points into undistorted view
            cv::undistortPoints( patternPoints, undistortedPatternPoints, 
                                 cameraMatrix, distortionCoefficients,
                                 cv::noArray(), cameraMatrix );
        }

        void Detector::_refiningFronto( const cv::Mat& input,// undistorted image
                                        cv::Mat& output,// result after transforming to fronto view
                                        const vector< cv::Point2f >& undistortedPatternPoints,
                                        cv::Mat& frontoTransform )
        {
            // Create the points for the perspective mapping
            // Borders points with radial padding
            cv::Point2f _topLeft     = 2 * undistortedPatternPoints[0] - 
                                       undistortedPatternPoints[ m_size.width + 1 ];

            cv::Point2f _topRight    = 2 * undistortedPatternPoints[ m_size.width - 1 ] - 
                                       undistortedPatternPoints[ 2 * m_size.width - 2 ];

            cv::Point2f _bottomRight = 2 * undistortedPatternPoints[ m_size.width * m_size.height - 1 ] - 
                                       undistortedPatternPoints[ m_size.width * ( m_size.height - 1 ) - 2 ];

            cv::Point2f _bottomLeft  = 2 * undistortedPatternPoints[ m_size.width * ( m_size.height - 1 ) ] - 
                                       undistortedPatternPoints[ m_size.width * ( m_size.height - 2 ) + 1 ];

            vector< cv::Point2f > _src = { _topLeft, _topRight, _bottomRight, _bottomLeft };

            vector< cv::Point2f > _dst = { cv::Point2f( 0, 0 ), 
                                           cv::Point2f( m_frameSize.width / 2, 0 ), 
                                           cv::Point2f( m_frameSize.width / 2, m_frameSize.height / 2 ), 
                                           cv::Point2f( 0, m_frameSize.height / 2 ) };

            // Generate transformation matrix
            frontoTransform = cv::getPerspectiveTransform( _src, _dst );

            // Transform into fronto parallel view
            cv::warpPerspective( input, output, frontoTransform, cv::Size( m_frameSize.width / 2, m_frameSize.height / 2 ) );
        }

        void Detector::_refiningMask( const cv::Mat& input,
                                      cv::Mat& output )
        {
            cv::Mat _grayScale;

            cv::cvtColor( input, _grayScale, CV_RGB2GRAY );
            cv::adaptiveThreshold( _grayScale, output, 
                                   PIPELINE_REFINING_MASKING_STAGE_MAX_VALUE,
                                   cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV,
                                   PIPELINE_REFINING_MASKING_STAGE_BLOCKSIZE, 
                                   PIPELINE_REFINING_MASKING_STAGE_C );
        }

        void Detector::_refiningEdges( const cv::Mat& input,
                                       cv::Mat& output )
        {
            cv::blur( input, output, cv::Size( 3, 3 ) );

            cv::Canny( output, output, 
                       PIPELINE_REFINING_CANNY_MIN, 
                       PIPELINE_REFINING_CANNY_MAX, 
                       PIPELINE_REFINING_EDGES_BLOCK_SIZE );
        }

        bool Detector::_refiningFeatures( const cv::Mat& input,
                                          const cv::Mat& frontoView, // to copy ( output ) from for comparison
                                          cv::Mat& output,
                                          vector< cv::Point2f >& patternPointsFronto )
        {
            output = frontoView.clone();

            vector< vector< cv::Point > > _contours;
            vector< cv::Vec4i > _hierarchy;
            vector< cv::RotatedRect > _ellipsesBB;

            cv::findContours( input, _contours, _hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE );

            for ( auto _contour : _contours )
            {
                if ( _contour.size() > 10 )
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

                if ( ( PIPELINE_REFINING_ELLIPSE_MIN_SIZE < _size && _size < PIPELINE_REFINING_ELLIPSE_MAX_SIZE ) &&
                     ( PIPELINE_REFINING_ELLIPSE_MIN_RATIO < _ratio && _ratio < PIPELINE_REFINING_ELLIPSE_MAX_RATIO ) )
                {
                    _filteredEllipses.push_back( _ellipsesBB[q] );
                }

            }

            vector< cv::Point2f > _candidatePoints;
            for ( cv::RotatedRect _rect : _filteredEllipses )
            {
                _candidatePoints.push_back( _rect.center );
                cv::ellipse( output, _rect, cv::Scalar( 255, 0, 0 ), 2 );
            }

            // cout << "size of _candidatePoints: " << _candidatePoints.size() << endl;

            // // Detect only the ones that have two almost concentric ellipses
            
            // while( true )
            // {
            //     bool _hasPaired = false;

            //     vector< bool > _paired( _candidatePoints.size(), false );
            //     vector< cv::Point2f > _candidatesFilteredPoints;

            //     for ( int q = 0; q < _candidatePoints.size(); q++ )
            //     {
            //         for ( int p = 0; p < _candidatePoints.size(); p++ )
            //         {
            //             if ( p == q )
            //             {
            //                 continue;
            //             }

            //             if ( _paired[p] || _paired[q] )
            //             {
            //                 continue;
            //             }

            //             float _dx = _candidatePoints[q].x - _candidatePoints[p].x;
            //             float _dy = _candidatePoints[q].y - _candidatePoints[p].y;
            //             float _dist = sqrt( _dx * _dx + _dy * _dy );

            //             if ( _dist < 10 )
            //             {
            //                 _paired[p] = true;
            //                 _paired[q] = true;

            //                 _hasPaired = true;

            //                 cv::Point2f _fpoint;
            //                 _fpoint.x = ( _candidatePoints[p].x + _candidatePoints[q].x ) / 2.0f;
            //                 _fpoint.y = ( _candidatePoints[p].y + _candidatePoints[q].y ) / 2.0f;

            //                 _candidatesFilteredPoints.push_back( _fpoint );
            //             }
            //         }
            //     }

            //     if ( !_hasPaired )
            //     {
            //         break;
            //     }

            //     _candidatePoints = _candidatesFilteredPoints;
            //     _candidatesFilteredPoints.clear();
            // }

            // cout << "size of _candidatesFilteredPoints: " << _candidatePoints.size() << endl;

            // for ( int q = 0; q < _candidatePoints.size(); q++ )
            // {
            //     cv::circle( output, _candidatePoints[q], 2, cv::Scalar( 255, 255, 0 ), -1 );
            // }

            // if ( _candidatePoints.size() < m_size.width * m_size.height )
            // {
            //     return false;
            // }

            patternPointsFronto = _candidatePoints;

            return true/*_computeInitialPattern( _candidatePoints, patternPointsFronto, true )*/;
        }

        void Detector::_refiningProjected( const cv::Mat& undistortedView,
                                           cv::Mat& output,
                                           const cv::Mat& inverseFrontoTransform,
                                           const vector< cv::Point2f >& patternPointsFronto,
                                           vector< cv::Point2f >& patternPointsProjected,
                                           const vector< cv::Point2f >& patternPointsUndistorted )
        {
            output = undistortedView.clone();

            // convert points in fronto view to projected undistorted view

            for ( cv::Point2f _frontoPoint : patternPointsFronto )
            {
                cv::Mat2f _dstPoint;
                cv::Mat2f _srcPoint( _frontoPoint );

                cv::perspectiveTransform( _srcPoint, _dstPoint, inverseFrontoTransform );
                patternPointsProjected.push_back( cv::Point2f( _dstPoint( 0 ) ) );
            }

            // draw results in this undistorted space for comparison : initial vs refined

            cout << "size patternPointsProjected: " << patternPointsProjected.size() << endl;
            for ( int q = 0; q < patternPointsUndistorted.size(); q++ )
            {
                cv::circle( output, patternPointsUndistorted[q], 2, cv::Scalar( 255, 0, 0 ), -1 );
            }
            for ( int q = 0; q < patternPointsProjected.size(); q++ )
            {
                cv::circle( output, patternPointsProjected[q], 2, cv::Scalar( 0, 255, 0 ), -1 );
            }
            
        }

        void Detector::_refiningDistortion( const cv::Mat& originalView,
                                            cv::Mat& output,
                                            const cv::Mat& cameraMatrix, const cv::Mat& distortionCoefficients,
                                            const vector< cv::Point2f >& refinedUndistortedProjectedPoints,
                                            vector< cv::Point2f >& refinedDistortedProjectedPoints,
                                            const vector< cv::Point2f >& originalPatternPoints )
        {
            output = originalView.clone();

            utils::distortPoints( refinedUndistortedProjectedPoints,
                                  refinedDistortedProjectedPoints,
                                  cameraMatrix, distortionCoefficients );

            for ( int q = 0; q < originalPatternPoints.size(); q++ )
            {
                cv::circle( output, originalPatternPoints[q], 2, cv::Scalar( 255, 0, 0 ), -1 );
            }

            for ( int q = 0; q < refinedDistortedProjectedPoints.size(); q++ )
            {
                cv::circle( output, refinedDistortedProjectedPoints[q], 2, cv::Scalar( 0, 255, 0 ), -1 );
            }
        }

        void Detector::_pipeline( const cv::Mat& input )
        {
            m_stageFrameResults.clear();
            m_frame = input;

            m_workingInput = input;

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

        void Detector::getRefinedPoints( vector< cv::Point2f >& iPoints )
        {
            for ( int q = 0; q < m_refinedPoints.size(); q++ )
            {
                iPoints.push_back( m_refinedPoints[q] );
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