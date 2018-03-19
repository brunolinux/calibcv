
#include "../calibrationPatternUtils.h"
#include "calibrationPatternConcentricImpl.h"

#include <queue>

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

    bool findConcentricGrid( const cv::Mat& image, const cv::Size& pSize, 
                             const DetectionInfo& detInfo,
                             vector< cv::Point2f >& iCorners )
    {
        detection::DetectorConcentric* _detector = detection::DetectorConcentric::create( pSize );

        bool _found = _detector->run( image, detInfo );
        if ( _found )
        {
            _detector->getDetectedPoints( iCorners );
        }

        return _found;
    }

    void refineBatchConcentric( const cv::Size& pSize,
                                const vector< cv::Mat >& batchImagesToRefine,
                                const vector< vector< cv::Point2f > >& batchPointsToRefine,
                                const cv::Mat& cameraMatrix,
                                const cv::Mat& distortionCoefficients )
    {
        detection::DetectorConcentric* _detector = detection::DetectorConcentric::create( pSize );

        _detector->refineBatch( batchImagesToRefine, batchPointsToRefine, cameraMatrix, distortionCoefficients );
    }

    bool refineSingleConcentric( const cv::Size& pSize, 
                                 const cv::Mat& imageToRefine,
                                 const vector< cv::Point2f >& pointsToRefine,
                                 const cv::Mat& cameraMatrix,
                                 const cv::Mat& distortionCoefficients,
                                 cv::Mat& imageResult,
                                 vector< cv::Point2f >& pointsRefined )
    {
        detection::DetectorConcentric* _detector = detection::DetectorConcentric::create( pSize );

        return _detector->refineSingle( imageToRefine, 
                                        pointsToRefine, 
                                        cameraMatrix, 
                                        distortionCoefficients, 
                                        imageResult, 
                                        pointsRefined );
    }

    bool isRefiningConcentric( const cv::Size& pSize )
    {
        detection::DetectorConcentric* _detector = detection::DetectorConcentric::create( pSize );

        return _detector->isRefining();
    }

    bool hasRefinationToPickConcentric( const cv::Size& pSize )
    {
        detection::DetectorConcentric* _detector = detection::DetectorConcentric::create( pSize );

        return _detector->hasRefinationToPick();
    }

    void grabRefinationBatchConcentric( const cv::Size& pSize,
                                        vector< cv::Mat >& batchRefinedImages,
                                        vector< vector< cv::Point2f > >& batchRefinedPoints )
    {
        detection::DetectorConcentric* _detector = detection::DetectorConcentric::create( pSize );

        _detector->grabRefinationBatch( batchRefinedImages, batchRefinedPoints );
    }

    void updateConcentric( const cv::Size& pSize )
    {
        detection::DetectorConcentric* _detector = detection::DetectorConcentric::create( pSize );

        _detector->update();
    }


    namespace detection
    {

        DetectorConcentric* DetectorConcentric::INSTANCE = NULL;

        DetectorConcentric::DetectorConcentric( const cv::Size& size )
            : BaseDetector( size )
        {
            m_trackingPoints = vector< TrackingPoint >( m_numPoints );

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
        }

        DetectorConcentric::~DetectorConcentric()
        {

        }

        DetectorConcentric* DetectorConcentric::create( const cv::Size& size )
        {
            if ( DetectorConcentric::INSTANCE == NULL )
            {
                DetectorConcentric::INSTANCE = new DetectorConcentric( size );
            }
                
            return DetectorConcentric::INSTANCE;
        }

        void DetectorConcentric::release()
        {
            if ( DetectorConcentric::INSTANCE != NULL )
            {
                delete DetectorConcentric::INSTANCE;
                DetectorConcentric::INSTANCE = NULL;
            }
        }

        bool DetectorConcentric::run( const cv::Mat& input, const DetectionInfo& detInfo )
        {
            m_initialROI = detInfo.roi;

            m_frameSize = cv::Size( input.cols, input.rows );

            bool _ret = false;

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
            m_pipelinePanel->showMask( m_stepImageResults[ STEP_MASK ] );
            m_pipelinePanel->showEdges( m_stepImageResults[ STEP_EDGES ] );
            m_pipelinePanel->showBlobs( m_stepImageResults[ STEP_BLOBS ] );
            m_pipelinePanel->showTracking( m_stepImageResults[ STEP_TRACKING ] );

            m_pipelinePanel->cleanInfo();
            m_pipelinePanel->setLogInfo( getCurrentDetectionMode() );
            

            return _ret;
        }

        bool DetectorConcentric::runInitialDetectionMode( const cv::Mat& input )
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

        bool DetectorConcentric::_computeInitialPattern( const vector< cv::Point2f >& candidatePatternPoints,
                                                         vector< cv::Point2f >& matchedPoints, bool isFronto )
        {
            vector< cv::Point2f > _orderedKeypoints( m_numPoints );

            // Get bounding region to get the corners

            int _leftIndx   = 0;
            int _rightIndx  = 0;
            int _topIndx    = 0;
            int _bottomIndx = 0;

            if ( !isFronto )
            {
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
            }
            else
            {
                typedef priority_queue< int, vector< int >, ComparatorCenter > p_queue;

                p_queue pq( candidatePatternPoints );

                cv::Point2f center( 0, 0 );

                for( int i = 0; i < candidatePatternPoints.size(); i++ )
                {
                    pq.push(i);
                    center += candidatePatternPoints[i];
                }

                center /= float( candidatePatternPoints.size() );

                vector< int > _corners;
                _corners.push_back( pq.top() ); pq.pop();
                _corners.push_back( pq.top() ); pq.pop();
                _corners.push_back( pq.top() ); pq.pop();
                _corners.push_back( pq.top() ); pq.pop();

                for( int i = 0; i < _corners.size(); i++)
                {
                    int q = _corners[i];

                    if ( candidatePatternPoints[q].x < center.x &&
                         candidatePatternPoints[q].y < center.y )
                    {
                        _topIndx = q;
                    }

                    if ( candidatePatternPoints[q].x > center.x &&
                         candidatePatternPoints[q].y < center.y )
                    {
                        _rightIndx = q;
                    }

                    if ( candidatePatternPoints[q].y > center.y &&
                         candidatePatternPoints[q].x > center.x )
                    {
                        _bottomIndx = q;
                    }

                    if ( candidatePatternPoints[q].y > center.y &&
                         candidatePatternPoints[q].x < center.x )
                    {
                        _leftIndx = q;
                    }
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
                    break;
                }
            }

            return _isFit;
        }

        bool DetectorConcentric::runTrackingMode( const cv::Mat& input, const DetectionInfo& detInfo )
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

        bool DetectorConcentric::runRecoveringMode( const cv::Mat& input )
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

        void DetectorConcentric::_pipeline( const cv::Mat& input )
        {
            m_stepImageResults.clear();
            m_frame = input;

            m_workingInput = input;

            // thresholding step
            _runMaskGenerator( m_workingInput, 
                               m_stepImageResults[ STEP_MASK ] );
            // edge detection step
            _runEdgesGenerator( m_stepImageResults[ STEP_MASK ], 
                                m_stepImageResults[ STEP_EDGES ] );
            // features extractor step
            _runFeaturesExtractor( m_stepImageResults[ STEP_EDGES ],
                                   m_stepImageResults[ STEP_BLOBS ] );
            // tracking step
            _runTracking( m_stepImageResults[ STEP_BLOBS ],
                          m_stepImageResults[ STEP_TRACKING ] );


        }

        void DetectorConcentric::_runMaskGenerator( const cv::Mat& input, cv::Mat& output )
        {
            cv::Mat _grayScale;

            cv::cvtColor( input, _grayScale, CV_RGB2GRAY );
            cv::adaptiveThreshold( _grayScale, output, PIPELINE_MASKING_STAGE_MAX_VALUE,
                                   cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV,
                                   PIPELINE_MASKING_STAGE_BLOCKSIZE, PIPELINE_MASKING_STAGE_C );
        }

        void DetectorConcentric::_runEdgesGenerator( const cv::Mat& input, cv::Mat& output )
        {
            cv::Mat _gradX, _gradY, _gradAbsX, _gradAbsY;

            cv::Scharr( input, _gradX, CV_16S, 1, 0,
                        PIPELINE_EDGES_STAGE_SCALE, PIPELINE_EDGES_STAGE_DELTA, cv::BORDER_DEFAULT );
            cv::convertScaleAbs( _gradX, _gradAbsX );

            cv::Scharr( input, _gradY, CV_16S, 0, 1,
                        PIPELINE_EDGES_STAGE_SCALE, PIPELINE_EDGES_STAGE_DELTA, cv::BORDER_DEFAULT );
            cv::convertScaleAbs( _gradY, _gradAbsY );

            cv::addWeighted( _gradAbsX, 0.5, _gradAbsY, 0.5, 0, output );
        }

        void DetectorConcentric::_runFeaturesExtractor( const cv::Mat& input, cv::Mat& output )
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
        }

        void DetectorConcentric::_runTracking( const cv::Mat& input, cv::Mat& output )
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

                        if ( !_assigned[q] && utils::dist( m_trackingPoints[t].pos, _candidate ) < 10 )
                        {
                            m_trackingPoints[t].vel = cv::Point2f( 0, 0 );
                            m_trackingPoints[t].pos = _candidate;
                            m_trackingPoints[t].found = true;
                            _assigned[q] = true;
                            break;
                        }
                    }
                }

                // for ( int t = 0; t < m_trackingPoints.size(); t++ )
                // {
                //     if ( !m_trackingPoints[t].found )
                //     {
                //         cout << "tp: " << m_trackingPoints[t].pos << endl;
                //     }
                // }

                // for ( int q = 0; q < m_candidatePoints.size(); q++ )
                // {
                //     if ( !_assigned[q] )
                //     {
                //         cout << "cp: " << m_candidatePoints[q] << endl;
                //     }
                // }
                        
            }
            else
            {
                output = input.clone();
            }
        }

        void DetectorConcentric::getDetectedPoints( vector< cv::Point2f >& iPoints )
        {
            for ( int q = 0; q < m_trackingPoints.size(); q++ )
            {
                iPoints.push_back( m_trackingPoints[q].pos );
            }
        }

        bool DetectorConcentric::_refiningDetectionInternal( const cv::Mat& input, 
                                                             vector< cv::Point2f >& frontoRefinedPoints )
        {
            bool _found = false;

            cv::Mat _thresholded, _edges, _features;

            _refiningMask( input, _thresholded );
            _refiningEdges( _thresholded, _edges );
            _found = _refiningFeatures( _edges, input, _features, frontoRefinedPoints );

            m_stepImageResults[ STEP_REFINING_MASK ] = _thresholded.clone();
            m_stepImageResults[ STEP_REFINING_EDGES ] = _edges.clone();
            m_stepImageResults[ STEP_REFINING_FEATURES ] = _features.clone();

            m_pipelinePanel->showRefMask( m_stepImageResults[ STEP_REFINING_MASK ] );
            m_pipelinePanel->showRefEdges( m_stepImageResults[ STEP_REFINING_EDGES ] );
            m_pipelinePanel->showRefFeatures( m_stepImageResults[ STEP_REFINING_FEATURES ] );

            return _found;
        }

        void DetectorConcentric::_refiningMask( const cv::Mat& input,
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

        void DetectorConcentric::_refiningEdges( const cv::Mat& input,
                                                 cv::Mat& output )
        {
            cv::blur( input, output, cv::Size( 3, 3 ) );

            cv::Canny( output, output, 
                       PIPELINE_REFINING_CANNY_MIN, 
                       PIPELINE_REFINING_CANNY_MAX, 
                       PIPELINE_REFINING_EDGES_BLOCK_SIZE );
        }

        bool DetectorConcentric::_refiningFeatures( const cv::Mat& input,
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

            cv::Point2f _agg;
            int _counts;
            vector< cv::Point2f > _candidatesFilteredPoints;
            vector< bool > _paired( _candidatePoints.size(), false );

            for ( int q = 0; q < _candidatePoints.size(); q++ )
            {
                if( _paired[q] )
                {
                    continue;
                }

                _agg = _candidatePoints[q];
                _counts = 1;

                for ( int p = q + 1; p < _candidatePoints.size(); p++ )
                {
                    if ( _paired[p] )
                    {
                        continue;
                    }
                    float _dx = _candidatePoints[q].x - _candidatePoints[p].x;
                    float _dy = _candidatePoints[q].y - _candidatePoints[p].y;
                    float _dist = sqrt( _dx * _dx + _dy * _dy );

                    if ( _dist < 10 )
                    {
                        _paired[p] = true;
                        _agg += _candidatePoints[p];
                        _counts++;
                    }
                }
                _agg /= float( _counts );
                _candidatesFilteredPoints.push_back( _agg );
            }

            patternPointsFronto.clear();

            return _computeInitialPattern( _candidatesFilteredPoints, patternPointsFronto, true );
        }

        string DetectorConcentric::getCurrentDetectionMode()
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