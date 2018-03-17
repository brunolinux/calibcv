
#include "calibrationBaseDetector.h"

using namespace std;

namespace calibration
{

    BaseDetector::BaseDetector( const cv::Size& size )
    {
        m_size = size;
        m_numPoints = size.width * size.height;

        m_isRefining = false;
        m_hasRefinedPoints = false;

        m_stepImageResults = vector< cv::Mat >( MAX_STAGES );
        m_pipelinePanel = calibcv::SPatternDetectorPanel::create();
    }

    BaseDetector::~BaseDetector()
    {
        m_batchImagesRefinationResult.clear();
        m_batchPointsRefined.clear();
        m_workingDataBatchImages.clear();
        m_workingDataBatchPoints.clear();
    }

    void BaseDetector::_refiningConversionDirect( const cv::Mat& image2refine,
                                                  cv::Mat& frontoView, cv::Mat& frontoTransform,
                                                  const vector< cv::Point2f >& patternPoints,
                                                  const cv::Mat& cameraMatrix,
                                                  const cv::Mat& distortionCoefficients,
                                                  bool showIntermediateResults )
    {
        cv::Mat _undistortedView;

        vector< cv::Point2f > _undistortedPatternPoints;

        cv::Size _frameSize = cv::Size( image2refine.cols, image2refine.rows );

        // transform the iamge to undistorted view
        cv::undistort( image2refine, _undistortedView, cameraMatrix, distortionCoefficients );
        // transform also the points into undistorted view
        cv::undistortPoints( patternPoints, _undistortedPatternPoints, 
                             cameraMatrix, distortionCoefficients,
                             cv::noArray(), cameraMatrix );

        // Create the points for the perspective mapping
        // Borders points with radial padding
        cv::Point2f _topLeft     = 2 * _undistortedPatternPoints[0] - 
                                   _undistortedPatternPoints[ m_size.width + 1 ];

        cv::Point2f _topRight    = 2 * _undistortedPatternPoints[ m_size.width - 1 ] - 
                                   _undistortedPatternPoints[ 2 * m_size.width - 2 ];

        cv::Point2f _bottomRight = 2 * _undistortedPatternPoints[ m_size.width * m_size.height - 1 ] - 
                                   _undistortedPatternPoints[ m_size.width * ( m_size.height - 1 ) - 2 ];

        cv::Point2f _bottomLeft  = 2 * _undistortedPatternPoints[ m_size.width * ( m_size.height - 1 ) ] - 
                                   _undistortedPatternPoints[ m_size.width * ( m_size.height - 2 ) + 1 ];

        vector< cv::Point2f > _src = { _topLeft, _topRight, _bottomRight, _bottomLeft };

        vector< cv::Point2f > _dst = { cv::Point2f( 0, 0 ), 
                                       cv::Point2f( _frameSize.width / 2, 0 ), 
                                       cv::Point2f( _frameSize.width / 2, _frameSize.height / 2 ), 
                                       cv::Point2f( 0, _frameSize.height / 2 ) };

        // Generate transformation matrix
        frontoTransform = cv::getPerspectiveTransform( _src, _dst );

        // Transform into fronto parallel view
        cv::warpPerspective( _undistortedView, 
                             frontoView, 
                             frontoTransform,
                             cv::Size( _frameSize.width / 2, _frameSize.height / 2 ) );

        if ( showIntermediateResults )
        {
            m_stepImageResults[ STEP_REFINING_UNDISTORTED ] = _undistortedView;
            m_stepImageResults[ STEP_REFINING_FRONTO ] = frontoView.clone();

            for ( int q = 0; q < _undistortedPatternPoints.size(); q++ )
            {
                cv::circle( m_stepImageResults[ STEP_REFINING_UNDISTORTED ],
                            _undistortedPatternPoints[q], 2, cv::Scalar( 255, 255, 0 ), -1 );
            }
        }
    }


    bool BaseDetector::_refiningDetection( const cv::Mat& input, vector< cv::Point2f >& frontoRefinedPoints,
                                           bool showIntermediateResults )
    {
        bool _found = _refiningDetectionInternal( input, refinedPoints );

        if ( showIntermediateResults && _found )
        {
            m_stepImageResults[ STEP_REFINING_FEATURES ] = input.clone();

            for ( int q = 0; q < refinedPoints.size(); q++ )
            {
                cv::circle( m_stepImageResults[ STEP_REFINING_FEATURES ],
                            refinedPoints[q], 2, cv::Scalar( 255, 255, 0 ), -1 )
            }
        }

        return _found;
    }

    void BaseDetector::_refiningDetectionInternal( const cv::Mat& input, vector< cv::Point2f >& frontoRefinedPoints,
                                                   bool showIntermediateResults )
    {
        // Override this for each type of detector
    }

    void BaseDetector::_refiningConversionInverse( const vector< cv::Point2f>& frontoRefinedPoints,
                                                   vector< cv::Point2f >& refinedPoints,
                                                   cv::Mat& refinedImageResult,
                                                   const cv::Mat& inverseFrontoTransform,
                                                   const cv::Mat& originalView,
                                                   const vector< cv::Point2f >& originalPoints,
                                                   const cv::Mat& cameraMatrix,
                                                   const cv::Mat& distortionCoefficients,
                                                   bool showIntermediateResults )
    {
        vector< cv::Point2f > _patternPointsRefinedUndistorted;

        // convert points in fronto view to projected undistorted view

        for ( cv::Point2f _frontoPoint : frontoRefinedPoints )
        {
            cv::Mat2f _dstPoint;
            cv::Mat2f _srcPoint( _frontoPoint );

            cv::perspectiveTransform( _srcPoint, _dstPoint, inverseFrontoTransform );
            _patternPointsRefinedUndistorted.push_back( cv::Point2f( _dstPoint( 0 ) ) );
        }

        utils::distortPoints( _patternPointsRefinedUndistorted,
                              refinedPoints, cameraMatrix, distortionCoefficients );

        refinedImageResult = originalView.clone();

        for ( int q = 0; q < originalPoints.size(); q++ )
        {
            cv::circle( refinedImageResult, originalPoints[q], 
                        2, cv::Scalar( 0, 255, 255 ), -1 );
        }

        for ( int q = 0; q < refinedPoints.size(); q++ )
        {
            cv::circle( refinedImageResult, refinedPoints[q], 
                        2, cv::Scalar( 255, 255, 0 ), -1 );
        }

        if ( showIntermediateResults )
        {
            m_stepImageResults[ STEP_REFINING_DISTORTED ] = refinedImageResult.clone();
        }

    }

    void BaseDetector::refineBatch( const vector< cv::Mat >& batchImagesToRefine,
                                    const vector< vector< cv::Point2f > >& batchPointsToRefine,
                                    const cv::Mat& cameraMatrix,
                                    const cv::Mat& distortionCoefficients,
                                    bool useThreads,
                                    bool showIntermediateResults )
    {
        assert( batchImagesToRefine.size() == batchPointsToRefine.size() );

        m_batchPointsRefined.clear();
        m_batchImagesRefinationResult.clear();

        if ( useThreads )
        {
            m_hasRefinedPoints = false;
            m_isRefining = true;

            m_workingDataBatchImages.clear();
            m_workingDataBatchPoints.clear();

            m_workingDataCameraMatrix = cameraMatrix;
            m_workingDataDistortionCoefficients = distortionCoefficients;
            m_workingDataBatchPoints = batchPointsToRefine;
            m_workingDataBatchImages = batchImagesToRefine;

            pthread_create( &m_threadHandle, NULL, BaseDetector::refinerWorker, ( void* ) this );
        }
        else
        {
            _refineBatchInternal( batchImagesToRefine,
                                  batchPointsToRefine,
                                  cameraMatrix,
                                  distortionCoefficients );

            m_hasRefinedPoints = true;
            m_isRefining = false;
        }
    }

    void BaseDetector::_refineBatchInternal( const vector< cv::Mat >& batchImagesToRefine,
                                             const vector< vector< cv::Point2f > >& batchPointsToRefine,
                                             const cv::Mat& cameraMatrix,
                                             const cv::Mat& distortionCoefficients )
    {
        int _numFound = 0;
        for ( int q = 0; q < batchImagesToRefine.size(); q++ )
        {
            cv::Mat _frontoView, _frontoTransform, _refinationResult;
            vector< cv::Point2f > _refinedPoints, _refinedPointsFronto;

            _refiningConversionDirect( batchImagesToRefine[q], 
                                       _frontoView, _frontoTransform,
                                       batchPointsToRefine[q], cameraMatrix,
                                       distortionCoefficients, showIntermediateResults );

            bool _found = _refiningDetection( _frontoView, _refinedPointsFronto, showIntermediateResults );

            if ( !_found )
            {
                cout << "Skipping frame, didn't find pattern in fronto view" << endl;
                break;
            }

            _numFound++;

            _refiningConversionInverse( _refinedPointsFronto, 
                                        _refinedPoints,
                                        _refinationResult,
                                        _frontoTransform.inv(),
                                        batchImagesToRefine[q],
                                        batchPointsToRefine[q],
                                        cameraMatrix,
                                        distortionCoefficients,
                                        showIntermediateResults );

            m_batchImagesRefinationResult.push_back( _refinationResult );
            m_batchPointsRefined.push_back( _refinedPoints );
        }

        cout << "refined " << _numFound << " / " << batchImagesToRefine.size() << " in batch" << endl;
    }

    static void* BaseDetector::refinerWorker( void* pDetector )
    {
        BaseDetector* _detector = ( BaseDetector* ) pDetector;

        _detector->_refineBatchInternal( _detector->m_workingDataBatchImages,
                                         _detector->m_workingDataBatchPoints,
                                         _detector->m_workingDataCameraMatrix,
                                         _detector->m_workingDataDistortionCoefficients );

        _detector->m_hasRefinedPoints = true;
    }

    void BaseDetector::update()
    {
        if ( m_isRefining )
        {
            if ( m_hasRefinedPoints )
            {
                pthread_join( m_threadHandle, NULL );
                m_isRefining = false;
            }
        }

        m_pipelinePanel->showRefUndistorted( m_stepImageResults[ STAGE_REFINING_UNDISTORTED ] );
        m_pipelinePanel->showRefFronto( m_stepImageResults[ STAGE_REFINING_FRONTO ] );
        m_pipelinePanel->showRefProjected( m_stepImageResults[ STAGE_REFINING_PROJECTED ] );
        m_pipelinePanel->showRefDistorted( m_stepImageResults[ STAGE_REFINING_DISTORTED ] );
    }

    void BaseDetector::grabRefinationBatch( vector< cv::Mat >& batchRefinedImages,
                                            vector< vector< cv::Point2f > >& batchRefinedPoints )
    {
        if ( !m_hasRefinedPoints )
        {
            cout << "Wait!, there is no data at the moment :(" << endl;
            return;
        }

        assert( m_batchImagesRefinationResult.size() == m_batchPointsRefined.size() );

        // Just in case
        batchRefinedImages.clear();
        batchRefinedPoints.clear();

        for ( int q = 0; q < m_batchImagesRefinationResult.size(); q++ )
        {
            batchRefinedImages.push_back( m_batchImagesRefinationResult[q] );
            batchRefinedPoints.push_back( m_batchPointsRefined[q] );
        }

        m_hasRefinedPoints = false;
    }

}