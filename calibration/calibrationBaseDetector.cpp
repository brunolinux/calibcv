
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

        m_pipelinePanel = NULL;
        calibcv::SPatternDetectorPanel::release();
    }

    void BaseDetector::_refiningConversionDirect( const cv::Mat& image2refine,
                                                  const vector< cv::Point2f >& patternPoints,
                                                  const cv::Mat& cameraMatrix,
                                                  const cv::Mat& distortionCoefficients,
                                                  cv::Mat& frontoView, 
                                                  cv::Mat& frontoTransform,
                                                  cv::Mat& undistortedView,
                                                  vector< cv::Point2f >& undistortedPatternPoints )
    {
        cv::Size _frameSize = cv::Size( image2refine.cols, image2refine.rows );

        // transform the iamge to undistorted view
        cv::undistort( image2refine, undistortedView, cameraMatrix, distortionCoefficients );
        // transform also the points into undistorted view
        cv::undistortPoints( patternPoints, undistortedPatternPoints, 
                             cameraMatrix, distortionCoefficients,
                             cv::noArray(), cameraMatrix );

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
                                       cv::Point2f( _frameSize.width / 2, 0 ), 
                                       cv::Point2f( _frameSize.width / 2, _frameSize.height / 2 ), 
                                       cv::Point2f( 0, _frameSize.height / 2 ) };

        // Generate transformation matrix
        frontoTransform = cv::getPerspectiveTransform( _src, _dst );

        // Transform into fronto parallel view
        cv::warpPerspective( undistortedView, 
                             frontoView, 
                             frontoTransform,
                             cv::Size( _frameSize.width / 2, _frameSize.height / 2 ) );
    }


    bool BaseDetector::_refiningDetection( const cv::Mat& input, vector< cv::Point2f >& frontoRefinedPoints )
    {
        bool _found = _refiningDetectionInternal( input, frontoRefinedPoints );

        return _found;
    }

    bool BaseDetector::_refiningDetectionInternal( const cv::Mat& input, 
                                                   vector< cv::Point2f >& frontoRefinedPoints )
    {
        // Override this for each type of detector
        cout << "should override this" << endl;
        return false;
    }

    void BaseDetector::_refiningConversionInverse( const vector< cv::Point2f>& frontoRefinedPoints,
                                                   const cv::Mat& inverseFrontoTransform,
                                                   const cv::Mat& originalView,
                                                   const vector< cv::Point2f >& originalPoints,
                                                   const cv::Mat& cameraMatrix,
                                                   const cv::Mat& distortionCoefficients,
                                                   vector< cv::Point2f >& undistortedRefinedPoints,
                                                   vector< cv::Point2f >& refinedPoints,
                                                   cv::Mat& refinedImageResult )
    {
        // convert points in fronto view to projected undistorted view

        for ( cv::Point2f _frontoPoint : frontoRefinedPoints )
        {
            cv::Mat2f _dstPoint;
            cv::Mat2f _srcPoint( _frontoPoint );

            cv::perspectiveTransform( _srcPoint, _dstPoint, inverseFrontoTransform );
            undistortedRefinedPoints.push_back( cv::Point2f( _dstPoint( 0 ) ) );
        }

        utils::distortPoints( undistortedRefinedPoints,
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

    }

    bool BaseDetector::refineSingle( const cv::Mat& imageToRefine,
                                     const vector< cv::Point2f >& pointsToRefine,
                                     const cv::Mat& cameraMatrix,
                                     const cv::Mat& distortionCoefficients,
                                     cv::Mat& imageResult,
                                     vector< cv::Point2f >& pointsRefined )
    {
        bool _found = _refineSingleInternal( imageToRefine,
                                             pointsToRefine,
                                             cameraMatrix,
                                             distortionCoefficients,
                                             imageResult,
                                             pointsRefined );
        // if ( !_found )
        // {
        //     cout << "couldn't refine points for single request" << endl;
        // }

        return _found;
    }

    bool BaseDetector::_refineSingleInternal( const cv::Mat& imageToRefine,
                                              const vector< cv::Point2f >& pointsToRefine,
                                              const cv::Mat& cameraMatrix,
                                              const cv::Mat& distortionCoefficients,
                                              cv::Mat& imageResult,
                                              vector< cv::Point2f >& pointsRefined )
    {
        cv::Mat _undistortedView, 
                _frontoView, 
                _frontoTransform;

        vector< cv::Point2f > _undistortedPatternPoints,
                              _refinedPointsFronto,
                              _undistortedRefinedPoints;

        _refiningConversionDirect( imageToRefine, 
                                   pointsToRefine, 
                                   cameraMatrix,
                                   distortionCoefficients,
                                   _frontoView, 
                                   _frontoTransform,
                                   _undistortedView,
                                   _undistortedPatternPoints );

        // Draw this step for debugging ***********************************

        m_stepImageResults[ STEP_REFINING_UNDISTORTED ] = _undistortedView.clone();
        m_stepImageResults[ STEP_REFINING_FRONTO ] = _frontoView.clone();

        for ( int q = 0; q < _undistortedPatternPoints.size(); q++ )
        {
            cv::circle( m_stepImageResults[ STEP_REFINING_UNDISTORTED ],
                        _undistortedPatternPoints[q], 2, cv::Scalar( 255, 255, 0 ), -1 );
        }

        // ****************************************************************

        bool _found = _refiningDetection( _frontoView, _refinedPointsFronto );

        if ( !_found )
        {
            // cout << "Skipping frame, didn't find pattern in fronto view" << endl;
            return false;
        }

        // Draw this step for debugging ***********************************

        m_stepImageResults[ STEP_REFINING_FEATURES ] = _frontoView.clone();

        for ( int q = 0; q < _refinedPointsFronto.size(); q++ )
        {
            cv::circle( m_stepImageResults[ STEP_REFINING_FEATURES ],
                        _refinedPointsFronto[q], 2, cv::Scalar( 255, 255, 0 ), -1 );
        }

        // ****************************************************************

        _refiningConversionInverse( _refinedPointsFronto, 
                                    _frontoTransform.inv(),
                                    imageToRefine,
                                    pointsToRefine,
                                    cameraMatrix,
                                    distortionCoefficients,
                                    _undistortedRefinedPoints,
                                    pointsRefined,
                                    imageResult );

        m_stepImageResults[ STEP_REFINING_PROJECTED ] = _undistortedView.clone();

        for ( int q = 0; q < _undistortedRefinedPoints.size(); q++ )
        {
            cv::circle( m_stepImageResults[ STEP_REFINING_PROJECTED ], 
                        _undistortedRefinedPoints[q], 
                        2, cv::Scalar( 0, 255, 0 ), -1 );
        }

        m_stepImageResults[ STEP_REFINING_DISTORTED ] = imageResult.clone();

        return true;
    }

    void BaseDetector::refineBatch( const vector< cv::Mat >& batchImagesToRefine,
                                    const vector< vector< cv::Point2f > >& batchPointsToRefine,
                                    const cv::Mat& cameraMatrix,
                                    const cv::Mat& distortionCoefficients,
                                    bool useThreads )
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

            m_workingDataCameraMatrix = cameraMatrix.clone();
            m_workingDataDistortionCoefficients = distortionCoefficients.clone();

            for ( int q = 0; q < batchImagesToRefine.size(); q++ )
            {
                m_workingDataBatchPoints.push_back( batchPointsToRefine[q] );
                m_workingDataBatchImages.push_back( batchImagesToRefine[q].clone() );
            }

            pthread_create( &m_threadHandle, NULL, BaseDetector::refinerWorker, ( void* ) this );
            cout << "sent worker" << endl;
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
            cv::Mat _undistortedView, 
                    _frontoView, 
                    _frontoTransform, 
                    _refinationResult;

            vector< cv::Point2f > _refinedPoints, 
                                  _refinedPointsFronto, 
                                  _undistortedPatternPoints,
                                  _undistortedRefinedPoints;

            _refiningConversionDirect( batchImagesToRefine[q], 
                                       batchPointsToRefine[q],
                                       cameraMatrix,
                                       distortionCoefficients,
                                       _frontoView, 
                                       _frontoTransform,
                                       _undistortedView,
                                       _undistortedPatternPoints );

            bool _found = _refiningDetection( _frontoView, _refinedPointsFronto );

            if ( !_found )
            {
                // cout << "Skipping frame, didn't find pattern in fronto view" << endl;
                continue;
            }

            _numFound++;

            _refiningConversionInverse( _refinedPointsFronto, 
                                        _frontoTransform.inv(),
                                        batchImagesToRefine[q],
                                        batchPointsToRefine[q],
                                        cameraMatrix,
                                        distortionCoefficients,
                                        _undistortedRefinedPoints,
                                        _refinedPoints,
                                        _refinationResult );

            // cv::Mat _refinationResult;
            // vector< cv::Point2f > _refinedPoints;

            // bool _found = _refineSingleInternal( batchImagesToRefine[q],
            //                                      batchPointsToRefine[q],
            //                                      cameraMatrix,
            //                                      distortionCoefficients,
            //                                      _refinationResult,
            //                                      _refinedPoints );

            // if ( !_found )
            // {
            //     continue;
            // }

            // _numFound++;

            m_batchImagesRefinationResult.push_back( _refinationResult );
            m_batchPointsRefined.push_back( _refinedPoints );
        }

        cout << "refined " << _numFound << " / " << batchImagesToRefine.size() << " in batch" << endl;
    }

    void* BaseDetector::refinerWorker( void* pDetector )
    {
        BaseDetector* _detector = ( BaseDetector* ) pDetector;

        cout << "working ..." << endl;

        cout << "numbatch: " << _detector->m_workingDataBatchImages.size() << endl;

        _detector->_refineBatchInternal( _detector->m_workingDataBatchImages,
                                         _detector->m_workingDataBatchPoints,
                                         _detector->m_workingDataCameraMatrix,
                                         _detector->m_workingDataDistortionCoefficients );

        cout << "done" << endl;

        _detector->m_hasRefinedPoints = true;
    }

    void BaseDetector::update()
    {
        if ( m_isRefining )
        {
            if ( m_hasRefinedPoints )
            {
                cout << "stopped refiner thread" << endl;
                pthread_join( m_threadHandle, NULL );
                m_isRefining = false;
            }
            else
            {
                cout << "still refining" << endl;
            }
        }

        m_pipelinePanel->showRefUndistorted( m_stepImageResults[ STEP_REFINING_UNDISTORTED ] );
        m_pipelinePanel->showRefFronto( m_stepImageResults[ STEP_REFINING_FRONTO ] );
        m_pipelinePanel->showRefFeatures( m_stepImageResults[ STEP_REFINING_FEATURES ] );
        m_pipelinePanel->showRefProjected( m_stepImageResults[ STEP_REFINING_PROJECTED ] );
        m_pipelinePanel->showRefDistorted( m_stepImageResults[ STEP_REFINING_DISTORTED ] );
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

        if ( m_isRefining )
        {
            cout << "stopped refiner thread" << endl;
            pthread_join( m_threadHandle, NULL );
            m_isRefining = false;
        }

        m_hasRefinedPoints = false;
    }

}