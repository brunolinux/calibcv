

#include "calibrationPatternCircleGridSymmetricImpl.h"


using namespace std;


namespace calibration { namespace circleGridSymmetric {



    void drawCircleGridSymmetricPatternCorners( const vector< cv::Point2f >& iCorners, cv::Mat& image )
    {
        cv::drawChessboardCorners( image, pInfo.size, iCorners, true );
    }

    bool findCircleGridSymmetric( const cv::Mat& image, const cv::Size& pSize, 
                                   const DetectionInfo& detInfo,
                                   vector< cv::Point2f >& iCorners )
    {
        detection::DetectorCircleGridSymmetric* _detector = detection::DetectorCircleGridSymmetric::create( pSize );

        bool _found = _detector->run( image, detInfo );
        if ( _found )
        {
            _detector->getDetectedPoints( iCorners );
        }

        return _found;
    }

    void refineBatchCircleGridSymmetric( const vector< cv::Mat >& batchImagesToRefine,
                                          const vector< vector< cv::Point2f > >& batchPointsToRefine,
                                          const cv::Mat& cameraMatrix,
                                          const cv::Mat& distortionCoefficients )
    {
        detection::DetectorCircleGridSymmetric* _detector = detection::DetectorCircleGridSymmetric::create( pSize );

        _detector->refineBatch( batchImagesToRefine, batchPointsToRefine, cameraMatrix, distortionCoefficients );
    }

    bool isRefining()
    {
        detection::DetectorCircleGridSymmetric* _detector = detection::DetectorCircleGridSymmetric::create( pSize );

        return _detector->isRefining();
    }

    bool hasRefinationToPick()
    {
        detection::DetectorCircleGridSymmetric* _detector = detection::DetectorCircleGridSymmetric::create( pSize );

        return _detector->hasRefinationToPick();
    }

    void grabRefinationBatch( vector< cv::Mat >& batchRefinedImages,
                              vector< CalibrationBucket >& batchBuckets )
    {
        detection::DetectorCircleGridSymmetric* _detector = detection::DetectorCircleGridSymmetric::create( pSize );

        _detector->grabRefinationBatch( batchImagesToRefine, batchBuckets );
    }


    namespace detection
    {



        DetectorCircleGridSymmetric::DetectorCircleGridSymmetric( const cv::Size& pSize )
            : BaseDetector( pSize )
        {
            
        }

        DetectorCircleGridSymmetric::~DetectorCircleGridSymmetric()
        {

        }

        DetectorCircleGridSymmetric* DetectorCircleGridSymmetric::INSTANCE = NULL;

        DetectorCircleGridSymmetric* DetectorCircleGridSymmetric::create( const cv::Size& pSize )
        {
            if ( DetectorCircleGridSymmetric::INSTANCE == NULL )
            {
                DetectorCircleGridSymmetric::INSTANCE = new DetectorCircleGridSymmetric( pSize );
            }

            return DetectorCircleGridSymmetric::INSTANCE;
        }

        void DetectorCircleGridSymmetric::release()
        {
            if ( DetectorCircleGridSymmetric::INSTANCE != NULL )
            {
                delete DetectorCircleGridSymmetric::INSTANCE;
                DetectorCircleGridSymmetric::INSTANCE = NULL;
            }
        }

        bool DetectorCircleGridSymmetric::run( const cv::Mat& input, const DetectionInfo& detInfo )
        {
            m_patternPoints.clear();
            
            return cv::findCirclesGrid( image, m_size, m_patternPoints, cv::CALIB_CB_ASYMMETRIC_GRID );
        }

        bool DetectorCircleGridSymmetric::_refiningDetectionInternal( const cv::Mat& input, vector< cv::Point2f >& frontoRefinedPoints,
                                                                      bool showIntermediateResults )
        {
            frontoRefinedPoints.clear();

            return cv::findCirclesGrid( input, m_size, frontoRefinedPoints, cv::CALIB_CB_ASYMMETRIC_GRID );
        }

        void DetectorCircleGridSymmetric::getDetectedPoints( vector< cv::Point2f >& iPoints )
        {
            // Just in case
            iPoints.clear();

            for ( int q = 0; q < m_patternPoints.size(); q++ )
            {
                iPoints.push_back( m_patternPoints[q] );
            }
        }

    }







}}