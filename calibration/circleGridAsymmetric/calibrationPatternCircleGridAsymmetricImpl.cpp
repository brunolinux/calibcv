

#include "calibrationPatternCircleGridAsymmetricImpl.h"


using namespace std;


namespace calibration { namespace circleGridAsymmetric {



    void drawCircleGridAsymmetricPatternCorners( const vector< cv::Point2f >& iCorners, cv::Mat& image )
    {
        cv::drawChessboardCorners( image, pInfo.size, iCorners, true );
    }

    bool findCircleGridAsymmetric( const cv::Mat& image, const cv::Size& pSize, 
                                   const DetectionInfo& detInfo,
                                   vector< cv::Point2f >& iCorners )
    {
        detection::DetectorCircleGridAsymmetric* _detector = detection::DetectorCircleGridAsymmetric::create( pSize );

        bool _found = _detector->run( image, detInfo );
        if ( _found )
        {
            _detector->getDetectedPoints( iCorners );
        }

        return _found;
    }

    void refineBatchCircleGridAsymmetric( const vector< cv::Mat >& batchImagesToRefine,
                                          const vector< vector< cv::Point2f > >& batchPointsToRefine,
                                          const cv::Mat& cameraMatrix,
                                          const cv::Mat& distortionCoefficients )
    {
        detection::DetectorCircleGridAsymmetric* _detector = detection::DetectorCircleGridAsymmetric::create( pSize );

        _detector->refineBatch( batchImagesToRefine, batchPointsToRefine, cameraMatrix, distortionCoefficients );
    }

    bool isRefining()
    {
        detection::DetectorCircleGridAsymmetric* _detector = detection::DetectorCircleGridAsymmetric::create( pSize );

        return _detector->isRefining();
    }

    bool hasRefinationToPick()
    {
        detection::DetectorCircleGridAsymmetric* _detector = detection::DetectorCircleGridAsymmetric::create( pSize );

        return _detector->hasRefinationToPick();
    }

    void grabRefinationBatch( vector< cv::Mat >& batchRefinedImages,
                              vector< CalibrationBucket >& batchBuckets )
    {
        detection::DetectorCircleGridAsymmetric* _detector = detection::DetectorCircleGridAsymmetric::create( pSize );

        _detector->grabRefinationBatch( batchImagesToRefine, batchBuckets );
    }


    namespace detection
    {



        DetectorCircleGridAsymmetric::DetectorCircleGridAsymmetric( const cv::Size& pSize )
            : BaseDetector( pSize )
        {
            
        }

        DetectorCircleGridAsymmetric::~DetectorCircleGridAsymmetric()
        {

        }

        DetectorCircleGridAsymmetric* DetectorCircleGridAsymmetric::INSTANCE = NULL;

        DetectorCircleGridAsymmetric* DetectorCircleGridAsymmetric::create( const cv::Size& pSize )
        {
            if ( DetectorCircleGridAsymmetric::INSTANCE == NULL )
            {
                DetectorCircleGridAsymmetric::INSTANCE = new DetectorCircleGridAsymmetric( pSize );
            }

            return DetectorCircleGridAsymmetric::INSTANCE;
        }

        void DetectorCircleGridAsymmetric::release()
        {
            if ( DetectorCircleGridAsymmetric::INSTANCE != NULL )
            {
                delete DetectorCircleGridAsymmetric::INSTANCE;
                DetectorCircleGridAsymmetric::INSTANCE = NULL;
            }
        }

        bool DetectorCircleGridAsymmetric::run( const cv::Mat& input, const DetectionInfo& detInfo )
        {
            m_patternPoints.clear();
            
            return cv::findCirclesGrid( image, m_size, m_patternPoints, cv::CALIB_CB_ASYMMETRIC_GRID );
        }

        bool DetectorCircleGridAsymmetric::_refiningDetectionInternal( const cv::Mat& input, vector< cv::Point2f >& frontoRefinedPoints,
                                                                       bool showIntermediateResults )
        {
            frontoRefinedPoints.clear();

            return cv::findCirclesGrid( input, m_size, frontoRefinedPoints );
        }


        void DetectorCircleGridAsymmetric::getDetectedPoints( vector< cv::Point2f >& iPoints )
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