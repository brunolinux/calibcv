

#include "calibrationPatternChessboardImpl.h"


using namespace std;


namespace calibration { namespace chessboard {



    void drawChessboardPatternCorners( const vector< cv::Point2f >& iCorners, cv::Mat& image )
    {
        cv::drawChessboardCorners( image, pInfo.size, iCorners, true );
    }

    bool findChessboardGrid( const cv::Mat& image, const cv::Size& pSize, 
                             const DetectionInfo& detInfo,
                             vector< cv::Point2f >& iCorners )
    {
        detection::DetectorChessboard* _detector = detection::DetectorChessboard::create( pSize );

        bool _found = _detector->run( image, detInfo );
        if ( _found )
        {
            _detector->getDetectedPoints( iCorners );
        }

        return _found;
    }

    void refineBatchChessboard( const vector< cv::Mat >& batchImagesToRefine,
                                const vector< vector< cv::Point2f > >& batchPointsToRefine,
                                const cv::Mat& cameraMatrix,
                                const cv::Mat& distortionCoefficients )
    {
        detection::DetectorChessboard* _detector = detection::DetectorChessboard::create( pSize );

        _detector->refineBatch( batchImagesToRefine, batchPointsToRefine, cameraMatrix, distortionCoefficients );
    }

    bool isRefining()
    {
        detection::DetectorChessboard* _detector = detection::DetectorChessboard::create( pSize );

        return _detector->isRefining();
    }

    bool hasRefinationToPick()
    {
        detection::DetectorChessboard* _detector = detection::DetectorChessboard::create( pSize );

        return _detector->hasRefinationToPick();
    }

    void grabRefinationBatch( vector< cv::Mat >& batchRefinedImages,
                              vector< CalibrationBucket >& batchBuckets )
    {
        detection::DetectorChessboard* _detector = detection::DetectorChessboard::create( pSize );

        _detector->grabRefinationBatch( batchImagesToRefine, batchBuckets );
    }


    namespace detection
    {



        DetectorChessboard::DetectorChessboard( const cv::Size& pSize )
            : BaseDetector( pSize )
        {
            
        }

        DetectorChessboard::~DetectorChessboard()
        {

        }

        DetectorChessboard* DetectorChessboard::INSTANCE = NULL;

        DetectorChessboard* DetectorChessboard::create( const cv::Size& pSize )
        {
            if ( DetectorChessboard::INSTANCE == NULL )
            {
                DetectorChessboard::INSTANCE = new DetectorChessboard( pSize );
            }

            return DetectorChessboard::INSTANCE;
        }

        void DetectorChessboard::release()
        {
            if ( DetectorChessboard::INSTANCE != NULL )
            {
                delete DetectorChessboard::INSTANCE;
                DetectorChessboard::INSTANCE = NULL;
            }
        }

        bool DetectorChessboard::run( const cv::Mat& input, const DetectionInfo& detInfo )
        {
            m_patternPoints.clear();

            return cv::findChessboardCorners( image, m_size, m_patternPoints,
                                              CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE );
        }

        bool DetectorChessboard::_refiningDetectionInternal( const cv::Mat& input, vector< cv::Point2f >& frontoRefinedPoints,
                                                             bool showIntermediateResults )
        {
            frontoRefinedPoints.clear();

            return cv::findChessboardCorners( input, m_size, frontoRefinedPoints,
                                              CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE );
        }

        void DetectorChessboard::getDetectedPoints( vector< cv::Point2f >& iPoints )
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