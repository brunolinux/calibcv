
#pragma once

#include "calibrationCommon.h"
#include "../calibrationBaseDetector.h"

using namespace std;


namespace calibration { namespace chessboard {


    void drawChessboardPatternCorners( const vector< cv::Point2f >& iCorners, cv::Mat& image );

    bool findChessboardGrid( const cv::Mat& image, const cv::Size& pSize, 
                             const DetectionInfo& detInfo,
                             vector< cv::Point2f >& iCorners );

    void refineBatchChessboard( const vector< cv::Mat >& batchImagesToRefine,
                                const vector< vector< cv::Point2f > >& batchPointsToRefine,
                                const cv::Mat& cameraMatrix,
                                const cv::Mat& distortionCoefficients );

    bool isRefiningChessboard();

    bool hasRefinationToPickChessboard();

    void grabRefinationBatchChessboard( vector< cv::Mat >& batchRefinedImages,
                                        vector< CalibrationBucket >& batchBuckets );



    namespace detection
    {


        class DetectorChessboard : public BaseDetector
        {

            protected :

            bool _refiningDetectionInternal( const cv::Mat& input, vector< cv::Point2f >& frontoRefinedPoints,
                                             bool showIntermediateResults = false ) override;

            DetectorChessboard( const cv::Size& size );

            public :

            static DetectorChessboard* INSTANCE;
            static DetectorChessboard* create( const cv::Size& size );
            static void release();

            ~DetectorChessboard();

            bool run( const cv::Mat& input, const DetectionInfo& detInfo ) override;
            void getDetectedPoints( vector< cv::Point2f >& iPoints ) override;


        };



    }




}}