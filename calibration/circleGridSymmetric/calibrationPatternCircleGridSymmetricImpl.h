
#pragma once

#include "calibrationCommon.h"
#include "../calibrationBaseDetector.h"

using namespace std;


namespace calibration { namespace circleGridSymmetric {


    void drawCircleGridSymmetricPatternCorners( const vector< cv::Point2f >& iCorners, cv::Mat& image );

    bool findCircleGridSymmetric( const cv::Mat& image, const cv::Size& pSize, 
                                  const DetectionInfo& detInfo,
                                  vector< cv::Point2f >& iCorners );

    void refineBatchCircleGridSymmetric( const vector< cv::Mat >& batchImagesToRefine,
                                         const vector< vector< cv::Point2f > >& batchPointsToRefine,
                                         const cv::Mat& cameraMatrix,
                                         const cv::Mat& distortionCoefficients );

    bool isRefiningCircleGridSymmetric();

    bool hasRefinationToPickCircleGridSymmetric();

    void grabRefinationBatchCircleGridSymmetric( vector< cv::Mat >& batchRefinedImages,
                                                  vector< CalibrationBucket >& batchBuckets );



    namespace detection
    {


        class DetectorCircleGridSymmetric : public BaseDetector
        {

            protected :
            
            bool _refiningDetectionInternal( const cv::Mat& input, vector< cv::Point2f >& frontoRefinedPoints,
                                             bool showIntermediateResults = false ) override;

            DetectorCircleGridSymmetric( const cv::Size& size );

            public :

            static DetectorCircleGridSymmetric* INSTANCE;
            static DetectorCircleGridSymmetric* create( const cv::Size& size );
            static void release();

            ~DetectorCircleGridSymmetric();

            bool run( const cv::Mat& input, const DetectionInfo& detInfo ) override;
            void getDetectedPoints( vector< cv::Point2f >& iPoints ) override;


        };


    }


}}