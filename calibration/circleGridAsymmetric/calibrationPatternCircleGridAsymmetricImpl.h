
#pragma once

#include "calibrationCommon.h"
#include "../calibrationBaseDetector.h"

using namespace std;


namespace calibration { namespace circleGridAsymmetric {


    void drawCircleGridAsymmetricPatternCorners( const vector< cv::Point2f >& iCorners, cv::Mat& image );

    bool findCircleGridAsymmetric( const cv::Mat& image, const cv::Size& pSize, 
                                   const DetectionInfo& detInfo,
                                   vector< cv::Point2f >& iCorners );

    void refineBatchCircleGridAsymmetric( const vector< cv::Mat >& batchImagesToRefine,
                                          const vector< vector< cv::Point2f > >& batchPointsToRefine,
                                          const cv::Mat& cameraMatrix,
                                          const cv::Mat& distortionCoefficients );

    bool isRefiningCircleGridAsymmetric();

    bool hasRefinationToPickCircleGridAsymmetric();

    void grabRefinationBatchCircleGridAsymmetric( vector< cv::Mat >& batchRefinedImages,
                                                  vector< CalibrationBucket >& batchBuckets );



    namespace detection
    {


        class DetectorCircleGridAsymmetric : public BaseDetector
        {

            protected :

            bool _refiningDetectionInternal( const cv::Mat& input, vector< cv::Point2f >& frontoRefinedPoints,
                                             bool showIntermediateResults = false ) override;
            
            DetectorCircleGridAsymmetric( const cv::Size& size );

            public :

            static DetectorCircleGridAsymmetric* INSTANCE;
            static DetectorCircleGridAsymmetric* create( const cv::Size& size );
            static void release();

            ~DetectorCircleGridAsymmetric();

            bool run( const cv::Mat& input, const DetectionInfo& detInfo ) override;
            void getDetectedPoints( vector< cv::Point2f >& iPoints ) override;


        };


    }


}}