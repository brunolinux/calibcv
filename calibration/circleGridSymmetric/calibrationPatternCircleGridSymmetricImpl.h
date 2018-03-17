
#pragma once

#include "../calibrationCommon.h"
#include "../calibrationBaseDetector.h"

using namespace std;


namespace calibration { namespace circleGridSymmetric {


    void drawCircleGridSymmetricPatternCorners( const vector< cv::Point2f >& iCorners, cv::Mat& image, const PatternInfo& pInfo );

    bool findCircleGridSymmetric( const cv::Mat& image, const cv::Size& pSize, 
                                  const DetectionInfo& detInfo,
                                  vector< cv::Point2f >& iCorners );

    void refineBatchCircleGridSymmetric( const cv::Size& pSize,
                                         const vector< cv::Mat >& batchImagesToRefine,
                                         const vector< vector< cv::Point2f > >& batchPointsToRefine,
                                         const cv::Mat& cameraMatrix,
                                         const cv::Mat& distortionCoefficients );

    bool refineSingleCircleGridSymmetric( const cv::Size& pSize, 
                                          const cv::Mat& imageToRefine,
                                          const vector< cv::Point2f >& pointsToRefine,
                                          const cv::Mat& cameraMatrix,
                                          const cv::Mat& distortionCoefficients,
                                          cv::Mat& imageResult,
                                          vector< cv::Point2f >& pointsRefined );

    bool isRefiningCircleGridSymmetric( const cv::Size& pSize );

    bool hasRefinationToPickCircleGridSymmetric( const cv::Size& pSize );

    void grabRefinationBatchCircleGridSymmetric( const cv::Size& pSize,
                                                 vector< cv::Mat >& batchRefinedImages,
                                                 vector< vector< cv::Point2f > >& batchRefinedPoints );


    void updateCircleGridSymmetric( const cv::Size& pSize );

    namespace detection
    {


        class DetectorCircleGridSymmetric : public BaseDetector
        {

            protected :
            
            bool _refiningDetectionInternal( const cv::Mat& input, 
                                             vector< cv::Point2f >& frontoRefinedPoints ) override;

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