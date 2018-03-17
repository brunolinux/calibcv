
#pragma once

#include "../calibrationCommon.h"
#include "../calibrationBaseDetector.h"

using namespace std;


namespace calibration { namespace circleGridAsymmetric {


    void drawCircleGridAsymmetricPatternCorners( const vector< cv::Point2f >& iCorners, cv::Mat& image, const PatternInfo& pInfo );

    bool findCircleGridAsymmetric( const cv::Mat& image, const cv::Size& pSize, 
                                   const DetectionInfo& detInfo,
                                   vector< cv::Point2f >& iCorners );

    void refineBatchCircleGridAsymmetric( const cv::Size& size,
                                          const vector< cv::Mat >& batchImagesToRefine,
                                          const vector< vector< cv::Point2f > >& batchPointsToRefine,
                                          const cv::Mat& cameraMatrix,
                                          const cv::Mat& distortionCoefficients );

    void refineSingleCircleGridAsymmetric( const cv::Size& pSize, 
                                           const cv::Mat& imageToRefine,
                                           const vector< cv::Point2f >& pointsToRefine,
                                           const cv::Mat& cameraMatrix,
                                           const cv::Mat& distortionCoefficients,
                                           cv::Mat& imageResult,
                                           vector< cv::Point2f >& pointsRefined );

    bool isRefiningCircleGridAsymmetric( const cv::Size& size );

    bool hasRefinationToPickCircleGridAsymmetric( const cv::Size& size );

    void grabRefinationBatchCircleGridAsymmetric( const cv::Size& size,
                                                  vector< cv::Mat >& batchRefinedImages,
                                                  vector< vector< cv::Point2f > >& batchRefinedPoints );



    namespace detection
    {


        class DetectorCircleGridAsymmetric : public BaseDetector
        {

            protected :

            bool _refiningDetectionInternal( const cv::Mat& input, 
                                             vector< cv::Point2f >& frontoRefinedPoints ) override;
            
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