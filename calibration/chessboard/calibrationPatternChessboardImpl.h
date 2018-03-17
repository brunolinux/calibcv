
#pragma once

#include "../calibrationCommon.h"
#include "../calibrationBaseDetector.h"

using namespace std;


namespace calibration { namespace chessboard {


    void drawChessboardPatternCorners( const vector< cv::Point2f >& iCorners, cv::Mat& image, const PatternInfo& pInfo );

    bool findChessboardGrid( const cv::Mat& image, const cv::Size& pSize, 
                             const DetectionInfo& detInfo,
                             vector< cv::Point2f >& iCorners );

    void refineBatchChessboard( const cv::Size& size,
                                const vector< cv::Mat >& batchImagesToRefine,
                                const vector< vector< cv::Point2f > >& batchPointsToRefine,
                                const cv::Mat& cameraMatrix,
                                const cv::Mat& distortionCoefficients );

    bool refineSingleChessboard( const cv::Size& pSize, 
                                 const cv::Mat& imageToRefine,
                                 const vector< cv::Point2f >& pointsToRefine,
                                 const cv::Mat& cameraMatrix,
                                 const cv::Mat& distortionCoefficients,
                                 cv::Mat& imageResult,
                                 vector< cv::Point2f >& pointsRefined );

    bool isRefiningChessboard( const cv::Size& size );

    bool hasRefinationToPickChessboard( const cv::Size& size );

    void grabRefinationBatchChessboard( const cv::Size& size,
                                        vector< cv::Mat >& batchRefinedImages,
                                        vector< vector< cv::Point2f > >& batchRefinedPoints );

    void updateChessboard( const cv::Size& size );

    namespace detection
    {


        class DetectorChessboard : public BaseDetector
        {

            protected :

            bool _refiningDetectionInternal( const cv::Mat& input, 
                                             vector< cv::Point2f >& frontoRefinedPoints ) override;

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