
#pragma once

#include "../calibrationCommon.h"
#include "calibrationPatternCircleGridSymmetricImpl.h"

using namespace std;

namespace calibration { namespace circleGridSymmetric {


    bool getCorners( vector< cv::Point2f >& iCorners, 
                     const cv::Mat& image, 
                     const PatternInfo& pInfo,
                     const DetectionInfo& dInfo );

    void getKnownPlanePositions( vector< cv::Point3f >& kCorners, const PatternInfo& pInfo );


    void drawPatternCorners( const vector< cv::Point2f >& iCorners, cv::Mat& image, const PatternInfo& pInfo );

    void refineBatch( const cv::Size& size,
                      const vector< cv::Mat >& batchImagesToRefine,
                      const vector< vector< cv::Point2f > >& batchPointsToRefine,
                      const cv::Mat& cameraMatrix,
                      const cv::Mat& distortionCoefficients );

    bool refineSingle( const cv::Size& size,
                       const cv::Mat& imageToRefine,
                       const vector< cv::Point2f >& pointsToRefine,
                       const cv::Mat& cameraMatrix,
                       const cv::Mat& distortionCoefficients,
                       cv::Mat& imageResult,
                       vector< cv::Point2f >& pointsRefined );

    bool isRefining( const cv::Size& size );

    bool hasRefinationToPick( const cv::Size& size );

    void grabRefinationBatch( const cv::Size& size,
                              vector< cv::Mat >& batchRefinedImages,
                              vector< vector< cv::Point2f > >& batchRefinedPoints );

    void update( const cv::Size& size );

}}
