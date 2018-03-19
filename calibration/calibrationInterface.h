
#pragma once

#include <sstream>
#include <cmath>
#include <ctime>

#include "calibrationCommon.h"
#include "calibrationPatternUtils.h"

#include "chessboard/calibrationPatternChessboard.h"
#include "circleGridSymmetric/calibrationPatternCircleGridSymmetric.h"
#include "circleGridAsymmetric/calibrationPatternCircleGridAsymmetric.h"
#include "concentric/calibrationPatternConcentric.h"

using namespace std;

namespace calibration
{

    
    bool getPatternKnownPlanePositions( vector< cv::Point3f >& corners, const PatternInfo& pInfo );
  
    bool getPatternCorners( vector< cv::Point2f >& iCorners, const cv::Mat& image, const PatternInfo& pInfo, const DetectionInfo& dInfo );

    void drawPatternCorners( const vector< cv::Point2f >& iCorners, cv::Mat& image, const PatternInfo& pInfo );

    void requestBatchRefinment( const PatternInfo& pInfo,
                                const vector< cv::Mat >& batchImagesToRefine,
                                const vector< vector< cv::Point2f > >& batchPointsToRefine,
                                const cv::Mat& cameraMatrix,
                                const cv::Mat& distortionCoefficients );

    bool requestSingleRefinment( const PatternInfo& pInfo,
                                 const cv::Mat& imageToRefine,
                                 const vector< cv::Point2f >& pointsToRefine,
                                 const cv::Mat& cameraMatrix,
                                 const cv::Mat& distortionCoefficients,
                                 cv::Mat& imageResult,
                                 vector< cv::Point2f >& pointsRefined );

    bool isRefining( const PatternInfo& pInfo );

    bool hasRefinationToPick( const PatternInfo& pInfo );

    void grabRefinationBatch( const PatternInfo& pInfo, 
                              vector< cv::Mat >& batchRefinedImages,
                              vector< vector< cv::Point2f > >& batchRefinedPoints );

    void update( const PatternInfo& pInfo );
}