
#pragma once

#include "../calibrationCommon.h"
#include "calibrationPatternCircleGridAsymmetricImpl.h"

using namespace std;

namespace calibration { namespace circleGridAsymmetric {


    bool getCorners( vector< cv::Point2f >& iCorners, 
                     const cv::Mat& image, 
                     const PatternInfo& pInfo,
                     const DetectionInfo& dInfo )
    {
        return findCircleGridAsymmetric( image, pInfo.size, dInfo, iCorners );
    }

    void getKnownPlanePositions( vector< cv::Point3f >& kCorners, const PatternInfo& pInfo )
    {
        for ( int y = 0; y < pInfo.size.height; y++ )
        {
            for ( int x = 0; x < pInfo.size.width; x++ )
            {
                kCorners.push_back( cv::Point3f( y * pInfo.cb_squareLength, 
                                                 ( 2 * x + y % 2 ) * pInfo.cb_squareLength, 
                                                 0.0f ) );
            }
        }
    }


    void drawPatternCorners( const vector< cv::Point2f >& iCorners, cv::Mat& image, const PatternInfo& pInfo )
    {
        drawCircleGridAsymmetricPatternCorners( iCorners, image, pInfo );
    }

    void refineBatch( const vector< cv::Mat >& batchImagesToRefine,
                      const vector< vector< cv::Point2f > >& batchPointsToRefine,
                      const cv::Mat& cameraMatrix,
                      const cv::Mat& distortionCoefficients )
    {
        refineBatchCircleGridAsymmetric( batchImagesToRefine, cameraMatrix, distortionCoefficients );
    }

    bool isRefining()
    {
        return isRefiningCircleGridAsymmetric();
    }

    bool hasRefinationToPick()
    {
        return hasRefinationToPickCircleGridAsymmetric();
    }

    void grabRefinationBatch( vector< cv::Mat >& batchRefinedImages,
                              vector< CalibrationBucket >& batchBuckets )
    {
        grabRefinationBatchCircleGridAsymmetric( batchRefinedImages, batchBuckets );
    }

}}
