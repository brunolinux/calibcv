
#pragma once

#include "../calibrationCommon.h"
#include "calibrationPatternConcentricImpl.h"

using namespace std;

namespace calibration { namespace concentric {


    bool getCorners( vector< cv::Point2f >& iCorners, 
                     const cv::Mat& image, 
                     const PatternInfo& pInfo,
                     const DetectionInfo& dInfo )
    {
        return findConcentricGrid( image, pInfo.size, dInfo, iCorners );
    }

    void getKnownPlanePositions( vector< cv::Point3f >& kCorners, const PatternInfo& pInfo )
    {
        for ( int y = 0; y < pInfo.size.height; y++ )
        {
            for ( int x = 0; x < pInfo.size.width; x++ )
            {
                kCorners.push_back( cv::Point3f( y * pInfo.cb_squareLength, 
                                                 x * pInfo.cb_squareLength, 
                                                 0.0f ) );
            }
        }
    }


    void drawPatternCorners( const vector< cv::Point2f >& iCorners, cv::Mat& image, const PatternInfo& pInfo )
    {
        drawConcentricPatternCorners( iCorners, image, pInfo );
    }

    void refineBatch( const vector< cv::Mat >& batchImagesToRefine,
                      const cv::Mat& cameraMatrix,
                      const cv::Mat& distortionCoefficients )
    {
        refineBatchConcentric( batchImagesToRefine, cameraMatrix, distortionCoefficients );
    }

    bool isRefining()
    {
        return isRefiningConcentric();
    }

    bool hasRefinationToPick()
    {
        return hasRefinationToPickConcentric();
    }

    void grabRefinationBatch( vector< cv::Mat >& batchRefinedImages,
                              vector< CalibrationBucket >& batchBuckets )
    {
        grabRefinationBatchConcentric( batchRefinedImages, batchBuckets );
    }

}}