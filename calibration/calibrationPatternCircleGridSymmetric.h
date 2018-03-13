
#pragma once

#include "calibrationCommon.h"


namespace calibration { namespace circleGridSymmetric {


    bool getCorners( vector< cv::Point2f >& iCorners, 
                     const cv::Mat& image, 
                     const PatternInfo& pInfo,
                     const DetectionInfo& dInfo )
    {
        return cv::findCirclesGrid( image, pInfo.size, iCorners );
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
		cv::drawChessboardCorners( image, pInfo.size, iCorners, true );
    }


}}
