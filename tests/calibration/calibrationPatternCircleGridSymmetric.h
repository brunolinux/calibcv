
#pragma once

#include "calibrationCommon.h"


namespace calibration { namespace circleGridSymmetric {


    bool getCorners( vector< cv::Point2f >& iCorners, const cv::Mat& image, const PatternInfo& pInfo )
    {
        return cv::findCirclesGrid( image, pInfo.cb_size, iCorners );
    }

    void getKnownPlanePositions( vector< cv::Point3f >& kCorners, const PatternInfo& pInfo )
    {
        for ( int x = 0; x < pInfo.cb_size.width; x++ )
        {
            for ( int y = 0; y < pInfo.cb_size.height; y++ )
            {
                kCorners.push_back( cv::Point3f( x * pInfo.cb_squareLength, 
                                                 y * pInfo.cb_squareLength, 
                                                 0.0f ) );
            }
        }
    }


    void drawPatternCorners( const vector< cv::Point2f >& iCorners, cv::Mat& image, const PatternInfo& pInfo )
    {
		cv::drawChessboardCorners( image, pInfo.cb_size, iCorners, true );
    }


}}
