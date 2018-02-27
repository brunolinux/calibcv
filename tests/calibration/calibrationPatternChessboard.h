
#pragma once

#include "calibrationCommon.h"


namespace calibration { namespace chessboard {


    bool getCorners( vector< cv::Point2f >& iCorners, const cv::Mat& image, const PatternInfo& pInfo )
    {
        return cv::findChessboardCorners( image, pInfo.cb_size, iCorners,
                                          CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE );
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
