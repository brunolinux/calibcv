
#pragma once

#include "calibrationCommon.h"


namespace calibration { namespace chessboard {


    bool getCorners( vector< cv::Point2f >& iCorners, 
                     const cv::Mat& image, 
                     const PatternInfo& pInfo, 
                     const DetectionInfo& dInfo )
    {
        return cv::findChessboardCorners( image, pInfo.size, iCorners,
                                          CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE );
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
