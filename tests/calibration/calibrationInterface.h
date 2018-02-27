
#pragma once

#include "calibrationCommon.h"

#include "calibrationPatternChessboard.h"
#include "calibrationPatternCircleGridSymmetric.h"
#include "calibrationPatternCircleGridAsymmetric.h"

using namespace std;


namespace calibration
{

    
    bool getPatternKnownPlanePositions( vector< cv::Point3f >& corners, const PatternInfo& pInfo )
    {
        bool success = true;
        
        switch ( pInfo.type )
        {
            case PATTERN_TYPE_CHESSBOARD :
                
                chessboard::getKnownPlanePositions( corners, pInfo );
                
                break;
                
            case PATTERN_TYPE_SYMMETRIC_CIRCLES :

                circleGridSymmetric::getKnownPlanePositions( corners, pInfo );

                break;

            case PATTERN_TYPE_ASYMMETRIC_CIRCLES :

                circleGridAsymmetric::getKnownPlanePositions( corners, pInfo );

                break;

            default :
                
                cout << "pattern type: " << pInfo.type << " not found" << endl;
                success = false;
                
                break;
        }
        
        return success;
    }
  
    bool getPatternCorners( vector< cv::Point2f >& iCorners, const cv::Mat& image, const PatternInfo& pInfo )
    {
        
        bool success = true;
        
        switch ( pInfo.type )
        {
            case PATTERN_TYPE_CHESSBOARD :
                
                success = chessboard::getCorners( iCorners, image, pInfo );
                
                break;
                
            case PATTERN_TYPE_SYMMETRIC_CIRCLES :

                success = circleGridSymmetric::getCorners( iCorners, image, pInfo );

                break;

            case PATTERN_TYPE_ASYMMETRIC_CIRCLES :

                success = circleGridAsymmetric::getCorners( iCorners, image, pInfo );

                break;

            default :
                
                cout << "pattern type: " << pInfo.type << " not found" << endl;
                success = false;
                
                break;
        }
        
        return success;
    }

    void drawPatternCorners( const vector< cv::Point2f >& iCorners, cv::Mat& image, const PatternInfo& pInfo )
    {
        switch ( pInfo.type )
        {
            case PATTERN_TYPE_CHESSBOARD :
            case PATTERN_TYPE_SYMMETRIC_CIRCLES :
            case PATTERN_TYPE_ASYMMETRIC_CIRCLES :

                chessboard::drawPatternCorners( iCorners, image, pInfo );

            break;
        }
    }





}