
#pragma once

#include <sstream>
#include <cmath>
#include <ctime>

#include "calibrationCommon.h"
#include "calibrationPatternConcentricUtils.h"

#include "calibrationPatternChessboard.h"
#include "calibrationPatternCircleGridSymmetric.h"
#include "calibrationPatternCircleGridAsymmetric.h"
#include "calibrationPatternConcentric.h"

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

            case PATTERN_TYPE_CONCENTRIC_CIRCLES :

                concentric::getKnownPlanePositions( corners, pInfo );

                break;

            default :
                
                cout << "pattern type: " << pInfo.type << " not found" << endl;
                success = false;
                
                break;
        }
        
        return success;
    }
  
    bool getPatternCorners( vector< cv::Point2f >& iCorners, const cv::Mat& image, const PatternInfo& pInfo, const DetectionInfo& dInfo )
    {

        cv::Mat _imgGray;
        cv::cvtColor( image, _imgGray, CV_BGR2GRAY );
        
        bool success = true;
        
        switch ( pInfo.type )
        {
            case PATTERN_TYPE_CHESSBOARD :
                
                success = chessboard::getCorners( iCorners, _imgGray, pInfo, dInfo );

                // refine only if chessboard pattern
                if ( success )
                {
                    cv::cornerSubPix( _imgGray, iCorners, cv::Size( 11,11 ), cv::Size( -1,-1 ), 
                                      cv::TermCriteria( cv::TermCriteria::EPS + 
                                                        cv::TermCriteria::COUNT, 30, 0.1 ) );
                }

                break;
                
            case PATTERN_TYPE_SYMMETRIC_CIRCLES :

                success = circleGridSymmetric::getCorners( iCorners, _imgGray, pInfo, dInfo );

                break;

            case PATTERN_TYPE_ASYMMETRIC_CIRCLES :

                success = circleGridAsymmetric::getCorners( iCorners, _imgGray, pInfo, dInfo );

                break;

            case PATTERN_TYPE_CONCENTRIC_CIRCLES :

                success = concentric::getCorners( iCorners, image, pInfo, dInfo );

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

            case PATTERN_TYPE_CONCENTRIC_CIRCLES :

                concentric::drawPatternCorners( iCorners, image, pInfo );

            break;
        }
    }

    void requestBatchRefinment( const PatternInfo& pInfo,
                                const vector< cv::Mat >& batchImagesToRefine,
                                const cv::Mat& cameraMatrix,
                                const cv::Mat& distortionCoefficients )
    {
        // TODO: Should be something like this

        // switch ( pInfo.type )
        // {
        //     case PATTERN_TYPE_CHESSBOARD :
                
        //         chessboard::refineBatch( batchImagesToRefine, cameraMatrix, distortionCoefficients );
                
        //         break;
                
        //     case PATTERN_TYPE_SYMMETRIC_CIRCLES :

        //         circleGridSymmetric::refineBatch( batchImagesToRefine, cameraMatrix, distortionCoefficients );

        //         break;

        //     case PATTERN_TYPE_ASYMMETRIC_CIRCLES :

        //         circleGridAsymmetric::refineBatch( batchImagesToRefine, cameraMatrix, distortionCoefficients );

        //         break;

        //     case PATTERN_TYPE_CONCENTRIC_CIRCLES :

        //         concentric::refineBatch( batchImagesToRefine, cameraMatrix, distortionCoefficients );

        //         break;

        //     default :
                
        //         cout << "pattern type: " << pInfo.type << " not found" << endl;
                
        //         break;
        // }


    }

    bool isRefining( const PatternInfo& pInfo )
    {
        // TODO: Should be something like this

        // switch ( pInfo.type )
        // {
        //     case PATTERN_TYPE_CHESSBOARD :
                
        //         return chessboard::isRefining();
                
        //     case PATTERN_TYPE_SYMMETRIC_CIRCLES :

        //         return circleGridSymmetric::isRefining();

        //     case PATTERN_TYPE_ASYMMETRIC_CIRCLES :

        //         return circleGridAsymmetric::isRefining();

        //     case PATTERN_TYPE_CONCENTRIC_CIRCLES :

        //         return concentric::isRefining();

        //     default :
                
        //         cout << "pattern type: " << pInfo.type << " not found" << endl;

        //         break;
        // }

        return false;
    }

    bool hasRefinationToPick( const PatternInfo& pInfo )
    {
        // TODO: Should be something like this

        // switch ( pInfo.type )
        // {
        //     case PATTERN_TYPE_CHESSBOARD :
                
        //         return chessboard::hasRefinationToPick();
                
        //     case PATTERN_TYPE_SYMMETRIC_CIRCLES :

        //         return circleGridSymmetric::hasRefinationToPick();

        //     case PATTERN_TYPE_ASYMMETRIC_CIRCLES :

        //         return circleGridAsymmetric::hasRefinationToPick();

        //     case PATTERN_TYPE_CONCENTRIC_CIRCLES :

        //         return concentric::hasRefinationToPick();

        //     default :
                
        //         cout << "pattern type: " << pInfo.type << " not found" << endl;

        //         break;
        // }

        return false;
    }

    void grabRefinationBatch( const PatternInfo& pInfo, 
                              vector< cv::Mat >& batchRefinedImages,
                              vector< CalibrationBucket >& batchBuckets )
    {
        // TODO: Should be something like this
        
        // switch ( pInfo.type )
        // {
        //     case PATTERN_TYPE_CHESSBOARD :
                
        //         return chessboard::grabRefinationBatch( batchRefinedImages, batchBuckets );
                
        //     case PATTERN_TYPE_SYMMETRIC_CIRCLES :

        //         return circleGridSymmetric::grabRefinationBatch( batchRefinedImages, batchBuckets );

        //     case PATTERN_TYPE_ASYMMETRIC_CIRCLES :

        //         return circleGridAsymmetric::grabRefinationBatch( batchRefinedImages, batchBuckets );

        //     case PATTERN_TYPE_CONCENTRIC_CIRCLES :

        //         return concentric::grabRefinationBatch( batchRefinedImages, batchBuckets );

        //     default :
                
        //         cout << "pattern type: " << pInfo.type << " not found" << endl;

        //         break;
        // }
    }
}