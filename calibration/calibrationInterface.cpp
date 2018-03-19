

#include "calibrationInterface.h"

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
                                const vector< vector< cv::Point2f > >& batchPointsToRefine,
                                const cv::Mat& cameraMatrix,
                                const cv::Mat& distortionCoefficients )
    {

        switch ( pInfo.type )
        {
            case PATTERN_TYPE_CHESSBOARD :
                
                chessboard::refineBatch( pInfo.size,
                                         batchImagesToRefine, 
                                         batchPointsToRefine, 
                                         cameraMatrix, 
                                         distortionCoefficients );
                
                break;
                
            case PATTERN_TYPE_SYMMETRIC_CIRCLES :

                circleGridSymmetric::refineBatch( pInfo.size,
                                                  batchImagesToRefine, 
                                                  batchPointsToRefine, 
                                                  cameraMatrix, 
                                                  distortionCoefficients );

                break;

            case PATTERN_TYPE_ASYMMETRIC_CIRCLES :

                circleGridAsymmetric::refineBatch( pInfo.size,
                                                   batchImagesToRefine, 
                                                   batchPointsToRefine, 
                                                   cameraMatrix, 
                                                   distortionCoefficients );

                break;

            case PATTERN_TYPE_CONCENTRIC_CIRCLES :

                concentric::refineBatch( pInfo.size,
                                         batchImagesToRefine, 
                                         batchPointsToRefine, 
                                         cameraMatrix, 
                                         distortionCoefficients );

                break;

            default :
                
                cout << "pattern type: " << pInfo.type << " not found" << endl;
                
                break;
        }


    }

    bool requestSingleRefinment( const PatternInfo& pInfo,
                                 const cv::Mat& imageToRefine,
                                 const vector< cv::Point2f >& pointsToRefine,
                                 const cv::Mat& cameraMatrix,
                                 const cv::Mat& distortionCoefficients,
                                 cv::Mat& imageResult,
                                 vector< cv::Point2f >& pointsRefined )
    {
        bool _couldRefine = false;

        switch ( pInfo.type )
        {
            case PATTERN_TYPE_CHESSBOARD :
                
                _couldRefine = chessboard::refineSingle( pInfo.size,
                                          imageToRefine, 
                                          pointsToRefine, 
                                          cameraMatrix, 
                                          distortionCoefficients,
                                          imageResult,
                                          pointsRefined );
                
                break;
                
            case PATTERN_TYPE_SYMMETRIC_CIRCLES :

                _couldRefine = circleGridSymmetric::refineSingle( pInfo.size,
                                                                  imageToRefine, 
                                                                  pointsToRefine, 
                                                                  cameraMatrix, 
                                                                  distortionCoefficients,
                                                                  imageResult,
                                                                  pointsRefined );

                break;

            case PATTERN_TYPE_ASYMMETRIC_CIRCLES :

                _couldRefine = circleGridAsymmetric::refineSingle( pInfo.size,
                                                                   imageToRefine, 
                                                                   pointsToRefine, 
                                                                   cameraMatrix, 
                                                                   distortionCoefficients,
                                                                   imageResult,
                                                                   pointsRefined );

                break;

            case PATTERN_TYPE_CONCENTRIC_CIRCLES :

                _couldRefine = concentric::refineSingle( pInfo.size,
                                                         imageToRefine, 
                                                         pointsToRefine, 
                                                         cameraMatrix, 
                                                         distortionCoefficients,
                                                         imageResult,
                                                         pointsRefined );

                break;

            default :
                
                cout << "pattern type: " << pInfo.type << " not found" << endl;
                
                break;
        }

        return _couldRefine;
    }

    bool isRefining( const PatternInfo& pInfo )
    {
        switch ( pInfo.type )
        {
            case PATTERN_TYPE_CHESSBOARD :
                
                return chessboard::isRefining( pInfo.size );
                
            case PATTERN_TYPE_SYMMETRIC_CIRCLES :

                return circleGridSymmetric::isRefining( pInfo.size );

            case PATTERN_TYPE_ASYMMETRIC_CIRCLES :

                return circleGridAsymmetric::isRefining( pInfo.size );

            case PATTERN_TYPE_CONCENTRIC_CIRCLES :

                return concentric::isRefining( pInfo.size );

            default :
                
                cout << "pattern type: " << pInfo.type << " not found" << endl;

                break;
        }

        return false;
    }

    bool hasRefinationToPick( const PatternInfo& pInfo )
    {

        switch ( pInfo.type )
        {
            case PATTERN_TYPE_CHESSBOARD :
                
                return chessboard::hasRefinationToPick( pInfo.size );
                
            case PATTERN_TYPE_SYMMETRIC_CIRCLES :

                return circleGridSymmetric::hasRefinationToPick( pInfo.size );

            case PATTERN_TYPE_ASYMMETRIC_CIRCLES :

                return circleGridAsymmetric::hasRefinationToPick( pInfo.size );

            case PATTERN_TYPE_CONCENTRIC_CIRCLES :

                return concentric::hasRefinationToPick( pInfo.size );

            default :
                
                cout << "pattern type: " << pInfo.type << " not found" << endl;

                break;
        }

        return false;
    }

    void grabRefinationBatch( const PatternInfo& pInfo, 
                              vector< cv::Mat >& batchRefinedImages,
                              vector< vector< cv::Point2f > >& batchRefinedPoints )
    {
   
        switch ( pInfo.type )
        {
            case PATTERN_TYPE_CHESSBOARD :
                
                return chessboard::grabRefinationBatch( pInfo.size, batchRefinedImages, batchRefinedPoints );
                
            case PATTERN_TYPE_SYMMETRIC_CIRCLES :

                return circleGridSymmetric::grabRefinationBatch( pInfo.size, batchRefinedImages, batchRefinedPoints );

            case PATTERN_TYPE_ASYMMETRIC_CIRCLES :

                return circleGridAsymmetric::grabRefinationBatch( pInfo.size, batchRefinedImages, batchRefinedPoints );

            case PATTERN_TYPE_CONCENTRIC_CIRCLES :

                return concentric::grabRefinationBatch( pInfo.size, batchRefinedImages, batchRefinedPoints );

            default :
                
                cout << "pattern type: " << pInfo.type << " not found" << endl;

                break;
        }
    }

    void update( const PatternInfo& pInfo )
    {
        switch ( pInfo.type )
        {
            case PATTERN_TYPE_CHESSBOARD :
                
                return chessboard::update( pInfo.size );
                
            case PATTERN_TYPE_SYMMETRIC_CIRCLES :

                return circleGridSymmetric::update( pInfo.size );

            case PATTERN_TYPE_ASYMMETRIC_CIRCLES :

                return circleGridAsymmetric::update( pInfo.size );

            case PATTERN_TYPE_CONCENTRIC_CIRCLES :

                return concentric::update( pInfo.size );

            default :
                
                cout << "pattern type: " << pInfo.type << " not found" << endl;

                break;
        }
    }
}