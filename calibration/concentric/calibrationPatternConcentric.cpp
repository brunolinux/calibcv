
#include "calibrationPatternConcentric.h"

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

    void refineBatch( const cv::Size& size,
                      const vector< cv::Mat >& batchImagesToRefine,
                      const vector< vector< cv::Point2f > >& batchPointsToRefine,
                      const cv::Mat& cameraMatrix,
                      const cv::Mat& distortionCoefficients )
    {
        refineBatchConcentric( size, batchImagesToRefine, batchPointsToRefine, cameraMatrix, distortionCoefficients );
    }

    bool refineSingle( const cv::Size& size,
                       const cv::Mat& imageToRefine,
                       const vector< cv::Point2f >& pointsToRefine,
                       const cv::Mat& cameraMatrix,
                       const cv::Mat& distortionCoefficients,
                       cv::Mat& imageResult,
                       vector< cv::Point2f >& pointsRefined )
    {
        return refineSingleConcentric( size, 
                                       imageToRefine, 
                                       pointsToRefine, 
                                       cameraMatrix, 
                                       distortionCoefficients,
                                       imageResult,
                                       pointsRefined );
    }

    bool isRefining( const cv::Size& size )
    {
        return isRefiningConcentric( size );
    }

    bool hasRefinationToPick( const cv::Size& size )
    {
        return hasRefinationToPickConcentric( size );
    }

    void grabRefinationBatch( const cv::Size& size,
                              vector< cv::Mat >& batchRefinedImages,
                              vector< vector< cv::Point2f > >& batchRefinedPoints )
    {
        grabRefinationBatchConcentric( size, batchRefinedImages, batchRefinedPoints );
    }

    void update( const cv::Size& size )
    {
        updateConcentric( size );
    }

}}