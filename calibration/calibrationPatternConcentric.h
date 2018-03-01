


#include "calibrationCommon.h"

#include "calibrationPatternConcentricImpl.h"

namespace calibration { namespace concentric {


	bool getCorners( vector< cv::Point2f >& iCorners, const cv::Mat& image, const PatternInfo& pInfo )
	{
		return calibcv::findConcentricGrid( image, pInfo.cb_size, iCorners );
	}

    void getKnownPlanePositions( vector< cv::Point3f >& kCorners, const PatternInfo& pInfo )
    {
        for ( int y = 0; y < pInfo.cb_size.height; y++ )
        {
            for ( int x = 0; x < pInfo.cb_size.width; x++ )
            {
                kCorners.push_back( cv::Point3f( y * pInfo.cb_squareLength, 
                                                 x * pInfo.cb_squareLength, 
                                                 0.0f ) );
            }
        }
    }


    void drawPatternCorners( const vector< cv::Point2f >& iCorners, cv::Mat& image, const PatternInfo& pInfo )
    {
    	calibcv::drawConcentricPatternCorners( iCorners, image, pInfo );
    }





}}