
#pragma once

#include "calibrationCommon.h"

using namespace std;

#define RANGE_CLOSE 5

namespace utils {


	struct ComparatorKeyPointsCentric
	{

	   	private :

	    float m_x;
	    float m_y;

	    public :

	    bool operator()( const cv::KeyPoint& a, const cv::KeyPoint& b )
	    {
		    float aCmp = abs( a.pt.x - m_x ) + abs( a.pt.y - m_y );
		    float bCmp = abs( b.pt.x - m_x ) + abs( b.pt.y - m_y );

		    return aCmp > bCmp;
	    }

	    void setCenter( float x, float y );
	};

	struct ComparatorKeyPointsY
	{
	   	public :

	   	bool operator()( const cv::KeyPoint& a, const cv::KeyPoint& b )
	    {
	    	return a.pt.y > b.pt.y;
	    }
	};

	struct ComparatorKeyPointsX
	{
	    public :

	    bool operator()( const cv::KeyPoint& a, const cv::KeyPoint& b )
	    {
	    	return a.pt.x < b.pt.x;
	    }
	};


	bool isGridPatternFit( const vector< cv::Point2f >& candidates,
						   vector< cv::Point2f >& matchedPattern,
						   const cv::Size boardSize,
						   int indxp0, int indxp1,
						   int indxp2, int indxp3 );

	void constructGridPattern( const cv::Point2f& p0,
						 	   const cv::Point2f& p1,
						 	   const cv::Point2f& p2,
						 	   const cv::Point2f& p3,
						 	   const cv::Size boardSize,
						 	   vector< cv::Point2f >& vGridPattern );

	float dist( const cv::Point2f& p1, const cv::Point2f& p2 );

	float checkEnd2EndColinearity( const vector< cv::Point2f >& points );

    void computeReprojectionErrors( const cv::Mat& cameraMatrix, const cv::Mat& distortionCoefficients,
                                    const vector< vector< cv::Point3f > >& worldPoints,
                                    const vector< vector< cv::Point2f > >& imagePoints,
                                    const vector< cv::Mat >& rvecs, const vector< cv::Mat >& tvecs,
                                    float& reprojectionError,
                                    vector<float>& perViewErrors );

    void computeColinearityErrors( const cv::Mat& cameraMatrix, const cv::Mat& distortionCoefficients,
    							   const vector< vector< cv::Point3f > >& worldPoints,
                                   const vector< vector< cv::Point2f > >& imagePoints,
                                   const vector< cv::Mat >& rvecs, const vector< cv::Mat >& tvecs,
                                   float& oldColinearity, float& newColinearity );

   	void distortPoints( const vector< cv::Point2f >& undistorted,
   						vector< cv::Point2f >& distorted,
   						const cv::Mat& cameraMatrix, const cv::Mat& distortionCoefficients )
   	{
   		// just in case
   		distorted.clear();

   		double _fx = cameraMatrix.at< double >( 0 );
   		double _fy = cameraMatrix.at< double >( 4 );
   		double _cx = cameraMatrix.at< double >( 2 );
   		double _cy = cameraMatrix.at< double >( 5 );

   		double _k1 = distortionCoefficients.at< double >( 0 );
   		double _k2 = distortionCoefficients.at< double >( 1 );
   		double _p1 = distortionCoefficients.at< double >( 2 );
   		double _p2 = distortionCoefficients.at< double >( 3 );
   		double _k3 = distortionCoefficients.at< double >( 4 );

   		for ( int q = 0; q < undistorted.size(); q++ )
   		{
   			double _x = ( undistorted[q].x - _cx ) / _fx;
   			double _y = ( undistorted[q].y - _cy ) / _fy;

   			double _r2 = _x * _x + _y * _y;
   			double _r4 = _r2 * _r2;
   			double _r6 = _r2 * _r4;

   			double _xy = _x * _y;

   			// Apply radial distortion
   			double _xDistort = _x * ( 1 + _k1 * _r2 + _k2 * _r4 + _k3 * _r6 );
   			double _yDistort = _y * ( 1 + _k1 * _r2 + _k2 * _r4 + _k3 * _r6 );

   			_xDistort += ( 2 * _p1 * _xy + _p2 * ( _r2 + 2 * _x * _x )  );
   			_yDistort += ( 2 * _p2 * _xy + _p1 * ( _r2 + 2 * _y * _y )  );

   			_xDistort = _xDistort * _fx + _cx;
   			_yDistort = _yDistort * _fy + _cy;

   			distorted.push_back( cv::Point2f( (float) _xDistort, (float) _yDistort ) );
   		}
   	}

}