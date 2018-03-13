
#include "calibrationPatternConcentricUtils.h"

namespace utils {


	// Pattern should be like :
	/*
	*       sizewidth
	*   0 -------------- 1
	*    |       		|
	*    |       		| sizeheight
	*    |       		|
	*   3 -------------- 2
	*/
	bool isGridPatternFit( const vector< cv::Point2f >& candidates,
						   vector< cv::Point2f >& matchedPattern,
						   const cv::Size boardSize,
						   int indxp0, int indxp1,
						   int indxp2, int indxp3 )
	{
		bool _isFit = false;

		cv::Point2f _p0 = candidates[ indxp0 ];
		cv::Point2f _p1 = candidates[ indxp1 ];
		cv::Point2f _p2 = candidates[ indxp2 ];
		cv::Point2f _p3 = candidates[ indxp3 ];

		vector< cv::Point2f > _patternPoints;

		constructGridPattern( _p0, _p1, _p2, _p3, boardSize, _patternPoints );

		vector< bool > _found( candidates.size(), false );

		// Naive check
		for ( int q = 0; q < _patternPoints.size(); q++ )
		{
			cv::Point2f _closest;
			float _closestDist = cv::norm( _patternPoints[q] - _closest );
			int _closestIndx = -1;

			for ( int p = 0; p < candidates.size(); p++ )
			{
				if ( _found[p] )
				{
					continue;
				}

				if ( _closestIndx == -1 )
				{
					_closest = candidates[p];
					_closestDist = cv::norm( _patternPoints[q] - candidates[p] );
					_closestIndx = p;
				}
				else
				{
					float _dist = cv::norm( _patternPoints[q] - candidates[p] );
					if ( _dist < _closestDist )
					{
						_closestDist = _dist;
						_closest = candidates[p];
						_closestIndx = p;
					}
				}
			}

			_found[_closestIndx] = true;

			if ( _closestDist < RANGE_CLOSE )
			{
				matchedPattern.push_back( _closest );
			}
		}

		if ( matchedPattern.size() == boardSize.width * boardSize.height )
		{
			_isFit = true;
		}

		return _isFit;
	}

	void constructGridPattern( const cv::Point2f& p0,
						 	   const cv::Point2f& p1,
						 	   const cv::Point2f& p2,
						 	   const cv::Point2f& p3,
						 	   const cv::Size boardSize,
						 	   vector< cv::Point2f >& vGridPattern )
	{
		int se = boardSize.width;
		int sn = boardSize.height;

		cv::Point2f _p0p1 = p1 - p0;
		cv::Point2f _p1p2 = p2 - p1;
		cv::Point2f _p2p3 = p3 - p2;
		cv::Point2f _p3p0 = p0 - p3;

		for ( int n = 0; n < sn; n++ )
		{
			for ( int e = 0; e < se; e++ )
			{
				float _nn = ( 1.0f / ( sn - 1 ) ) * n;
				float _ee = ( 1.0f / ( se - 1 ) ) * e;

				cv::Point2f _pTop 	 = p0 + _ee * _p0p1;
				cv::Point2f _pBottom = p3 - _ee * _p2p3;
				cv::Point2f _pLeft 	 = p0 - _nn * _p3p0;
				cv::Point2f _pRight  = p1 + _nn * _p1p2;

				cv::Point2f _point = ( 1.0f - _ee ) * _pLeft + ( _ee ) * _pRight +
									 ( 1.0f - _nn ) * _pBottom + ( _nn ) * _pTop -
									 ( 1.0f - _nn ) * ( 1.0f - _ee ) * p3 - 
									 ( 1.0f - _ee ) * ( _nn ) * p0 -
									 ( 1.0f - _nn ) * ( _ee ) * p2 -
									 ( _nn ) * ( _ee ) * p1;

				vGridPattern.push_back( _point );
			}
		}
	}

	float dist( const cv::Point2f& p1, const cv::Point2f& p2 )
	{
		return cv::norm( p1 - p2 );
	}

	float checkEnd2EndColinearity( const vector< cv::Point2f >& points )
	{
		// Get line parameters - compute equation ax + by + c = 0
		/* Given pStart and pEnd, we have
		*   a = ( yEnd - yStart )
		*   b = ( xStart - xEnd )
		*   c = ( xEnd * yStart - xStart * yEnd )
		*   dist( p, Line ) = | a * px + b * py + c | / sqrt( a^2 + b^2 )
		*/
		float _xStart = points[0].x;
		float _yStart = points[0].y;

		float _xEnd = points[ points.size() - 1 ].x;
		float _yEnd = points[ points.size() - 1 ].y;

		float _a = _yEnd - _yStart;
		float _b = _xStart - _xEnd;
		float _c = _xEnd * _yStart - _xStart * _yEnd;

		float _divider = sqrt( _a * _a + _b * _b );

		float _distCum = 0.0f;

		for ( int q = 1; q < points.size() - 1; q++ )
		{
			_distCum += abs( _a * points[q].x + _b * points[q].y + _c ) / _divider;
		}

		return _distCum / ( points.size() - 2 );
	}

    void computeReprojectionErrors( const cv::Mat& cameraMatrix, const cv::Mat& distortionCoefficients,
                                    const vector< vector< cv::Point3f > >& worldPoints,
                                    const vector< vector< cv::Point2f > >& imagePoints,
                                    const vector< cv::Mat >& rvecs, const vector< cv::Mat >& tvecs,
                                    float& reprojectionError,
                                    vector<float>& perViewErrors )
    {
        vector< cv::Point2f > _reprojectedPoints;
        size_t _totalPoints = 0;
            
        float _totalErr = 0;
        float _err = 0;
            
        perViewErrors.resize( worldPoints.size() );

        for( size_t i = 0; i < worldPoints.size(); ++i )
        {
            cv::projectPoints( worldPoints[i], rvecs[i], tvecs[i], cameraMatrix, distortionCoefficients, _reprojectedPoints );
            _err = cv::norm( imagePoints[i], _reprojectedPoints, cv::NORM_L2 );

            size_t _n           = worldPoints[i].size();
            perViewErrors[i]    = ( float ) std::sqrt( _err * _err / _n );
            _totalErr           += _err * _err;
            _totalPoints        += _n;
        }

        reprojectionError = std::sqrt( _totalErr / _totalPoints );
    }

}