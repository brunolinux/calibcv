
#include "calibrationPatternConcentricUtils.h"

namespace calibcv { namespace detection { namespace utils {


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

		constructGridPattern( _p0, _p1, _p2, _p3, _patternPoints );

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
				cv::Point2f _pBottom = p3 - _ee * _p3p2;
				cv::Point2f _pLeft 	 = p0 - _nn * _p3p0;
				cv::Point2f _pRight  = p1 + _nn * _p1p2;

				cv::Point2f _point = ( 1.0f - _ee ) * _pLeft + ( _ee ) * _pRight +
									 ( 1.0f - _nn ) * _pBottom + ( _nn ) * _pTop -
									 ( 1.0f - _nn ) * ( 1.0f - _ee ) * _p3 - 
									 ( 1.0f - _ee ) * ( _nn ) * _p0 -
									 ( 1.0f - _nn ) * ( _ee ) * _p2 -
									 ( _nn ) * ( _ee ) * _p1;

				vGridPattern.push_back( _point )
			}
		}
	}

	float dist( const cv::Point2f& p1, const cv::Point2f& p2 )
	{
		return cv::norm( p1 - p2 );
	}

}}}