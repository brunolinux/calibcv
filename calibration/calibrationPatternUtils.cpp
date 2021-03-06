
#include "calibrationPatternUtils.h"

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

            if ( _closestIndx == -1 )
            {
                return false;
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

    void computeColinearityErrors( const cv::Mat& cameraMatrix, const cv::Mat& distortionCoefficients,
    							   const vector< vector< cv::Point3f > >& worldPoints,
                                   const vector< vector< cv::Point2f > >& imagePoints,
                                   const vector< cv::Mat >& rvecs, const vector< cv::Mat >& tvecs,
                                   const cv::Size& boardSize,
                                   float& oldColinearity, float& newColinearity )
    {
        vector< cv::Point2f > _reprojectedPoints;
        size_t _totalPoints = 0;

        float _totalErrOld = 0;
        float _totalErrNew = 0;
        float _errOriginal = 0;
        float _errReprojected = 0;

        for( size_t i = 0; i < worldPoints.size(); ++i )
        {
            cv::projectPoints( worldPoints[i], rvecs[i], tvecs[i], cameraMatrix, distortionCoefficients, _reprojectedPoints );

            for( size_t j = 0; j < boardSize.height; j++ )
            {
                vector< cv::Point2f > _original;
                vector< cv::Point2f > _reprojected;
                for( size_t k = 0; k < boardSize.width; k++ )
                {
                    int _ptIndex = j * boardSize.height + k;
                    _original.push_back( imagePoints[i][_ptIndex] );
                    _reprojected.push_back( _reprojectedPoints[_ptIndex] );
                }
                _errOriginal += utils::checkEnd2EndColinearity( _original ) / boardSize.width;
                _errReprojected += utils::checkEnd2EndColinearity( _reprojected ) / boardSize.width;
            }
            _errOriginal /= boardSize.height;
            _errReprojected /= boardSize.height;
            _totalErrOld           += _errOriginal * _errOriginal;
            _totalErrNew           += _errReprojected * _errReprojected;

            size_t _n           = worldPoints[i].size();
            _totalPoints        += _n;
        }

        oldColinearity = std::sqrt( _totalErrOld / _totalPoints );
        newColinearity = std::sqrt( _totalErrNew / _totalPoints );
	}

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

     bool getLinesIntersection( const cv::Point2f ap0, const cv::Point2f ap1,
                                const cv::Point2f bp0, const cv::Point2f bp1,
                                cv::Point2f& intersection )
    {
        if ( cv::norm( ap1 - ap0 ) == 0.0f ||
             cv::norm( bp1 - bp0 ) == 0.0f )
        {
            return false;
        }

        cv::Point2f _ua = ( ap1 - ap0 ) / cv::norm( ap1 - ap0 );
        cv::Point2f _ub = ( bp1 - bp0 ) / cv::norm( bp1 - bp0 );

        float _dsys = -_ub.x * _ua.y + _ub.y * _ua.x;
        float _dq = -( ap0.x - bp0.x ) * _ua.y + ( ap0.y - bp0.y ) * _ua.x;
        float _dt = ( ap0.y - bp0.y ) * _ub.x - ( ap0.x - bp0.x ) * _ub.y;

        if ( _dsys == 0.0f )
        {
            return false;
        }

        float _t = _dt / _dsys;
        float _q = _dq / _dsys;

        intersection = ap0 + _ua * _t;

        return true;
    }

}
