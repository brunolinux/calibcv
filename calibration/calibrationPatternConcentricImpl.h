
#pragma once

#include "calibrationCommon.h"
#include "calibrationPatternConcentricUtils.h"

using namespace std;

namespace calibcv
{


	void drawConcentricPatternCorners( const vector< cv::Point2f >& iCorners, cv::Mat& image, const PatternInfo& pInfo );
	bool findConcentricGrid( const cv::Mat& image, const cv::Size pSize, vector< cv::Point2f >& iCorners );


	namespace detection
	{

	    struct TrackingPoint
	    {
	        cv::Point2f pos;
	        cv::Point2f vel;
	        bool found;
	    };

	    enum
	    {
	    	STAGE_THRESHOLDING = 0,
	    	STAGE_EDGE_DETECTION = 1,
	    	STAGE_FEATURES_EXTRACTION = 2,
	    	STAGE_KEYPOINTS_TRACKING = 3
	    };

	    enum
	    {
	    	MODE_FINDING_PATTERN = 0,
	    	MODE_TRACKING = 1,
	    	MODE_RECOVERING = 2
	    };

		class Detector
		{

			private :

			int m_mode;

			cv::Size m_size;
			int m_numPoints;

			cv::Rect2i m_initialROI;

			vector< cv::Point2f > m_candidatePoints;
			vector< cv::Point2f > m_matchedPoints;
			vector< TrackingPoint > m_trackingPoints;
			cv::Point2f m_patternCenter;
			cv::Point2f m_patternOrientation;

			vector< cv::Mat > m_stageFrameResults;

			CpuTimer m_timer;
			float m_stagesTimeCosts[4];

			cv::Ptr< cv::SimpleBlobDetector > m_blobsDetector;

			void _runTracking( const cv::Mat& input, cv::Mat& output );
			void _runFeaturesExtractor( const cv::Mat& input, cv::Mat& output );
			void _runEdgesGenerator( const cv::Mat& input, cv::Mat& output );
			void _runMaskGenerator( const cv::Mat& input, cv::Mat& output );

			void _computeInitialPattern( vector< cv::Point2f >& candidatePoints );
			void _pipeline( const cv::Mat& input );

			bool runInitialDetectionMode( const cv::Mat& input );
			bool runTrackingMode( const cv::Mat& input );
			bool runRecoveringMode( const cv::Mat& input );

			public :

			Detector( const cv::Size& size );
			~Detector();

			void run( const cv::Mat& input );

			void setInitialROI( const cv::Rect2i& roi ) { m_initialROI = roi; }

			bool getDetectedPoints( vector< cv::Point2f >& iPoints );
			void getTimeCosts( vector< float >& timeCosts );
			void getStageFrameResults( vector< cv::Mat >& vStageResults );
		};

	}



}