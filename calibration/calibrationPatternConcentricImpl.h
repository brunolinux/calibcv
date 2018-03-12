
#pragma once

#include "calibrationCommon.h"
#include "calibrationPatternConcentricUtils.h"

#include <panels/SPatternDetectorPanel.h>

using namespace std;

#define PIPELINE_MASKING_STAGE_MAX_VALUE 255
#define PIPELINE_MASKING_STAGE_BLOCKSIZE 21
#define PIPELINE_MASKING_STAGE_C         15

#define PIPELINE_EDGES_STAGE_SCALE 1
#define PIPELINE_EDGES_STAGE_DELTA 0

#define ELLIPSE_MIN_SIZE 60
#define ELLIPSE_MAX_SIZE 95
#define ELLIPSE_MIN_RATIO 0.8
#define ELLIPSE_MAX_RATIO 1.2

#define ROI_MARGIN 80

namespace calibration { namespace concentric {


	void drawConcentricPatternCorners( const vector< cv::Point2f >& iCorners,
									   cv::Mat& image, const PatternInfo& pInfo );

	bool findConcentricGrid( const cv::Mat& image, const cv::Size pSize,
                             const vector< cv::Point2f >& roi,
                             vector< cv::Point2f >& iCorners );


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
	    	STAGE_KEYPOINTS_TRACKING = 3,
			STAGE_KEYPOINTS_TRANSFORMATION = 4
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

			calibcv::SPatternDetectorPanel* m_pipelinePanel;

			int m_mode;

			cv::Size m_size;
			int m_numPoints;

			vector< cv::Point2f > m_initialROI;

			cv::Point2f m_cropOrigin;
			cv::Rect2f m_cropROI;

			vector< cv::Point2f > m_candidatePoints;
			vector< cv::Point2f > m_perspectivePoints;
			vector< cv::Point2f > m_matchedPoints;
			vector< TrackingPoint > m_trackingPoints;
			cv::Point2f m_patternCenter;
			cv::Point2f m_patternOrientation;

			cv::Mat m_frame;
			cv::Mat m_workingInput;
			vector< cv::Mat > m_stageFrameResults;

			CpuTimer m_timer;
			float m_stagesTimeCosts[4];

			cv::Ptr< cv::SimpleBlobDetector > m_blobsDetector;

			void _runTracking( const cv::Mat& input, cv::Mat& output );
			void _runFeaturesExtractor( const cv::Mat& input, cv::Mat& output );
			void _runEdgesGenerator( const cv::Mat& input, cv::Mat& output );
			void _runMaskGenerator( const cv::Mat& input, cv::Mat& output );

			void _runFeaturesExtractor( const cv::Mat& input, vector< cv::Point2f >& m_candidatePoints );

			bool _computeInitialPattern( const vector< cv::Point2f >& candidatePatternPoints,
                                         vector< cv::Point2f >& matchedPoints );
			void _pipeline( const cv::Mat& input );

			bool runInitialDetectionMode( const cv::Mat& input );
			bool runTrackingMode( const cv::Mat& input );
			bool runRecoveringMode( const cv::Mat& input );

			Detector( const cv::Size& size );

			public :

			static Detector* INSTANCE;
			static Detector* create( const cv::Size& size );
			static void release();

			~Detector();
			void _pipeline2( const cv::Mat& input, cv::Mat& fronto_parallel );

			void setPatternSize( const cv::Size& size );

			bool run( const cv::Mat& input, const vector< cv::Point2f >& roi );

			void setInitialROI( const vector< cv::Point2f >& roi ) { m_initialROI = roi; }

			void getDetectedPoints( vector< cv::Point2f >& iPoints );
			void getTimeCosts( vector< float >& timeCosts );
			void getStageFrameResults( vector< cv::Mat >& vStageResults );

			string getCurrentDetectionMode();
		};
	}

}}
