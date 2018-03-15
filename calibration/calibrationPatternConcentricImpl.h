
#pragma once

#include "calibrationCommon.h"
#include "calibrationPatternConcentricUtils.h"

#include <panels/SPatternDetectorPanel.h>

using namespace std;

#define PIPELINE_MASKING_STAGE_MAX_VALUE 255
#define PIPELINE_MASKING_STAGE_BLOCKSIZE 21
#define PIPELINE_MASKING_STAGE_C         15

#define PIPELINE_ELLIPSE_MIN_SIZE 15
#define PIPELINE_ELLIPSE_MAX_SIZE 52
#define PIPELINE_ELLIPSE_MIN_RATIO 0.6
#define PIPELINE_ELLIPSE_MAX_RATIO 1.3

#define PIPELINE_EDGES_STAGE_SCALE 1
#define PIPELINE_EDGES_STAGE_DELTA 0

#define ROI_MARGIN 80

namespace calibration { namespace concentric {


	void drawConcentricPatternCorners( const vector< cv::Point2f >& iCorners, 
									   cv::Mat& image, const PatternInfo& pInfo );

	bool findConcentricGrid( const cv::Mat& image, const cv::Size pSize, 
                             const DetectionInfo& detInfo,
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

			STAGE_REFINING_UNDISTORTED = 4,
			STAGE_REFINING_FRONTO = 5,
			STAGE_REFINING_MASK = 6,
			STAGE_REFINING_EDGES = 7,
			STAGE_REFINING_FEATURES = 8,
			STAGE_REFINING_PROJECTED = 9,
			STAGE_REFINING_DISTORTED = 10,

			PIPELINE_MAX_STAGES = 11
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
			vector< cv::Point2f > m_matchedPoints;
			vector< TrackingPoint > m_trackingPoints;
			vector< cv::Point2f > m_refinedPoints;
			cv::Point2f m_patternCenter;
			cv::Point2f m_patternOrientation;

			cv::Mat m_frame;
			cv::Size m_frameSize;
			cv::Mat m_workingInput;
			vector< cv::Mat > m_stageFrameResults;

			CpuTimer m_timer;
			float m_stagesTimeCosts[4];

			bool m_hasRefinedPoints;

			cv::Ptr< cv::SimpleBlobDetector > m_blobsDetector;

			void _runTracking( const cv::Mat& input, cv::Mat& output );
			void _runFeaturesExtractor( const cv::Mat& input, cv::Mat& output );
			void _runEdgesGenerator( const cv::Mat& input, cv::Mat& output );
			void _runMaskGenerator( const cv::Mat& input, cv::Mat& output );

			bool _computeInitialPattern( const vector< cv::Point2f >& candidatePatternPoints,
                                         vector< cv::Point2f >& matchedPoints, bool isFronto = false );
			void _pipeline( const cv::Mat& input );

			// Refining steps **********************************************************************

			bool _refining( const cv::Mat& input, 
							const cv::Mat& cameraMatrix, const cv::Mat& distortionCoefficients, 
							const vector< TrackingPoint >& patternPoints,
							vector< cv::Point2f >& refinedPoints );

        	void _refiningUndistortion( const cv::Mat& input, cv::Mat& output,
                                        const cv::Mat& cameraMatrix, const cv::Mat& distortionCoefficients,
                                        const vector< cv::Point2f >& patternPoints,
                                        vector< cv::Point2f >& undistortedPatternPoints );

        	void _refiningFronto( const cv::Mat& input,// undistorted image
                                  cv::Mat& output,// result after transforming to fronto view
                                  const vector< cv::Point2f >& undistortedPatternPoints,
                                  cv::Mat& frontoTransform );

        	void _refiningMask( const cv::Mat& input,
                                cv::Mat& output );

        	void _refiningEdges( const cv::Mat& input,
                                 cv::Mat& output );

        	bool _refiningFeatures( const cv::Mat& input,
                                    const cv::Mat& frontoView,
                                    cv::Mat& output,
                                    vector< cv::Point2f >& patternPointsFronto );

        	void _refiningProjected( const cv::Mat& undistortedView,
                                     cv::Mat& output,
                                     const cv::Mat& inverseFrontoTransform,
                                     const vector< cv::Point2f >& patternPointsFronto,
                                     vector< cv::Point2f >& patternPointsProjected,
                                     const vector< cv::Point2f >& patternPointsUndistorted );

        	void _refiningDistortion( const cv::Mat& originalView,
                                      cv::Mat& output,
                                      const cv::Mat& cameraMatrix, const cv::Mat& distortionCoefficients,
                                      const vector< cv::Point2f >& refinedUndistortedProjectedPoints,
                                      vector< cv::Point2f >& refinedDistortedProjectedPoints,
                                      const vector< cv::Point2f >& originalPatternPoints );

        	// *************************************************************************************

			bool runInitialDetectionMode( const cv::Mat& input );
			bool runTrackingMode( const cv::Mat& input, const DetectionInfo& detInfo );
			bool runRecoveringMode( const cv::Mat& input );

			Detector( const cv::Size& size );

			public :

			static Detector* INSTANCE;
			static Detector* create( const cv::Size& size );
			static void release();

			~Detector();

			void setPatternSize( const cv::Size& size );

			bool run( const cv::Mat& input, const DetectionInfo& detInfo );

			void setInitialROI( const vector< cv::Point2f >& roi ) { m_initialROI = roi; }

			void getDetectedPoints( vector< cv::Point2f >& iPoints );
			void getRefinedPoints( vector< cv::Point2f >& iPoints );

			bool hasRefinedPoints() { return m_hasRefinedPoints; }

			void getTimeCosts( vector< float >& timeCosts );
			void getStageFrameResults( vector< cv::Mat >& vStageResults );

			string getCurrentDetectionMode();
		};
	}

}}