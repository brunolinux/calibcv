
#pragma once

#include "../calibrationCommon.h"
#include "../calibrationBaseDetector.h"
#include "calibrationPatternUtils.h"

using namespace std;

#define PIPELINE_MASKING_STAGE_MAX_VALUE 255
#define PIPELINE_MASKING_STAGE_BLOCKSIZE 21
#define PIPELINE_MASKING_STAGE_C         15

#define PIPELINE_EDGES_STAGE_SCALE 1
#define PIPELINE_EDGES_STAGE_DELTA 0

#define PIPELINE_REFINING_MASKING_STAGE_MAX_VALUE 255
#define PIPELINE_REFINING_MASKING_STAGE_BLOCKSIZE 61
#define PIPELINE_REFINING_MASKING_STAGE_C         26

#define PIPELINE_REFINING_CANNY_MIN 50
#define PIPELINE_REFINING_CANNY_MAX 150
#define PIPELINE_REFINING_EDGES_BLOCK_SIZE 5

#define PIPELINE_REFINING_ELLIPSE_MIN_SIZE 25
#define PIPELINE_REFINING_ELLIPSE_MAX_SIZE 55
#define PIPELINE_REFINING_ELLIPSE_MIN_RATIO 0.8
#define PIPELINE_REFINING_ELLIPSE_MAX_RATIO 1.2

#define ROI_MARGIN 80

namespace calibration { namespace concentric {


	void drawConcentricPatternCorners( const vector< cv::Point2f >& iCorners, 
									   cv::Mat& image, const PatternInfo& pInfo );

	bool findConcentricGrid( const cv::Mat& image, const cv::Size& pSize, 
                             const DetectionInfo& detInfo,
                             vector< cv::Point2f >& iCorners );


    void refineBatchConcentric( const vector< cv::Mat >& batchImagesToRefine,
                      const cv::Mat& cameraMatrix,
                      const cv::Mat& distortionCoefficients );

    bool isRefiningConcentric();

    bool hasRefinationToPickConcentric();

    void grabRefinationBatchConcentric( vector< cv::Mat >& batchRefinedImages,
                              			vector< CalibrationBucket >& batchBuckets );

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

		class DetectorConcentric : public BaseDetector
		{

			private :

			int m_mode;

			vector< cv::Point2f > m_initialROI;

			vector< cv::Point2f > m_candidatePoints;
			vector< cv::Point2f > m_matchedPoints;
			vector< TrackingPoint > m_trackingPoints;

			cv::Mat m_workingInput;
			vector< cv::Mat > m_stageFrameResults;

			cv::Ptr< cv::SimpleBlobDetector > m_blobsDetector;

			void _runTracking( const cv::Mat& input, cv::Mat& output );
			void _runFeaturesExtractor( const cv::Mat& input, cv::Mat& output );
			void _runEdgesGenerator( const cv::Mat& input, cv::Mat& output );
			void _runMaskGenerator( const cv::Mat& input, cv::Mat& output );

			bool _computeInitialPattern( const vector< cv::Point2f >& candidatePatternPoints,
                                         vector< cv::Point2f >& matchedPoints, bool isFronto = false );
			void _pipeline( const cv::Mat& input );

			// Refining prakash steps **************************************************************

        	void _refiningMask( const cv::Mat& input,
                                cv::Mat& output );

        	void _refiningEdges( const cv::Mat& input,
                                 cv::Mat& output );

        	bool _refiningFeatures( const cv::Mat& input,
                                    const cv::Mat& frontoView,
                                    cv::Mat& output,
                                    vector< cv::Point2f >& patternPointsFronto );

        	// *************************************************************************************

			bool runInitialDetectionMode( const cv::Mat& input );
			bool runTrackingMode( const cv::Mat& input, const DetectionInfo& detInfo );
			bool runRecoveringMode( const cv::Mat& input );

            protected :

			DetectorConcentric( const cv::Size& size );

			public :

			static DetectorConcentric* INSTANCE;
			static DetectorConcentric* create( const cv::Size& size );
			static void release();

			~DetectorConcentric();

			bool run( const cv::Mat& input, const DetectionInfo& detInfo ) override;
			void getDetectedPoints( vector< cv::Point2f >& iPoints ) override;

			string getCurrentDetectionMode();
		};
	}

}}