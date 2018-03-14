
#pragma once

#include "../../Common.h"

#define TEXT_INIT 40
#define TEXT_COSTS_START_OFFSET TEXT_INIT + 60
#define TEXT_MARGIN_LEFT 50
#define TEXT_FONT_SCALE 0.5
#define TEXT_FONT_COLOR_FPS cv::Scalar( 0, 0, 255 )
#define TEXT_FONT_COLOR_COST cv::Scalar( 255, 0, 0 )
#define TEXT_FONT_COLOR_INFO cv::Scalar( 255, 255, 255 )
#define TEXT_THICKNESS 2

using namespace std;

namespace calibcv
{

    enum _pdWindowID
    {
        BASE = 0,
        MASK = 1,
        EDGES = 2,
        BLOBS = 3,
        TRACKING = 4,
        REFINING_UNDISTORTED = 5,
        REFINING_FRONTO = 6,
        REFINING_MASK = 7,
        REFINING_EDGES = 8,
        REFINING_FEATURES = 9,
        REFINING_PROJECTED = 10,
        REFINING_DISTORTED = 11,
        MAX_STAGES = 12
    };

    static map< _pdWindowID, int > TEXT_COSTS_OFFSETS( { { MASK, 100 },
                                                         { EDGES, 140 },
                                                         { BLOBS, 180 },
                                                         { TRACKING, 220 },
                                                         { REFINING_UNDISTORTED, 260 },
                                                         { REFINING_FRONTO, 300 },
                                                         { REFINING_MASK, 340 },
                                                         { REFINING_EDGES, 380 },
                                                         { REFINING_FEATURES, 420 },
                                                         { REFINING_PROJECTED, 460 },
                                                         { REFINING_DISTORTED, 500 } } );

    static map< _pdWindowID, string > WINDOW_MAP( { { BASE, "pd - base" },
                                                    { MASK, "pd - mask" },
                                                    { EDGES, "pd - edges" },
                                                    { BLOBS, "pd - blobs" },
                                                    { TRACKING, "pd - tracking" },
                                                    { REFINING_UNDISTORTED, "pd - ref.fronto" },
                                                    { REFINING_FRONTO, "pd - ref.fronto" },
                                                    { REFINING_MASK, "pd - ref.mask" },
                                                    { REFINING_EDGES, "pd - ref.edges" },
                                                    { REFINING_FEATURES, "pd - ref.features" },
                                                    { REFINING_PROJECTED, "pd - ref.projected" },
                                                    { REFINING_DISTORTED, "pd - ref.distorted" } } );

    static map< _pdWindowID, int > ACTIVE_WINDOWS( { { BASE, 1 },
                                                     { MASK, 0 },
                                                     { EDGES, 0 },
                                                     { BLOBS, 0 },
                                                     { TRACKING, 0 },
                                                     { REFINING_UNDISTORTED, 0 },
                                                     { REFINING_FRONTO, 0 },
                                                     { REFINING_MASK, 0 },
                                                     { REFINING_EDGES, 0 },
                                                     { REFINING_FEATURES, 0 },
                                                     { REFINING_PROJECTED, 0 },
                                                     { REFINING_DISTORTED, 0 } } );


    class SPatternDetectorPanel
    {

        private :

        cv::Mat m_baseBg;

        int m_windowsStates[5];

        string m_logInfo;

        SPatternDetectorPanel();
        void init();

        void _updateWindowMode( _pdWindowID windowId );

        public :

        static SPatternDetectorPanel* INSTANCE;
        static SPatternDetectorPanel* create();
        static void release();

        ~SPatternDetectorPanel();

        void cleanInfo();

        void showBase( const cv::Mat& mat );
        void showMask( const cv::Mat& mat );
        void showEdges( const cv::Mat& mat );
        void showBlobs( const cv::Mat& mat );
        void showTracking( const cv::Mat& mat );

        // Refining stage

        void showRefUndistorted( const cv::Mat& mat );
        void showRefFronto( const cv::Mat& mat );
        void showRefMask( const cv::Mat& mat );
        void showRefEdges( const cv::Mat& mat );
        void showRefFeatures( const cv::Mat& mat );
        void showRefProjected( const cv::Mat& mat );
        void showRefDistorted( const cv::Mat& mat );

        void setFPS( float fps );
        void setStageCost( float msCost, _pdWindowID stageId );

        void setLogInfo( string txtInfo );

        static void onHideWindowCallback( int dummyInt, void* dummyPtr );
    };



}
