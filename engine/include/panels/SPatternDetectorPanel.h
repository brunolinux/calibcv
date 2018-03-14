
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
        BASE,
        MASK,
        EDGES,
        BLOBS,
        TRACKING,
        REFINING_FRONTO,
        REFINING_MASK,
        REFINING_EDGES,
        REFINING_FEATURES,
        REFINING_PROJECTED
    };

    static map< _pdWindowID, int > TEXT_COSTS_OFFSETS( { { MASK, 100 },
                                                         { EDGES, 160 },
                                                         { BLOBS, 220 },
                                                         { TRACKING, 280 },
                                                         { REFINING_FRONTO, 340 },
                                                         { REFINING_MASK, 400 },
                                                         { REFINING_EDGES, 460 },
                                                         { REFINING_FEATURES, 520 },
                                                         { REFINING_PROJECTED, 580 } } );

    static map< _pdWindowID, string > WINDOW_MAP( { { BASE, "pd - base" },
                                                    { MASK, "pd - mask" },
                                                    { EDGES, "pd - edges" },
                                                    { BLOBS, "pd - blobs" },
                                                    { TRACKING, "pd - tracking" },
                                                    { REFINING_FRONTO, "pd - ref.fronto" },
                                                    { REFINING_MASK, "pd - ref.mask" },
                                                    { REFINING_EDGES, "pd - ref.edges" },
                                                    { REFINING_FEATURES, "pd - ref.features" },
                                                    { REFINING_PROJECTED, "pd - ref.projected" } } );

    static map< _pdWindowID, int > ACTIVE_WINDOWS( { { BASE, 1 },
                                                     { MASK, 0 },
                                                     { EDGES, 0 },
                                                     { BLOBS, 0 },
                                                     { TRACKING, 0 },
                                                     { REFINING_FRONTO, 0 },
                                                     { REFINING_MASK, 0 },
                                                     { REFINING_EDGES, 0 },
                                                     { REFINING_FEATURES, 0 },
                                                     { REFINING_PROJECTED, 0 } } );


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

        void showRefFronto( const cv::Mat& mat );
        void showRefMask( const cv::Mat& mat );
        void showRefEdges( const cv::Mat& mat );
        void showRefFeatures( const cv::Mat& mat );
        void showRefProjected( const cv::Mat& mat );

        void setFPS( float fps );
        void setStageCost( float msCost, _pdWindowID stageId );

        void setLogInfo( string txtInfo );

        static void onHideWindowCallback( int dummyInt, void* dummyPtr );
    };



}
