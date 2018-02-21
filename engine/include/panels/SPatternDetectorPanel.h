
#pragma once

#include "../../Common.h"

#define TEXT_INIT 50
#define TEXT_COSTS_START_OFFSET TEXT_INIT + 100
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
        PREPROCESSING,
        MASK,
        EDGES,
        BLOBS,
        TRACKING
    };

    static map< _pdWindowID, int > TEXT_COSTS_OFFSETS( { { PREPROCESSING, 80 },
                                                                   { MASK, 160 },
                                                                   { EDGES, 240 },
                                                                   { BLOBS, 320 },
                                                                   { TRACKING, 400 } } );

    static map< _pdWindowID, string > WINDOW_MAP( { { BASE, "pd - base" },
                                                              { PREPROCESSING, "pd - preprocessing" },
                                                              { MASK, "pd - mask" },
                                                              { EDGES, "pd - edges" },
                                                              { BLOBS, "pd - blobs" },
                                                              { TRACKING, "pd - tracking" } } );

    static map< _pdWindowID, int > ACTIVE_WINDOWS( { { BASE, 1 },
                                                               { PREPROCESSING, 0 },
                                                               { MASK, 0 },
                                                               { EDGES, 0 },
                                                               { BLOBS, 0 },
                                                               { TRACKING, 0 } } );


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
        void showPreprocessing( const cv::Mat& mat );
        void showMask( const cv::Mat& mat );
        void showEdges( const cv::Mat& mat );
        void showBlobs( const cv::Mat& mat );
        void showTracking( const cv::Mat& mat );

        void setFPS( float fps );
        void setStageCost( float msCost, _pdWindowID stageId );

        void setLogInfo( string txtInfo );

        static void onHideWindowCallback( int dummyInt, void* dummyPtr );
    };



}
