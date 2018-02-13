
#pragma once

#include "../Common.h"

using namespace std;

#define SVH_INPUT_WINDOW "SVH - Input window"

namespace calibcv
{

    class SVideoHandler
    {

        protected :

        cv::VideoCapture* m_capDevice;
        cv::Mat m_lastFrame;

        int m_totalFrames;
        int m_currentFrame;

        bool m_isPaused;
        bool m_isPickingROI;

        int m_px;
        int m_py;

        bool m_readingFromVideo;

        vector< cv::Point > m_roi;

        SVideoHandler();
        void init();

        void _takeFrameFromVideo( cv::Mat& dstFrame );
        void _takeFrameFromCamera( cv::Mat& dstFrame );

        public :

        static SVideoHandler* INSTANCE;
        static SVideoHandler* create();
        static void release();
        ~SVideoHandler();

        bool openVideo( string filename );
        bool openCamera( int deviceId );

        bool isPaused();
        void setPaused( bool pPaused );
        void togglePause();

        bool isPickingROI();
        void setPickingROI( bool pPickingROI );
        void togglePickingROI();

        void setPlaybackAtFrameIndex( int indx );
        void takeFrame( cv::Mat& dstFrame );

        vector< cv::Point > roi() { return m_roi; }

        bool isReadingFromVideo() { return m_readingFromVideo; }

        static void onTrackbarCallback( int dummyInt, void* dummyPtr );
        static void onMouseCallback( int event, int x, int y, int flags, void* userData );
    };



}
