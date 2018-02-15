
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
        cv::Rect2i m_fixedROI;

        int m_fixedROIwidth;
        int m_fixedROIheight;
        int m_fixedROIoffX;
        int m_fixedROIoffY;

        int m_frameWidth;
        int m_frameHeight;

        SVideoHandler();
        virtual void init();

        virtual void _takeFrameFromVideo( cv::Mat& dstFrame );
        virtual void _takeFrameFromCamera( cv::Mat& dstFrame );

        public :

        static SVideoHandler* INSTANCE;
        static SVideoHandler* create();
        static void release();
        ~SVideoHandler();

        virtual bool openVideo( string filename );
        virtual bool openCamera( int deviceId );

        bool isPaused();
        void setPaused( bool pPaused );
        void togglePause();

        bool isPickingROI();
        void setPickingROI( bool pPickingROI );
        void togglePickingROI();

        virtual void setPlaybackAtFrameIndex( int indx );
        void takeFrame( cv::Mat& dstFrame );

        vector< cv::Point > roi() { return m_roi; }
        cv::Rect2i fixedROI() { return m_fixedROI; }

        bool isReadingFromVideo() { return m_readingFromVideo; }

        static void onTrackbarCallback( int dummyInt, void* dummyPtr );
        static void onMouseCallback( int event, int x, int y, int flags, void* userData );

        static void onTrackbarROIcallback( int dummyInt, void* dummyPtr );
    };



}
