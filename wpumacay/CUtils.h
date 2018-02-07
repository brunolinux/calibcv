
#pragma once

#include "CCommon.h"

#include <string>
#include <iostream>
#include <unordered_map>

using namespace std;

#define INPUT_WINDOW "Input window"

namespace calibcv
{

    class CVideoHandler
    {

        private :

        cv::VideoCapture* m_capDevice;
        int m_totalFrames;
        int m_currentFrame;

        bool m_isPaused;

        CVideoHandler();
        void init();

        public :

        static CVideoHandler* INSTANCE;
        static CVideoHandler* create();
        static void release();
        ~CVideoHandler();

        bool openVideo( string filename );
        bool openCamera( int deviceId );

        bool isPaused() { return m_isPaused; }
        void setPaused( bool pPaused ) { m_isPaused = pPaused; }
        void togglePause() { m_isPaused = !m_isPaused; }

        void setPlaybackAtFrameIndex( int indx );
        void takeFrame( cv::Mat& dstFrame );

        static void onTrackbarCallback( int dummyInt, void* dummyPtr );

    };



}
