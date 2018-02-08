
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
        bool m_isPickingROI;

        int m_px;
        int m_py;

        vector< cv::Point > m_roi;

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
        void togglePause() 
        { 
            if ( m_isPaused && m_isPickingROI )
            {
                // if paused and picking roi, keep paused
                return;
            }

            if ( m_isPaused && m_roi.size() == 4 )
            {
                m_roi.clear();
            }

            cout << "isPaused: " << m_isPaused << endl;
            m_isPaused = !m_isPaused; 
        }

        bool isPickingROI() { return m_isPickingROI; }
        void setPickingROI( bool pPickingROI ) { m_isPickingROI = pPickingROI; }
        void togglePickingROI()
        {
            m_isPickingROI = !m_isPickingROI;
            if ( m_isPickingROI )
            {
                m_roi.clear();
            }

            cout << "isPickingROI: " << m_isPickingROI << endl;
        }

        void setPlaybackAtFrameIndex( int indx );
        void takeFrame( cv::Mat& dstFrame );

        vector< cv::Point > roi() { return m_roi; }

        static void onTrackbarCallback( int dummyInt, void* dummyPtr );
        static void onMouseCallback( int event, int x, int y, int flags, void* userData );
    };



}
