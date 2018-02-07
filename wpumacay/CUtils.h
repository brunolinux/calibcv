
#pragma once

#include <opencv2/opencv.hpp>
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

        CVideoHandler();
        void init();

        public :

        static CVideoHandler* INSTANCE;
        static CVideoHandler* create();
        static void release();
        ~CVideoHandler();

        bool openVideo( string filename );
        bool openCamera( int deviceId );

        void setPlaybackAtFrameIndex( int indx );
        void takeFrame( cv::Mat& dstFrame );

        static void onTrackbarCallback( int dummyInt, void* dummyPtr );

    };



}