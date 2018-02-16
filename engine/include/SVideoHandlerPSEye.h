
#pragma once

#include "SVideoHandler.h"

#include <camcore/SLinuxCamHandler.h>

using namespace std;

namespace calibcv
{

    class SVideoHandlerPSEye : public SVideoHandler
    {

        private :

        cam::handler::SLinuxCamHandler* m_camHandler;

        protected :

        SVideoHandlerPSEye();
        void init() override;

        void _takeFrameFromVideo( cv::Mat& dstFrame ) override;
        void _takeFrameFromCamera( cv::Mat& dstFrame ) override;

        public :

        static SVideoHandlerPSEye* INSTANCE;
        static SVideoHandlerPSEye* create();
        static void release();
        ~SVideoHandlerPSEye();

        bool openVideo( string filename ) override;
        bool openCamera( int deviceId ) override;

        void setPlaybackAtFrameIndex( int indx ) override;

        static void onMouseCallback( int event, int x, int y, int flags, void* userData );
        static void onTrackbarROIcallback( int dummyInt, void* dummyPtr );
    };



}
