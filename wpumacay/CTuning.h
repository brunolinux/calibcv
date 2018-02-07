
#pragma once

#include "CCommon.h"

#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <unordered_map>

using namespace std;

// // Params for simple hsv thresholding
// #define DEF_CS0MIN 11
// #define DEF_CS0MAX 148
// #define DEF_CS1MIN 0
// #define DEF_CS1MAX 35
// #define DEF_CS2MIN 123
// #define DEF_CS2MAX 245

// // Params for hsv threhsolding with value equalization
// #define DEF_CS0MIN 0
// #define DEF_CS0MAX 136
// #define DEF_CS1MIN 0
// #define DEF_CS1MAX 23
// #define DEF_CS2MIN 132
// #define DEF_CS2MAX 255

// Params for YCrCb thresholding with Y equalization
#define DEF_CS0MIN 0
#define DEF_CS0MAX 136
#define DEF_CS1MIN 0
#define DEF_CS1MAX 23
#define DEF_CS2MIN 132
#define DEF_CS2MAX 255

namespace calibcv { namespace tuning {

    enum _windowID
    {
        ORIGINAL = 0,
        THRESHED = 1,
        EQUALIZED_BEF = 2,
        EQUALIZED_AFTER = 3
    };

    enum _cspace
    {
        CS_RGB,
        CS_HSV
    };

    static unordered_map< _windowID, string > WINDOW_MAP( { { ORIGINAL, "Original" },
                                                            { THRESHED, "Threshed" },
                                                            { EQUALIZED_BEF, "Equalized Bef" },
                                                            { EQUALIZED_AFTER, "Equalized After" } } );


    class CColorSpaceTuner
    {

        private :

        int m_csMin[3];
        int m_csMax[3];

        cv::Scalar m_cspaceMin;
        cv::Scalar m_cspaceMax;

        CColorSpaceTuner();
        void init();

        public :

        static CColorSpaceTuner* INSTANCE;
        static CColorSpaceTuner* create();
        static void release();

        ~CColorSpaceTuner();

        void setBaseFrame( const cv::Mat& mat );
        void setThreshedFrame( const cv::Mat& bw );
        void setEqualizedFrameBef( const cv::Mat& mat );
        void setEqualizedFrameAfter( const cv::Mat& mat );

        static void onTrackbarCallback( int dummyInt, void* dummyPtr );

        cv::Scalar cspaceMin() { return m_cspaceMin; }
        cv::Scalar cspaceMax() { return m_cspaceMax; }

    };





}}