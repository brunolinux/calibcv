
#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <unordered_map>

using namespace std;



namespace calibcv { namespace tuning {

    enum _windowID
    {
        ORIGINAL = 0,
        THRESHED = 1,
        EQUALIZED = 2
    };

    static unordered_map< _windowID, string > WINDOW_MAP( { { ORIGINAL, "Original" },
                                                            { THRESHED, "Threshed" },
                                                            { EQUALIZED, "Equalized" } } );


    class CThreshHSVTuner
    {

        private :

        int m_hMin;
        int m_hMax;

        int m_sMin;
        int m_sMax;

        int m_vMin;
        int m_vMax;

        cv::Scalar m_hsvMin;
        cv::Scalar m_hsvMax;

        CThreshHSVTuner();
        void init();

        public :

        static CThreshHSVTuner* INSTANCE;
        static CThreshHSVTuner* create();
        static void release();

        ~CThreshHSVTuner();

        void setBaseFrame( const cv::Mat& mat );
        void setThreshedFrame( const cv::Mat& bw );

        static void onTrackbarCallback( int dummyInt, void* dummyPtr );

        cv::Scalar hsvMin() { return m_hsvMin; }
        cv::Scalar hsvMax() { return m_hsvMax; }

    };





}}