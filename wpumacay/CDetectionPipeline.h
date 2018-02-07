
#pragma once


#include "CCommon.h"

#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <unordered_map>


using namespace std;

#define DETECT_PIPELINE_CS0MIN 165
#define DETECT_PIPELINE_CS0MAX 255
#define DETECT_PIPELINE_CS1MIN 24
#define DETECT_PIPELINE_CS1MAX 162
#define DETECT_PIPELINE_CS2MIN 98
#define DETECT_PIPELINE_CS2MAX 162

#define DETECT_PIPELINE_MORPH_ELEMENT_SIZE 2
#define DETECT_PIPELINE_MORPH_ELEMENT_TYPE cv::MORPH_CROSS
#define DETECT_PIPELINE_BLUR_MASKED_SIZE 3
#define DETECT_PIPELINE_CANNY_MIN 50
#define DETECT_PIPELINE_CANNY_MAX 150
#define DETECT_PIPELINE_CANNY_SOBEL_MASK_SIZE 3

#define DETECT_PIPELINE_ELLIPSE_COUNT_THRESHOLD 7

namespace calibcv { namespace detection {


    class CDetectionPipeline
    {

        private :

        cv::Mat m_morphElement;
        cv::Scalar m_cspaceMin;
        cv::Scalar m_cspaceMax;

        cv::Mat m_currentFrame;

        CDetectionPipeline();
        void init();

        public :

        static CDetectionPipeline* INSTANCE;
        static CDetectionPipeline* create();
        static void release();

        ~CDetectionPipeline();

        void run( const cv::Mat& frame );

        // Mask creation using :
        // 1) channel equalization
        // 2) colorspace thresholding
        // 3) morph opening
        void stepMaskCreation( const cv::Mat& frame, cv::Mat& mask );

        // Creation of masked gray image :
        // 1) bitwise and with mask
        void stepMaskedCreation( const cv::Mat& frame, const cv::Mat& mask, cv::Mat& masked );

        // Creation of edges-image :
        // 1) blur
        // 2) Canny
        void stepEdgesCreation( const cv::Mat& masked, cv::Mat& edges,
                                int blurSize = DETECT_PIPELINE_BLUR_MASKED_SIZE, 
                                int cannyMin = DETECT_PIPELINE_CANNY_MIN, 
                                int cannyMax = DETECT_PIPELINE_CANNY_MAX, 
                                int cannyMaskSize = DETECT_PIPELINE_CANNY_SOBEL_MASK_SIZE );

        // First step of ellipses finding - just find them all
        // 1) find contours
        // 2) fit ellipses
        void stepFindEllipses( const cv::Mat& edges, 
                               cv::Mat& matContours,
                               CTypeContours& contours, 
                               CTypeHierarchy& hierarchy,
                               vector< cv::RotatedRect >& ellipsesBOB,
                               int ellipsesCountThrehsold = DETECT_PIPELINE_ELLIPSE_COUNT_THRESHOLD );

        // Combine with particle-filtering
        // option 1
        // 1) sort by distance to tracked center
        // 2) keep better ellipses
        void stepProcessEllipses( const vector< cv::RotatedRect>& inEllipses,
                                  vector< cv::RotatedRect>& outEllipses );

        void stepBlendResults( const cv::Mat& frame,
                               cv::Mat& result,
                               const vector< cv::RotatedRect >& ellipses );

    };




}}





