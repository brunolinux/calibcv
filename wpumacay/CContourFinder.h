
#pragma once

#include "CCommon.h"

#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <unordered_map>

using namespace std;

namespace calibcv { namespace contours {

    enum _windowID
    {
        INPUT = 0,
        EDGES = 1,
        CONTOURS = 2,
        ELLIPSES = 3
    };

    static unordered_map< _windowID, string > WINDOW_MAP( { { Input, "cf - Input image" },
                                                            { EDGES, "cf - Edges" },
                                                            { CONTOURS, "cf - Contours" },
                                                            { ELLIPSES, "cf - Ellipses" } } );

    class CCountourFinder
    {

        private :





        CCountourFinder();
        void init();

        public :

        static CCountourFinder* INSTANCE;
        static CCountourFinder* create();
        static void release();

        ~CCountourFinder();


        void showInput( const cv::Mat& mat );
        void showEdges( const cv::Mat& mat );
        void showContours( const cv::Mat& mat );
        void showEllipses( const cv::Mat& mat );



    };





}}