
#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>

using namespace std;

namespace calibcv
{

    class CVideoHandler
    {

        private :

        cv::VideoCapture* m_capDevice;

        public :

        CVideoHandler()
        {
            m_capDevice = new cv::VideoCapture();
        }

        ~CVideoHandler()
        {
            if ( m_capDevice != NULL )
            {
                delete m_capDevice;
                m_capDevice = NULL;
            }
        }

        bool openVideo( string filename )
        {
            cout << "opening file: " << filename << endl;
            m_capDevice->open( filename );

            return m_capDevice->isOpened();
        }

        bool openCamera( int deviceId )
        {
            cout << "opening device: " << deviceId << endl;
            m_capDevice->open( deviceId );

            return m_capDevice->isOpened();
        }

        void takeFrame( cv::Mat& dstFrame )
        {
            m_capDevice->read( dstFrame );
        }

    };

}