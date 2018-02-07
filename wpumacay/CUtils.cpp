
#include "CUtils.h"

using namespace std;

namespace calibcv
{
    CVideoHandler* CVideoHandler::INSTANCE = NULL;

    CVideoHandler::CVideoHandler()
    {
        m_capDevice = new cv::VideoCapture();
        m_totalFrames = -1;// -1 if from live camera
        m_currentFrame = 0;
    }

    void CVideoHandler::init()
    {
        cv::namedWindow( INPUT_WINDOW );

        m_currentFrame = 0;

        cv::createTrackbar( "frame", INPUT_WINDOW,
                            &CVideoHandler::INSTANCE->m_currentFrame,
                            1000,
                            CVideoHandler::onTrackbarCallback );
    }

    CVideoHandler::~CVideoHandler()
    {
        if ( m_capDevice != NULL )
        {
            delete m_capDevice;
            m_capDevice = NULL;
        }
    }

    CVideoHandler* CVideoHandler::create()
    {
        if ( CVideoHandler::INSTANCE != NULL )
        {
            delete CVideoHandler::INSTANCE;
        }

        CVideoHandler::INSTANCE = new CVideoHandler();
        CVideoHandler::INSTANCE->init();

        return CVideoHandler::INSTANCE;
    }

    void CVideoHandler::release()
    {
        if ( CVideoHandler::INSTANCE != NULL )
        {
            delete CVideoHandler::INSTANCE;
            CVideoHandler::INSTANCE = NULL;
        }
    }

    bool CVideoHandler::openVideo( string filename )
    {
        cout << "opening file: " << filename << endl;
        m_capDevice->open( filename );

        if ( m_capDevice->isOpened() )
        {
            m_totalFrames = m_capDevice->get( CV_CAP_PROP_FRAME_COUNT );
            cv::setTrackbarMax( "frame", INPUT_WINDOW, m_totalFrames );

            return true;
        }

        return false;
    }

    bool CVideoHandler::openCamera( int deviceId )
    {
        cout << "opening device: " << deviceId << endl;
        m_capDevice->open( deviceId );

        return m_capDevice->isOpened();
    }

    void CVideoHandler::setPlaybackAtFrameIndex( int indx )
    {
        assert( m_capDevice != NULL );

        if ( m_currentFrame >= m_totalFrames )
        {
            return;
        }

        m_currentFrame = indx;
        m_capDevice->set( CV_CAP_PROP_POS_FRAMES, m_currentFrame );
    }

    void CVideoHandler::takeFrame( cv::Mat& dstFrame )
    {
        m_capDevice->read( dstFrame );
        m_currentFrame++;
        cv::setTrackbarPos( "frame", INPUT_WINDOW, m_currentFrame );
    }

    void CVideoHandler::onTrackbarCallback( int dummyInt, void* dummyPtr )
    {
        if ( CVideoHandler::INSTANCE->m_totalFrames == -1 )
        {
            return;
        }

        CVideoHandler::INSTANCE->setPlaybackAtFrameIndex( CVideoHandler::INSTANCE->m_currentFrame );
    }

}