
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

        m_isPaused = false;
        m_isPickingROI = false;
    }

    void CVideoHandler::init()
    {
        cv::namedWindow( INPUT_WINDOW );

        m_currentFrame = 0;

        cv::createTrackbar( "frame", INPUT_WINDOW,
                            &CVideoHandler::INSTANCE->m_currentFrame,
                            1000,
                            CVideoHandler::onTrackbarCallback );

        cv::setMouseCallback( INPUT_WINDOW, CVideoHandler::onMouseCallback, NULL );
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

        int _indxBef = ( indx < 1 ) ? 1 : ( indx - 1 );

        cv::Mat _mat;
        m_capDevice->set( CV_CAP_PROP_POS_FRAMES, _indxBef );
        m_capDevice->read( _mat );
        cv::imshow( INPUT_WINDOW, _mat );

        m_currentFrame = indx;
        m_capDevice->set( CV_CAP_PROP_POS_FRAMES, m_currentFrame );
    }

    void CVideoHandler::takeFrame( cv::Mat& dstFrame )
    {
        if ( m_isPaused )
        {
            m_capDevice->read( dstFrame );
            cv::Mat _videoFrame = dstFrame.clone();

            // if ( m_isPickingROI )
            // {
                if ( m_roi.size() > 0 && m_roi.size() < 4 )
                {
                    auto _roi = m_roi;
                    _roi.push_back( cv::Point( m_px, m_py ) );

                    for ( int q = 0; q < _roi.size(); q++ )
                    {
                        int _indx1 = q;
                        int _indx2 = ( q + 1 ) % _roi.size();
                        cv::line( _videoFrame, _roi[ _indx1 ], _roi[ _indx2 ],
                                  cv::Scalar( 0, 0, 255 ), 4 );
                    }
                }
                else
                {
                    for ( int q = 0; q < m_roi.size(); q++ )
                    {
                        int _indx1 = q;
                        int _indx2 = ( q + 1 ) % m_roi.size();
                        cv::line( _videoFrame, m_roi[ _indx1 ], m_roi[ _indx2 ],
                                  cv::Scalar( 0, 0, 255 ), 4 );
                    }
                }
            // }

            cv::imshow( INPUT_WINDOW, _videoFrame );
            m_capDevice->set( CV_CAP_PROP_POS_FRAMES, m_currentFrame );
        }
        else
        {
            m_capDevice->read( dstFrame );
            cv::Mat _videoFrame = dstFrame.clone();
            m_currentFrame++;
            cv::setTrackbarPos( "frame", INPUT_WINDOW, m_currentFrame );

            for ( int q = 0; q < m_roi.size(); q++ )
            {
                int _indx1 = q;
                int _indx2 = ( q + 1 ) % m_roi.size();
                cv::line( _videoFrame, m_roi[ _indx1 ], m_roi[ _indx2 ],
                          cv::Scalar( 0, 0, 255 ), 4 );
            }

            cv::imshow( INPUT_WINDOW, _videoFrame );
        }
    }

    void CVideoHandler::onTrackbarCallback( int dummyInt, void* dummyPtr )
    {
        if ( CVideoHandler::INSTANCE->m_totalFrames == -1 )
        {
            return;
        }

        CVideoHandler::INSTANCE->setPlaybackAtFrameIndex( CVideoHandler::INSTANCE->m_currentFrame );
    }

    void CVideoHandler::onMouseCallback( int event, int x, int y, int flags, void* userData )
    {
        if ( event == cv::EVENT_MOUSEMOVE )
        {
            CVideoHandler::INSTANCE->m_px = x;
            CVideoHandler::INSTANCE->m_py = y;

            // cout << "px: " << x << endl;
            // cout << "py: " << y << endl;
        }

        if ( !CVideoHandler::INSTANCE->m_isPickingROI )
        {
            return;
        }

        if ( event == cv::EVENT_LBUTTONDOWN )
        {
            CVideoHandler::INSTANCE->m_roi.push_back( cv::Point( x, y ) );
        }

        if ( CVideoHandler::INSTANCE->m_roi.size() == 4 )
        {
            CVideoHandler::INSTANCE->m_isPickingROI = false;
            // CVideoHandler::INSTANCE->m_isPaused = false;
        }

    }

}
