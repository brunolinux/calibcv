
#include <SVideoHandler.h>

using namespace std;

namespace calibcv
{
    SVideoHandler* SVideoHandler::INSTANCE = NULL;

    SVideoHandler::SVideoHandler()
    {
        m_capDevice = new cv::VideoCapture();
        m_totalFrames = -1;// -1 if from live camera
        m_currentFrame = 0;

        m_isPaused = false;
        m_isPickingROI = false;
        m_readingFromVideo = true;
    }

    void SVideoHandler::init()
    {
        cv::namedWindow( SVH_INPUT_WINDOW );

        m_currentFrame = 0;

        cv::setMouseCallback( SVH_INPUT_WINDOW, SVideoHandler::onMouseCallback, NULL );
    }

    SVideoHandler::~SVideoHandler()
    {
        if ( m_capDevice != NULL )
        {
            delete m_capDevice;
            m_capDevice = NULL;
        }
    }

    SVideoHandler* SVideoHandler::create()
    {
        if ( SVideoHandler::INSTANCE != NULL )
        {
            delete SVideoHandler::INSTANCE;
        }

        SVideoHandler::INSTANCE = new SVideoHandler();
        SVideoHandler::INSTANCE->init();

        return SVideoHandler::INSTANCE;
    }

    void SVideoHandler::release()
    {
        if ( SVideoHandler::INSTANCE != NULL )
        {
            delete SVideoHandler::INSTANCE;
            SVideoHandler::INSTANCE = NULL;
        }
    }

    bool SVideoHandler::openVideo( string filename )
    {
        cout << "opening file: " << filename << endl;
        m_capDevice->open( filename );

        m_readingFromVideo = true;

        if ( m_capDevice->isOpened() )
        {
            cv::createTrackbar( "tbFrame", SVH_INPUT_WINDOW,
                                &SVideoHandler::INSTANCE->m_currentFrame,
                                1000,
                                SVideoHandler::onTrackbarCallback );

            m_totalFrames = m_capDevice->get( CV_CAP_PROP_FRAME_COUNT );
            cv::setTrackbarMax( "tbFrame", SVH_INPUT_WINDOW, m_totalFrames );

            return true;
        }

        return false;
    }

    bool SVideoHandler::openCamera( int deviceId )
    {
        cout << "opening device: " << deviceId << endl;
        m_capDevice->open( deviceId );

        m_readingFromVideo = false;

        return m_capDevice->isOpened();
    }

    bool SVideoHandler::isPaused()
    {
        return m_isPaused;
    }

    void SVideoHandler::setPaused( bool pPaused )
    {
        m_isPaused = pPaused;
    }

    void SVideoHandler::togglePause()
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

        // cout << "isPaused: " << m_isPaused << endl;
        m_isPaused = !m_isPaused; 
    }

    bool SVideoHandler::isPickingROI() 
    { 
        return m_isPickingROI; 
    }

    void SVideoHandler::setPickingROI( bool pPickingROI ) 
    { 
        m_isPickingROI = pPickingROI; 
    }

    void SVideoHandler::togglePickingROI()
    {
        m_isPickingROI = !m_isPickingROI;
        if ( m_isPickingROI )
        {
            m_roi.clear();
        }

        // cout << "isPickingROI: " << m_isPickingROI << endl;
    }

    void SVideoHandler::setPlaybackAtFrameIndex( int indx )
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
        cv::imshow( SVH_INPUT_WINDOW, _mat );

        m_currentFrame = indx;
        m_capDevice->set( CV_CAP_PROP_POS_FRAMES, m_currentFrame );
    }

    void SVideoHandler::_takeFrameFromVideo( cv::Mat& dstFrame )
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

            cv::imshow( SVH_INPUT_WINDOW, _videoFrame );
            m_capDevice->set( CV_CAP_PROP_POS_FRAMES, m_currentFrame );
        }
        else
        {
            m_capDevice->read( dstFrame );
            cv::Mat _videoFrame = dstFrame.clone();
            m_currentFrame++;
            cv::setTrackbarPos( "tbFrame", SVH_INPUT_WINDOW, m_currentFrame );

            for ( int q = 0; q < m_roi.size(); q++ )
            {
                int _indx1 = q;
                int _indx2 = ( q + 1 ) % m_roi.size();
                cv::line( _videoFrame, m_roi[ _indx1 ], m_roi[ _indx2 ],
                          cv::Scalar( 0, 0, 255 ), 4 );
            }

            cv::imshow( SVH_INPUT_WINDOW, _videoFrame );
        }

        m_lastFrame = dstFrame;
    }

    void SVideoHandler::_takeFrameFromCamera( cv::Mat& dstFrame )
    {
        if ( m_isPaused )
        {
            cv::Mat _videoFrame;
            m_capDevice->read( _videoFrame );
            dstFrame = m_lastFrame;

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

            cv::imshow( SVH_INPUT_WINDOW, _videoFrame );
        }
        else
        {
            m_capDevice->read( dstFrame );
            cv::Mat _videoFrame = dstFrame.clone();

            for ( int q = 0; q < m_roi.size(); q++ )
            {
                int _indx1 = q;
                int _indx2 = ( q + 1 ) % m_roi.size();
                cv::line( _videoFrame, m_roi[ _indx1 ], m_roi[ _indx2 ],
                          cv::Scalar( 0, 0, 255 ), 4 );
            }

            cv::imshow( SVH_INPUT_WINDOW, _videoFrame );

            m_lastFrame = dstFrame;
        }
    }

    void SVideoHandler::takeFrame( cv::Mat& dstFrame )
    {
        if ( m_readingFromVideo )
        {
            _takeFrameFromVideo( dstFrame );
        }
        else
        {
            _takeFrameFromCamera( dstFrame );
        }
    }

    void SVideoHandler::onTrackbarCallback( int dummyInt, void* dummyPtr )
    {
        if ( SVideoHandler::INSTANCE->m_totalFrames == -1 )
        {
            return;
        }

        SVideoHandler::INSTANCE->setPlaybackAtFrameIndex( SVideoHandler::INSTANCE->m_currentFrame );
    }

    void SVideoHandler::onMouseCallback( int event, int x, int y, int flags, void* userData )
    {
        if ( event == cv::EVENT_MOUSEMOVE )
        {
            SVideoHandler::INSTANCE->m_px = x;
            SVideoHandler::INSTANCE->m_py = y;

            // cout << "px: " << x << endl;
            // cout << "py: " << y << endl;
        }

        if ( !SVideoHandler::INSTANCE->m_isPickingROI )
        {
            return;
        }

        if ( event == cv::EVENT_LBUTTONDOWN )
        {
            SVideoHandler::INSTANCE->m_roi.push_back( cv::Point( x, y ) );
        }

        if ( SVideoHandler::INSTANCE->m_roi.size() == 4 )
        {
            SVideoHandler::INSTANCE->m_isPickingROI = false;
            // SVideoHandler::INSTANCE->m_isPaused = false;
        }

    }

}
