
#include <SVideoHandlerPSEye.h>
#include <SUtils.h>

using namespace std;

namespace calibcv
{
    SVideoHandlerPSEye* SVideoHandlerPSEye::INSTANCE = NULL;

    SVideoHandlerPSEye::SVideoHandlerPSEye()
    {
        m_camHandler = new cam::handler::SLinuxCamHandler( "/dev/video0",
                                                           640, 480 );
        m_totalFrames = -1;// -1 if from live camera
        m_currentFrame = 0;

        m_isPaused = false;
        m_isPickingROI = false;
        m_readingFromVideo = true;

        m_frameWidth = 640;
        m_frameHeight = 480;
    }

    void SVideoHandlerPSEye::init()
    {
        cv::namedWindow( SVH_INPUT_WINDOW );

        m_currentFrame = 0;

        cv::setMouseCallback( SVH_INPUT_WINDOW, SVideoHandlerPSEye::onMouseCallback, NULL );
    }

    SVideoHandlerPSEye::~SVideoHandlerPSEye()
    {
        if ( m_camHandler != NULL )
        {
            delete m_camHandler;
            m_camHandler = NULL;
        }
    }

    SVideoHandlerPSEye* SVideoHandlerPSEye::create()
    {
        if ( SVideoHandlerPSEye::INSTANCE != NULL )
        {
            delete SVideoHandlerPSEye::INSTANCE;
        }

        SVideoHandlerPSEye::INSTANCE = new SVideoHandlerPSEye();
        SVideoHandlerPSEye::INSTANCE->init();

        return SVideoHandlerPSEye::INSTANCE;
    }

    void SVideoHandlerPSEye::release()
    {
        if ( SVideoHandlerPSEye::INSTANCE != NULL )
        {
            delete SVideoHandlerPSEye::INSTANCE;
            SVideoHandlerPSEye::INSTANCE = NULL;
        }
    }

    bool SVideoHandlerPSEye::openVideo( string filename )
    {
        cout << "SVideoHandlerPSEye::openVideo> live video streaming only" << endl;
        return false;
    }

    bool SVideoHandlerPSEye::openCamera( int deviceId )
    {
        m_camHandler->openDevice();
        // m_camHandler->deviceSetStreamingProperty( CAMPROP_STREAMING_FRAMERATE, 60 );

        m_camHandler->startCapture();

        m_readingFromVideo = false;

        return true;
    }

    void SVideoHandlerPSEye::setPlaybackAtFrameIndex( int indx )
    {
        cout << "SVideoHandlerPSEye::setPlaybackAtFrameIndex> live video streaming only" << endl;
    }

    void SVideoHandlerPSEye::_takeFrameFromVideo( cv::Mat& dstFrame )
    {
        cout << "SVideoHandlerPSEye::_takeFrameFromVideo> live video streaming only" << endl;
    }

    void SVideoHandlerPSEye::_takeFrameFromCamera( cv::Mat& dstFrame )
    {
        cv::Mat _videoFrame;

        if ( m_isPaused )
        {
            cam::SImageRGB _rgbFrame;
            m_camHandler->takeFrame( _rgbFrame );
            
            imgRgb2cvMat( _videoFrame, _rgbFrame );
            dstFrame = m_lastFrame;
            cv::cvtColor( dstFrame, dstFrame, cv::COLOR_RGB2BGR );
        }
        else
        {
            cam::SImageRGB _rgbFrame;
            m_camHandler->takeFrame( _rgbFrame );

            imgRgb2cvMat( dstFrame, _rgbFrame );
            cv::cvtColor( dstFrame, dstFrame, cv::COLOR_RGB2BGR );

            _videoFrame = dstFrame.clone();

            m_lastFrame = dstFrame;
        }

        if ( m_isPickingROI )
        {
            if ( 0 < m_roi.size() && m_roi.size() < 4 )
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
        }
        
        if ( m_roi.size() == 4 )
        {
            for ( int q = 0; q < m_roi.size(); q++ )
            {
                int _indx1 = q;
                int _indx2 = ( q + 1 ) % m_roi.size();
                cv::line( _videoFrame, m_roi[ _indx1 ], m_roi[ _indx2 ],
                          cv::Scalar( 0, 0, 255 ), 4 );
            }
        }

        cv::imshow( SVH_INPUT_WINDOW, _videoFrame );
    }

    void SVideoHandlerPSEye::onMouseCallback( int event, int x, int y, int flags, void* userData )
    {
        if ( event == cv::EVENT_MOUSEMOVE )
        {
            SVideoHandlerPSEye::INSTANCE->m_px = x;
            SVideoHandlerPSEye::INSTANCE->m_py = y;
        }

        if ( !SVideoHandlerPSEye::INSTANCE->m_isPickingROI )
        {
            return;
        }

        if ( event == cv::EVENT_LBUTTONDOWN )
        {
            SVideoHandlerPSEye::INSTANCE->m_roi.push_back( cv::Point( x, y ) );
        }

        if ( SVideoHandlerPSEye::INSTANCE->m_roi.size() == 4 )
        {
            SVideoHandlerPSEye::INSTANCE->m_isPickingROI = false;
        }

    }

}
