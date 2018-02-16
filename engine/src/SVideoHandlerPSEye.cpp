
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

        m_fixedROIwidth = 265;
        m_fixedROIheight = 202;

        m_fixedROIoffX = 13;
        m_fixedROIoffY = -85;

        m_fixedROI = cv::Rect2i( 0, 0, 1, 1 );

        m_frameWidth = 640;
        m_frameHeight = 480;

        auto _fw = m_frameWidth;
        auto _fh = m_frameHeight;
        auto _rw = m_fixedROIwidth;
        auto _rh = m_fixedROIheight;
        auto _offX = m_fixedROIoffX;
        auto _offY = m_fixedROIoffY;
    }

    void SVideoHandlerPSEye::init()
    {
        cv::namedWindow( SVH_INPUT_WINDOW );

        m_currentFrame = 0;

        cv::setMouseCallback( SVH_INPUT_WINDOW, SVideoHandlerPSEye::onMouseCallback, NULL );
        cv::createTrackbar( "tbROIwidth", SVH_INPUT_WINDOW,
                            &SVideoHandlerPSEye::INSTANCE->m_fixedROIwidth,
                            400, SVideoHandlerPSEye::onTrackbarROIcallback );
        cv::createTrackbar( "tbROIheight", SVH_INPUT_WINDOW,
                            &SVideoHandlerPSEye::INSTANCE->m_fixedROIheight,
                            400, SVideoHandlerPSEye::onTrackbarROIcallback );
        cv::createTrackbar( "tbROIoffX", SVH_INPUT_WINDOW,
                            &SVideoHandlerPSEye::INSTANCE->m_fixedROIoffX,
                            200, SVideoHandlerPSEye::onTrackbarROIcallback );
        cv::createTrackbar( "tbROIoffY", SVH_INPUT_WINDOW,
                            &SVideoHandlerPSEye::INSTANCE->m_fixedROIoffY,
                            200, SVideoHandlerPSEye::onTrackbarROIcallback );

        cv::setTrackbarMin( "tbROIoffX", SVH_INPUT_WINDOW, -200 );
        cv::setTrackbarMax( "tbROIoffX", SVH_INPUT_WINDOW, 200 );

        cv::setTrackbarMin( "tbROIoffY", SVH_INPUT_WINDOW, -200 );
        cv::setTrackbarMax( "tbROIoffY", SVH_INPUT_WINDOW, 200 );
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
        m_camHandler->deviceSetStreamingProperty( CAMPROP_STREAMING_FRAMERATE, 60 );

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
        if ( m_isPaused )
        {
            cv::Mat _videoFrame;
            // cout << "? 1" << endl;
            // auto _rgbFrame = m_camHandler->takeFrame();
            cam::SImageRGB _rgbFrame;
            m_camHandler->takeFrame( _rgbFrame );
            // cout << "? 2" << endl;
            imgRgb2cvMat( _videoFrame, _rgbFrame );
            dstFrame = m_lastFrame;
            cv::cvtColor( dstFrame, dstFrame, cv::COLOR_RGB2BGR );

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

            cv::rectangle( _videoFrame, m_fixedROI, cv::Scalar( 0, 0, 255 ), 2 );
            cv::imshow( SVH_INPUT_WINDOW, _videoFrame );
        }
        else
        {
            // cout << "? 1" << endl;
            // auto _rgbFrame = m_camHandler->takeFrame();
            cam::SImageRGB _rgbFrame;
            m_camHandler->takeFrame( _rgbFrame );

            // cout << "? 2" << endl;
            imgRgb2cvMat( dstFrame, _rgbFrame );
            cv::cvtColor( dstFrame, dstFrame, cv::COLOR_RGB2BGR );
            // cout << "? 3" << endl;

            // cout << "dstFrame.rows: " << dstFrame.rows << endl;
            // cout << "dstFrame.cols: " << dstFrame.cols << endl;

            cv::Mat _videoFrame = dstFrame.clone();

            for ( int q = 0; q < m_roi.size(); q++ )
            {
                int _indx1 = q;
                int _indx2 = ( q + 1 ) % m_roi.size();
                cv::line( _videoFrame, m_roi[ _indx1 ], m_roi[ _indx2 ],
                          cv::Scalar( 0, 0, 255 ), 4 );
            }

            cv::rectangle( _videoFrame, m_fixedROI, cv::Scalar( 0, 0, 255 ), 2 );
            cv::imshow( SVH_INPUT_WINDOW, _videoFrame );

            m_lastFrame = dstFrame;
        }
    }

    void SVideoHandlerPSEye::onMouseCallback( int event, int x, int y, int flags, void* userData )
    {
        if ( event == cv::EVENT_MOUSEMOVE )
        {
            SVideoHandlerPSEye::INSTANCE->m_px = x;
            SVideoHandlerPSEye::INSTANCE->m_py = y;

            // cout << "px: " << x << endl;
            // cout << "py: " << y << endl;
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
            // SVideoHandlerPSEye::INSTANCE->m_isPaused = false;
        }

    }

    void SVideoHandlerPSEye::onTrackbarROIcallback( int dummyInt, void* dummyPtr )
    {
        auto _fw = SVideoHandlerPSEye::INSTANCE->m_frameWidth;
        auto _fh = SVideoHandlerPSEye::INSTANCE->m_frameHeight;
        auto _rw = SVideoHandlerPSEye::INSTANCE->m_fixedROIwidth;
        auto _rh = SVideoHandlerPSEye::INSTANCE->m_fixedROIheight;
        auto _offX = SVideoHandlerPSEye::INSTANCE->m_fixedROIoffX;
        auto _offY = SVideoHandlerPSEye::INSTANCE->m_fixedROIoffY;

        SVideoHandlerPSEye::INSTANCE->m_fixedROI = cv::Rect2i( 0.5 * _fw - 0.5 * _rw + _offX,
                                                               0.5 * _fh - 0.5 * _rh + _offY,
                                                               _rw, _rh );
    }

}
