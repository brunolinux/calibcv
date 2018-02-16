
#include <panels/SPatternDetectorPanel.h>

using namespace std;

namespace calibcv
{

    SPatternDetectorPanel* SPatternDetectorPanel::INSTANCE = NULL;

    SPatternDetectorPanel::SPatternDetectorPanel()
    {
        m_windowsStates[BASE] = 1;
        m_windowsStates[PREPROCESSING] = 0;
        m_windowsStates[MASK] = 0;
        m_windowsStates[EDGES] = 0;
        m_windowsStates[BLOBS] = 0;
        m_windowsStates[TRACKING] = 0;

        m_baseBg = cv::Mat::zeros( 640, 480, CV_8UC3 );
    }

    void SPatternDetectorPanel::init()
    {
        cv::namedWindow( WINDOW_MAP[ BASE ] );
        // cv::namedWindow( WINDOW_MAP[ MASK ] );
        // cv::namedWindow( WINDOW_MAP[ EDGES ] );
        // cv::namedWindow( WINDOW_MAP[ BLOBS ] );
        // cv::namedWindow( WINDOW_MAP[ TRACKING ] );

        cv::createTrackbar( "hide-preprocessing", WINDOW_MAP[ BASE ],
                            &m_windowsStates[ PREPROCESSING ],
                            1,
                            SPatternDetectorPanel::onHideWindowCallback );

        cv::createTrackbar( "hide-mask", WINDOW_MAP[ BASE ],
                            &m_windowsStates[ MASK ],
                            1,
                            SPatternDetectorPanel::onHideWindowCallback );

        cv::createTrackbar( "hide-edges", WINDOW_MAP[ BASE ],
                            &m_windowsStates[ EDGES ],
                            1,
                            SPatternDetectorPanel::onHideWindowCallback );

        cv::createTrackbar( "hide-blobs", WINDOW_MAP[ BASE ],
                            &m_windowsStates[ BLOBS ],
                            1,
                            SPatternDetectorPanel::onHideWindowCallback );

        cv::createTrackbar( "hide-tracking", WINDOW_MAP[ BASE ],
                            &m_windowsStates[ TRACKING ],
                            1,
                            SPatternDetectorPanel::onHideWindowCallback );

        showBase( m_baseBg );
    }

    SPatternDetectorPanel* SPatternDetectorPanel::create()
    {
        if ( SPatternDetectorPanel::INSTANCE != NULL )
        {
            delete SPatternDetectorPanel::INSTANCE;
        }

        SPatternDetectorPanel::INSTANCE = new SPatternDetectorPanel();
        SPatternDetectorPanel::INSTANCE->init();

        return SPatternDetectorPanel::INSTANCE;
    }

    void SPatternDetectorPanel::release()
    {
        if ( SPatternDetectorPanel::INSTANCE != NULL )
        {
            delete SPatternDetectorPanel::INSTANCE;
            SPatternDetectorPanel::INSTANCE = NULL;
        }
    }

    SPatternDetectorPanel::~SPatternDetectorPanel()
    {
        cv::destroyWindow( WINDOW_MAP[ BASE ] );
        cv::destroyWindow( WINDOW_MAP[ MASK ] );
        cv::destroyWindow( WINDOW_MAP[ EDGES ] );
        cv::destroyWindow( WINDOW_MAP[ BLOBS ] );
        cv::destroyWindow( WINDOW_MAP[ TRACKING ] );
    }

    void SPatternDetectorPanel::showBase( const cv::Mat& mat )
    {
        cv::imshow( WINDOW_MAP[ BASE ], mat );
    }

    void SPatternDetectorPanel::showMask( const cv::Mat& mat )
    {
        if ( m_windowsStates[ MASK ] == 1 )
        {
            cv::imshow( WINDOW_MAP[ MASK ], mat );
        }
    }

    void SPatternDetectorPanel::showEdges( const cv::Mat& mat )
    {
        if ( m_windowsStates[ EDGES ] == 1 )
        {
            cv::imshow( WINDOW_MAP[ EDGES ], mat );
        }
    }

    void SPatternDetectorPanel::showBlobs( const cv::Mat& mat )
    {
        if ( m_windowsStates[ BLOBS ] == 1 )
        {
            cv::imshow( WINDOW_MAP[ BLOBS ], mat );
        }
    }

    void SPatternDetectorPanel::showTracking( const cv::Mat& mat )
    {
        if ( m_windowsStates[ TRACKING ] == 1 )
        {
            cv::imshow( WINDOW_MAP[ TRACKING ], mat );
        }
    }

    void SPatternDetectorPanel::showPreprocessing( const cv::Mat& mat )
    {
        if ( m_windowsStates[ PREPROCESSING ] == 1 )
        {
            cv::imshow( WINDOW_MAP[ PREPROCESSING ], mat );
        }
    }

    void SPatternDetectorPanel::_updateWindowMode( _pdWindowID windowId )
    {
        if ( ACTIVE_WINDOWS[ windowId ] == 1 && m_windowsStates[ windowId ] == 0 )
        {
            cv::destroyWindow( WINDOW_MAP[ windowId ] );
            ACTIVE_WINDOWS[ windowId ] = 0;
        }
        else if ( ACTIVE_WINDOWS[ windowId ] == 0 && m_windowsStates[ windowId ] == 1 )
        {
            cv::namedWindow( WINDOW_MAP[ windowId ] );
            ACTIVE_WINDOWS[ windowId ] = 1;
        }
    }

    void SPatternDetectorPanel::onHideWindowCallback( int dummyInt, void* dummyPtr )
    {
        for ( auto _it = WINDOW_MAP.begin(); _it != WINDOW_MAP.end(); _it++ )
        {
            SPatternDetectorPanel::INSTANCE->_updateWindowMode( _it->first );
        }
    }

    void SPatternDetectorPanel::setFPS( float fps )
    {
        // std::cout << "fps: " << fps << std::endl;
        string _fpsText = "FPS: ";
        _fpsText += to_string( fps );

        cv::putText( m_baseBg, _fpsText, cv::Point( TEXT_MARGIN_LEFT, TEXT_INIT ),
                     cv::FONT_HERSHEY_SIMPLEX, TEXT_FONT_SCALE, TEXT_FONT_COLOR_FPS,
                     TEXT_THICKNESS );

        showBase( m_baseBg );
    }

    void SPatternDetectorPanel::setStageCost( float msCost, _pdWindowID stageId )
    {
        // std::cout << WINDOW_MAP[ stageId ] << ": " << msCost << std::endl;
        string _costText = WINDOW_MAP[ stageId ];
        _costText += ": ";
        _costText += to_string( msCost );

        cv::putText( m_baseBg, _costText,
                     cv::Point( TEXT_MARGIN_LEFT, TEXT_COSTS_START_OFFSET + TEXT_COSTS_OFFSETS[ stageId ] ),
                     cv::FONT_HERSHEY_SIMPLEX, TEXT_FONT_SCALE, TEXT_FONT_COLOR_COST,
                     TEXT_THICKNESS );

        showBase( m_baseBg );
    }

    void SPatternDetectorPanel::setLogInfo( string txtInfo )
    {
        m_logInfo = txtInfo;
    }

    void SPatternDetectorPanel::cleanInfo()
    {
        m_baseBg.setTo( cv::Scalar( 0, 0, 0 ) );
        cv::putText( m_baseBg, m_logInfo,
                     cv::Point( 320, 240 ),
                     cv::FONT_HERSHEY_SIMPLEX, TEXT_FONT_SCALE, TEXT_FONT_COLOR_INFO,
                     TEXT_THICKNESS );
    }
}
