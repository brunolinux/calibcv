
#include "CDetectionPanel.h"

using namespace std;

namespace calibcv { namespace detection {


   CDetectionPanel* CDetectionPanel::INSTANCE = NULL;

    CDetectionPanel::CDetectionPanel()
    {
    }

    void CDetectionPanel::init()
    {
        cv::namedWindow( WINDOW_MAP[ MASK ] );
        cv::namedWindow( WINDOW_MAP[ MASKED ] );
        cv::namedWindow( WINDOW_MAP[ EDGES ] );
        cv::namedWindow( WINDOW_MAP[ CONTOURS ] );
        cv::namedWindow( WINDOW_MAP[ ELLIPSES ] );

        m_blurSize = BLUR_SIZE;
        m_cannyMin = CANNY_MIN;
        m_cannyMax = CANNY_MAX;
        m_cannySobelMaskSize = CANNY_SOBEL_MASK_SIZE;
        m_cannySobelMaskDelta = 0;
        m_ellipseCountThreshold = ELLIPSE_COUNT_THRESHOLD;

        m_ellipseMinSize  = ELLIPSE_MIN_SIZE;
        m_ellipseMaxSize  = ELLIPSE_MAX_SIZE;
        m_ellipseMinRatio = ELLIPSE_MIN_RATIO;
        m_ellipseMaxRatio = ELLIPSE_MAX_RATIO;

        cv::createTrackbar( "cannyMin", WINDOW_MAP[ MASK ],
                            &CDetectionPanel::INSTANCE->m_cannyMin,
                            255,
                            CDetectionPanel::onTrackbarCallback );
        cv::createTrackbar( "cannyMax", WINDOW_MAP[ MASK ],
                            &CDetectionPanel::INSTANCE->m_cannyMax,
                            255 * 3,
                            CDetectionPanel::onTrackbarCallback );
        cv::createTrackbar( "cannyMaskSize", WINDOW_MAP[ MASK ],
                            &CDetectionPanel::INSTANCE->m_cannySobelMaskDelta,
                            2,
                            CDetectionPanel::onTrackbarCallback );
        cv::createTrackbar( "ellipseCountThreshold", WINDOW_MAP[ MASK ],
                            &CDetectionPanel::INSTANCE->m_ellipseCountThreshold,
                            255,
                            CDetectionPanel::onTrackbarCallback );

        cv::createTrackbar( "ellipseMinSize", WINDOW_MAP[ MASK ],
                            &CDetectionPanel::INSTANCE->m_ellipseMinSize,
                            200,
                            CDetectionPanel::onTrackbarCallback );
        cv::createTrackbar( "ellipseMaxSize", WINDOW_MAP[ MASK ],
                            &CDetectionPanel::INSTANCE->m_ellipseMaxSize,
                            200,
                            CDetectionPanel::onTrackbarCallback );
        cv::createTrackbar( "ellipseMinRatio", WINDOW_MAP[ MASK ],
                            &CDetectionPanel::INSTANCE->m_ellipseMinRatio,
                            200,
                            CDetectionPanel::onTrackbarCallback );
        cv::createTrackbar( "ellipseMaxRatio", WINDOW_MAP[ MASK ],
                            &CDetectionPanel::INSTANCE->m_ellipseMaxRatio,
                            200,
                            CDetectionPanel::onTrackbarCallback );
    }

    CDetectionPanel* CDetectionPanel::create()
    {
        if ( CDetectionPanel::INSTANCE != NULL )
        {
            delete CDetectionPanel::INSTANCE;
        }

        CDetectionPanel::INSTANCE = new CDetectionPanel();
        CDetectionPanel::INSTANCE->init();

        return CDetectionPanel::INSTANCE;
    }

    void CDetectionPanel::release()
    {
        if ( CDetectionPanel::INSTANCE != NULL )
        {
            delete CDetectionPanel::INSTANCE;
            CDetectionPanel::INSTANCE = NULL;
        }
    }

    CDetectionPanel::~CDetectionPanel()
    {
        cv::destroyWindow( WINDOW_MAP[ MASK ] );
        cv::destroyWindow( WINDOW_MAP[ MASKED ] );
        cv::destroyWindow( WINDOW_MAP[ EDGES ] );
        cv::destroyWindow( WINDOW_MAP[ CONTOURS ] );
        cv::destroyWindow( WINDOW_MAP[ ELLIPSES ] );
    }

    void CDetectionPanel::showMask( const cv::Mat& mat )
    {
        cv::imshow( WINDOW_MAP[ MASK ], mat );
    }

    void CDetectionPanel::showMasked( const cv::Mat& mat )
    {
        cv::imshow( WINDOW_MAP[ MASKED ], mat );
    }

    void CDetectionPanel::showEdges( const cv::Mat& bw )
    {
        cv::imshow( WINDOW_MAP[ EDGES ], bw );
    }

    void CDetectionPanel::showContours( const cv::Mat& mat )
    {
        cv::imshow( WINDOW_MAP[ CONTOURS ], mat );
    }

    void CDetectionPanel::showEllipses( const cv::Mat& mat )
    {
        cv::imshow( WINDOW_MAP[ ELLIPSES ], mat );
    }

    void CDetectionPanel::onTrackbarCallback( int dummyInt, void* dummyPtr )
    {
        CDetectionPanel::INSTANCE->m_cannySobelMaskSize = CANNY_SOBEL_MASK_SIZE + CDetectionPanel::INSTANCE->m_cannySobelMaskDelta * 2;

        if ( CDetectionPanel::INSTANCE->m_ellipseCountThreshold < ELLIPSE_COUNT_THRESHOLD )
        {
            CDetectionPanel::INSTANCE->m_ellipseCountThreshold = ELLIPSE_COUNT_THRESHOLD;
            cv::setTrackbarPos( "ellipseCountThreshold" , WINDOW_MAP[ MASK ], ELLIPSE_COUNT_THRESHOLD );
        }

        if ( CDetectionPanel::INSTANCE->m_ellipseMinSize > CDetectionPanel::INSTANCE->m_ellipseMaxSize )
        {
            CDetectionPanel::INSTANCE->m_ellipseMinSize = CDetectionPanel::INSTANCE->m_ellipseMaxSize;
            cv::setTrackbarPos( "ellipseMinSize", WINDOW_MAP[ MASK ], CDetectionPanel::INSTANCE->m_ellipseMinSize );
        }

        if ( CDetectionPanel::INSTANCE->m_ellipseMinRatio > CDetectionPanel::INSTANCE->m_ellipseMaxRatio )
        {
            CDetectionPanel::INSTANCE->m_ellipseMinRatio = CDetectionPanel::INSTANCE->m_ellipseMaxRatio;
            cv::setTrackbarPos( "ellipseMinAngle", WINDOW_MAP[ MASK ], CDetectionPanel::INSTANCE->m_ellipseMinRatio );
        }
    }




}}