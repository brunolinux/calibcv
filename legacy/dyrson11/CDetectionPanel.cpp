
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
    }




}}