
#include "CTuning.h"


namespace calibcv { namespace tuning {

    CColorSpaceTuner* CColorSpaceTuner::INSTANCE = NULL;

    CColorSpaceTuner::CColorSpaceTuner()
    {
    }

    void CColorSpaceTuner::init()
    {
        cv::namedWindow( WINDOW_MAP[ ORIGINAL ] );
        cv::namedWindow( WINDOW_MAP[ THRESHED ] );
        cv::namedWindow( WINDOW_MAP[ EQUALIZED_BEF ] );
        cv::namedWindow( WINDOW_MAP[ EQUALIZED_AFTER ] );

        m_csMin[0] = DEF_CS0MIN; m_csMax[0] = DEF_CS0MAX;
        m_csMin[1] = DEF_CS1MIN; m_csMax[1] = DEF_CS1MAX;
        m_csMin[2] = DEF_CS2MIN; m_csMax[2] = DEF_CS2MAX;

        m_cspaceMin = cv::Scalar( m_csMin[0], m_csMin[1], m_csMin[2] );
        m_cspaceMax = cv::Scalar( m_csMax[0], m_csMax[1], m_csMax[2] );

        cv::createTrackbar( "csMin[0]", WINDOW_MAP[ ORIGINAL ],
                            &CColorSpaceTuner::INSTANCE->m_csMin[0],
                            255,
                            CColorSpaceTuner::onTrackbarCallback );
        cv::createTrackbar( "csMax[0]", WINDOW_MAP[ ORIGINAL ],
                            &CColorSpaceTuner::INSTANCE->m_csMax[0],
                            255,
                            CColorSpaceTuner::onTrackbarCallback );

        cv::createTrackbar( "csMin[1]", WINDOW_MAP[ ORIGINAL ],
                            &CColorSpaceTuner::INSTANCE->m_csMin[1],
                            255,
                            CColorSpaceTuner::onTrackbarCallback );
        cv::createTrackbar( "csMax[1]", WINDOW_MAP[ ORIGINAL ],
                            &CColorSpaceTuner::INSTANCE->m_csMax[1],
                            255,
                            CColorSpaceTuner::onTrackbarCallback );

        cv::createTrackbar( "csMin[2]", WINDOW_MAP[ ORIGINAL ],
                            &CColorSpaceTuner::INSTANCE->m_csMin[2],
                            255,
                            CColorSpaceTuner::onTrackbarCallback );
        cv::createTrackbar( "csMax[2]", WINDOW_MAP[ ORIGINAL ],
                            &CColorSpaceTuner::INSTANCE->m_csMax[2],
                            255,
                            CColorSpaceTuner::onTrackbarCallback );
    }

    CColorSpaceTuner* CColorSpaceTuner::create()
    {
        if ( CColorSpaceTuner::INSTANCE != NULL )
        {
            delete CColorSpaceTuner::INSTANCE;
        }

        CColorSpaceTuner::INSTANCE = new CColorSpaceTuner();
        CColorSpaceTuner::INSTANCE->init();

        return CColorSpaceTuner::INSTANCE;
    }

    void CColorSpaceTuner::release()
    {
        if ( CColorSpaceTuner::INSTANCE != NULL )
        {
            delete CColorSpaceTuner::INSTANCE;
            CColorSpaceTuner::INSTANCE = NULL;
        }
    }

    CColorSpaceTuner::~CColorSpaceTuner()
    {
        cv::destroyWindow( WINDOW_MAP[ ORIGINAL ] );
        cv::destroyWindow( WINDOW_MAP[ THRESHED ] );
        cv::destroyWindow( WINDOW_MAP[ EQUALIZED_BEF ] );
        cv::destroyWindow( WINDOW_MAP[ EQUALIZED_AFTER ] );
    }

    void CColorSpaceTuner::setBaseFrame( const cv::Mat& mat )
    {
        cv::imshow( WINDOW_MAP[ ORIGINAL ], mat );
    }

    void CColorSpaceTuner::setThreshedFrame( const cv::Mat& bw )
    {
        cv::imshow( WINDOW_MAP[ THRESHED ], bw );
    }

    void CColorSpaceTuner::setEqualizedFrameBef( const cv::Mat& mat )
    {
        cv::imshow( WINDOW_MAP[ EQUALIZED_BEF ], mat );
    }

    void CColorSpaceTuner::setEqualizedFrameAfter( const cv::Mat& mat )
    {
        cv::imshow( WINDOW_MAP[ EQUALIZED_AFTER ], mat );
    }

    void CColorSpaceTuner::onTrackbarCallback( int dummyInt, void* dummyPtr )
    {
        CColorSpaceTuner::INSTANCE->m_cspaceMin = cv::Scalar( CColorSpaceTuner::INSTANCE->m_csMin[0],
                                                              CColorSpaceTuner::INSTANCE->m_csMin[1],
                                                              CColorSpaceTuner::INSTANCE->m_csMin[2] );

        CColorSpaceTuner::INSTANCE->m_cspaceMax = cv::Scalar( CColorSpaceTuner::INSTANCE->m_csMax[0],
                                                              CColorSpaceTuner::INSTANCE->m_csMax[1],
                                                              CColorSpaceTuner::INSTANCE->m_csMax[2] );
    }


}}