
#include "CTuning.h"


namespace calibcv { namespace tuning {

    CThreshHSVTuner* CThreshHSVTuner::INSTANCE = NULL;

    CThreshHSVTuner::CThreshHSVTuner()
    {
    }

    void CThreshHSVTuner::init()
    {
        cv::namedWindow( WINDOW_MAP[ ORIGINAL ] );
        cv::namedWindow( WINDOW_MAP[ THRESHED ] );
        cv::namedWindow( WINDOW_MAP[ EQUALIZED ] );

        m_hMin = 0; m_hMax = 255;
        m_sMin = 0; m_sMax = 255;
        m_vMin = 0; m_vMax = 255;

        cv::createTrackbar( "hueMin", WINDOW_MAP[ ORIGINAL ],
                            &CThreshHSVTuner::INSTANCE->m_hMin,
                            255,
                            CThreshHSVTuner::onTrackbarCallback );
        cv::createTrackbar( "hueMax", WINDOW_MAP[ ORIGINAL ],
                            &CThreshHSVTuner::INSTANCE->m_hMax,
                            255,
                            CThreshHSVTuner::onTrackbarCallback );

        cv::createTrackbar( "satMin", WINDOW_MAP[ ORIGINAL ],
                            &CThreshHSVTuner::INSTANCE->m_sMin,
                            255,
                            CThreshHSVTuner::onTrackbarCallback );
        cv::createTrackbar( "satMax", WINDOW_MAP[ ORIGINAL ],
                            &CThreshHSVTuner::INSTANCE->m_sMax,
                            255,
                            CThreshHSVTuner::onTrackbarCallback );

        cv::createTrackbar( "valueMin", WINDOW_MAP[ ORIGINAL ],
                            &CThreshHSVTuner::INSTANCE->m_vMin,
                            255,
                            CThreshHSVTuner::onTrackbarCallback );
        cv::createTrackbar( "valueMax", WINDOW_MAP[ ORIGINAL ],
                            &CThreshHSVTuner::INSTANCE->m_vMax,
                            255,
                            CThreshHSVTuner::onTrackbarCallback );
    }

    CThreshHSVTuner* CThreshHSVTuner::create()
    {
        if ( CThreshHSVTuner::INSTANCE != NULL )
        {
            delete CThreshHSVTuner::INSTANCE;
        }

        CThreshHSVTuner::INSTANCE = new CThreshHSVTuner();
        CThreshHSVTuner::INSTANCE->init();

        return CThreshHSVTuner::INSTANCE;
    }

    void CThreshHSVTuner::release()
    {
        if ( CThreshHSVTuner::INSTANCE != NULL )
        {
            delete CThreshHSVTuner::INSTANCE;
            CThreshHSVTuner::INSTANCE = NULL;
        }
    }

    CThreshHSVTuner::~CThreshHSVTuner()
    {
        cv::destroyWindow( WINDOW_MAP[ ORIGINAL ] );
        cv::destroyWindow( WINDOW_MAP[ THRESHED ] );
        cv::destroyWindow( WINDOW_MAP[ EQUALIZED ] );
    }

    void CThreshHSVTuner::setBaseFrame( const cv::Mat& mat )
    {
        cv::imshow( WINDOW_MAP[ ORIGINAL ], mat );
    }

    void CThreshHSVTuner::setThreshedFrame( const cv::Mat& bw )
    {
        cv::imshow( WINDOW_MAP[ THRESHED ], bw );
    }

    void CThreshHSVTuner::onTrackbarCallback( int dummyInt, void* dummyPtr )
    {
        CThreshHSVTuner::INSTANCE->m_hsvMin = cv::Scalar( CThreshHSVTuner::INSTANCE->m_hMin,
                                                          CThreshHSVTuner::INSTANCE->m_sMin,
                                                          CThreshHSVTuner::INSTANCE->m_vMin );

        CThreshHSVTuner::INSTANCE->m_hsvMax = cv::Scalar( CThreshHSVTuner::INSTANCE->m_hMax,
                                                          CThreshHSVTuner::INSTANCE->m_sMax,
                                                          CThreshHSVTuner::INSTANCE->m_vMax );
    }


}}