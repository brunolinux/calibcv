
#include "CDetectionPipeline.h"

#include <cmath>

using namespace std;


namespace calibcv { namespace detection {

    CDetectionPipeline* CDetectionPipeline::INSTANCE = NULL;

    CDetectionPipeline::CDetectionPipeline()
    {
        m_morphElement = cv::getStructuringElement( cv::MORPH_CROSS, 
                                                    cv::Size( 2 * DETECT_PIPELINE_MORPH_ELEMENT_SIZE + 1, 
                                                              2 * DETECT_PIPELINE_MORPH_ELEMENT_SIZE + 1 ),
                                                    cv::Point( DETECT_PIPELINE_MORPH_ELEMENT_SIZE, 
                                                               DETECT_PIPELINE_MORPH_ELEMENT_SIZE ) );

        m_cspaceMin = cv::Scalar( DETECT_PIPELINE_CS0MIN, 
                                  DETECT_PIPELINE_CS1MIN, 
                                  DETECT_PIPELINE_CS2MIN );

        m_cspaceMax = cv::Scalar( DETECT_PIPELINE_CS0MAX, 
                                  DETECT_PIPELINE_CS1MAX, 
                                  DETECT_PIPELINE_CS2MAX );
    }

    void CDetectionPipeline::init()
    {
        // Init some stuff
    }

    CDetectionPipeline* CDetectionPipeline::create()
    {
        if ( CDetectionPipeline::INSTANCE != NULL )
        {
            delete CDetectionPipeline::INSTANCE;
        }

        CDetectionPipeline::INSTANCE = new CDetectionPipeline();
        CDetectionPipeline::INSTANCE->init();

        return CDetectionPipeline::INSTANCE;
    }

    void CDetectionPipeline::release()
    {
        if ( CDetectionPipeline::INSTANCE != NULL )
        {
            delete CDetectionPipeline::INSTANCE;
            CDetectionPipeline::INSTANCE = NULL;
        }
    }

    CDetectionPipeline::~CDetectionPipeline()
    {
        
    }



    void CDetectionPipeline::run( const cv::Mat& frame )
    {
        m_currentFrame = frame;

        // TODO: Single call to all steps
    }

    void CDetectionPipeline::stepMaskCreation( const cv::Mat& frame, cv::Mat& mask )
    {
        // cv::Mat _frameCSpace;
        // cv::Mat _eqChannel;
        // cv::Mat _channel;
        // cv::Mat _threshed;

        // cv::cvtColor( frame, _frameCSpace, cv::COLOR_BGR2YCrCb );

        // cv::extractChannel( _frameCSpace, _channel, 0 );
        // cv::equalizeHist( _channel, _eqChannel );
        // cv::insertChannel( _eqChannel, _frameCSpace ,0 );

        // cv::inRange( _frameCSpace, m_cspaceMin, m_cspaceMax, _threshed );

        // cv::erode( _threshed, mask, m_morphElement );
        // cv::dilate( mask, mask, m_morphElement );

        cv::Mat _grayScale;
        cv::cvtColor( frame, _grayScale, CV_RGB2GRAY );
        adaptiveThreshold( _grayScale, mask, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 41, 15 );
    }

    void CDetectionPipeline::stepMaskedCreation( const cv::Mat& frame, const cv::Mat& mask, cv::Mat& masked )
    {
        cv::Mat _gray;

        cv::cvtColor( frame, _gray, cv::COLOR_BGR2GRAY );
        cv::bitwise_and( _gray, mask, masked );
    }

    void CDetectionPipeline::stepEdgesCreation( const cv::Mat& masked, cv::Mat& edges,
                                                int blurSize, int cannyMin, int cannyMax, int cannyMaskSize )
    {
        cv::Mat _iEdges;

        cv::blur( masked, _iEdges, cv::Size( blurSize,
                                             blurSize ) );

        cv::Canny( _iEdges, edges, 
                   cannyMin,
                   cannyMax,
                   cannyMaskSize );
    }

    void CDetectionPipeline::stepFindEllipses( const cv::Mat& edges, 
                                               cv::Mat& matContours,
                                               CTypeContours& contours, 
                                               CTypeHierarchy& hierarchy,
                                               vector< cv::RotatedRect >& ellipsesBOB,
                                               int ellipsesCountThrehsold )
    {
        cv::findContours( edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE );

        for ( CTypeContour contour : contours )
        {
            if ( contour.size() > ellipsesCountThrehsold )
            {
                ellipsesBOB.push_back( cv::fitEllipse( cv::Mat( contour ) ) );
            }
        }

        matContours = edges.clone();

        for ( int q = 0; q < contours.size(); q++ )
        {
            cv::drawContours( matContours, contours, q, cv::Scalar( 255, 255, 255 ), 2 );
        }
    }

    void CDetectionPipeline::stepProcessEllipses( const vector< cv::RotatedRect>& inEllipses,
                                                  vector< cv::RotatedRect>& outEllipses,
                                                  const CProcessingParams& params )
    {
        for ( int q = 0; q < inEllipses.size(); q++ )
        {
            float _a = inEllipses[q].size.width;
            float _b = inEllipses[q].size.height;
            float _size = sqrt( _a * _a + _b * _b );
            float _ratio = _a / _b;

            if ( ( params.minSize < _size && _size < params.maxSize ) &&
                 ( params.minRatio < _ratio && _ratio < params.maxRatio ) )
            {
                cout << "size: " << _size << endl;
                cout << "ratio: " << _ratio << endl;
                outEllipses.push_back( inEllipses[q] );
            }

        }
    }

    void CDetectionPipeline::stepBlendResults( const cv::Mat& frame,
                                               cv::Mat& result,
                                               const vector< cv::RotatedRect >& ellipses )
    {
        result = frame.clone();

        for ( cv::RotatedRect ellipRect : ellipses )
        {
            cv::ellipse( result, ellipRect, cv::Scalar( 255, 0, 0 ), 2 );
        }
    }

}}