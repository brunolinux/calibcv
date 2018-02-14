
#include "CDetectionPipeline.h"


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

    void CDetectionPipeline::stepMaskCreation( const cv::Mat& frame, cv::Mat& mask, vector< cv::KeyPoint >& keypoints )
    {
        cv::Mat _grayScale, grad_x, grad_y, abs_grad_x, abs_grad_y;
        int scale = 1;
        int delta = 0;

        cv::cvtColor(frame, _grayScale, CV_RGB2GRAY);
        cv::adaptiveThreshold(_grayScale, mask, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 41, 15);

        cv::Scharr( mask, grad_x, CV_16S, 1, 0, scale, delta, cv::BORDER_DEFAULT );
        cv::convertScaleAbs( grad_x, abs_grad_x );

        cv::Scharr( mask, grad_y, CV_16S, 0, 1, scale, delta, cv::BORDER_DEFAULT );
        cv::convertScaleAbs( grad_y, abs_grad_y );

        cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, mask );

        cv::SimpleBlobDetector::Params params;
        params.filterByArea = true;
        params.minArea = 200;
        params.maxArea = 2500;
        params.filterByColor = true;
        params.blobColor = 0;
        params.filterByInertia = false;
        params.minInertiaRatio = 0.6;
        params.maxInertiaRatio = 2;

        cv::Ptr< cv::SimpleBlobDetector > blobsDetector = cv::SimpleBlobDetector::create( params );
        blobsDetector->detect(mask, keypoints);

        /*cv::Mat _frameCSpace;
        cv::Mat _eqChannel;
        cv::Mat _channel;
        cv::Mat _threshed;

        cv::cvtColor( frame, _frameCSpace, cv::COLOR_BGR2YCrCb );

        cv::extractChannel( _frameCSpace, _channel, 0 );
        cv::equalizeHist( _channel, _eqChannel );
        cv::insertChannel( _eqChannel, _frameCSpace ,0 );

        cv::inRange( _frameCSpace, m_cspaceMin, m_cspaceMax, _threshed );

        cv::erode( _threshed, mask, m_morphElement );
        cv::dilate( mask, mask, m_morphElement );*/
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
                                               vector< cv::KeyPoint >& keypoints,
                                               int ellipsesCountThrehsold )
    {
        cv::findContours( edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE );



        /*for ( CTypeContour contour : contours )
        {
            if ( contour.size() > ellipsesCountThrehsold )
            {
                ellipsesBOB.push_back( cv::fitEllipse( cv::Mat( contour ) ) );
            }
        }*/

        matContours = edges.clone();

        for ( int q = 0; q < contours.size(); q++ )
        {
            cv::drawContours( matContours, contours, q, cv::Scalar( 255, 255, 255 ), 2 );
        }
    }

    void CDetectionPipeline::stepProcessEllipses( const vector< cv::RotatedRect>& inEllipses,
                                                  vector< cv::RotatedRect>& outEllipses )
    {

    }

    void CDetectionPipeline::stepBlendResults( const cv::Mat& frame,
                                               cv::Mat& result,
                                               const vector< cv::KeyPoint >& ellipses )
    {
        result = frame.clone();

        for ( cv::KeyPoint ellipRect : ellipses )
        {
            cv::circle(result, ellipRect.pt, 4, cv::Scalar(255, 0, 255), -1);
            //cv::ellipse( result, ellipRect, cv::Scalar( 255, 0, 0 ), 2 );
        }
    }

}}
