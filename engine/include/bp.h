#include <SVideoHandler.h>

#include <../include/opencv2/opencv.hpp>
#include <../include/opencv2/tracking.hpp>
#include <../include/opencv2/core/ocl.hpp>


#define WINDOW_ORIGINAL_FRAME "wOriginalFrame"
#define SAMPLE_TIME 10 // 30fps
#define CAMERA_DEVICE_ID 0
#define VIDEO_TEST "../res/calibration_ps3eyecam.avi"
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

bool g_trackingStarted = false;
cv::Point2f middle;
float minDistance[3];

bool cmp( cv::KeyPoint a, cv::KeyPoint b )
{
    float aCmp = abs( a.pt.x - middle.x ) + abs( a.pt.y - middle.y );
    float bCmp = abs( b.pt.x - middle.x ) + abs( b.pt.y - middle.y );
    if(aCmp == bCmp)
    {

    }
    return aCmp > bCmp;
}

bool cmp2( cv::KeyPoint a, cv::KeyPoint b )
{
    return a.pt.y < b.pt.y;
}

bool sort3( cv::KeyPoint &a, cv::KeyPoint &b, cv::KeyPoint &c )
{
    if ( a.pt.x > b.pt.x )
        swap( a, b );
    if ( a.pt.x > c.pt.x )
        swap( a, c );
    if ( b.pt.x > c.pt.x )
        swap( b, c );
}

float distanceToLine( cv::Point2f start, cv::Point2f end, cv::Point2f from )
{
    return  abs( (end.y - start.y) * from.x - ( end.x - start.x ) * from.y + end.x * start.y - end.y * start.x ) /
            sqrt( ( end.y - start.y ) * ( end.y - start.y ) + ( end.x - start.x ) * ( end.x - start.x ) );
}

float distanceToPoint( cv::Point2f start, cv::Point2f end )
{
    return  sqrt( ( end.y - start.y ) * ( end.y - start.y ) + ( end.x - start.x ) * ( end.x - start.x ) );
}

float computeBorders( vector< cv::KeyPoint > &keypoints, cv::KeyPoint &st, cv::KeyPoint &en, cv::KeyPoint &minUp, cv::KeyPoint &minDown )
{
    float tmpDist, distUp = 100, distDown = 100;

    for ( cv::KeyPoint keyPt : keypoints )
    {
        if( keyPt.pt != st.pt && keyPt.pt != en.pt )
        {
            tmpDist = distanceToLine( st.pt, en.pt, keyPt.pt );
            if( tmpDist < distUp )
            {
                distDown = distUp;
                minDown = minUp;
                distUp = tmpDist;
                minUp = keyPt;
            }
            else if( tmpDist < distDown )
            {
                distDown = tmpDist;
                minDown = keyPt;
            }
        }
    }

    if( minUp.pt.y > minDown.pt.y )
        swap(minUp, minDown);
}

float computeLines( vector< cv::KeyPoint > &keypoints, cv::KeyPoint &st, cv::KeyPoint &en, cv::KeyPoint &minUp, cv::KeyPoint &minMid, cv::KeyPoint &minDown )
{
    float tmpDist, distUp = 100, distDown = 100, distMid = 100;

    for ( cv::KeyPoint keyPt : keypoints )
    {
        if( keyPt.pt != st.pt && keyPt.pt != en.pt )
        {
            tmpDist = distanceToLine( st.pt, en.pt, keyPt.pt );
            if( tmpDist < distUp )
            {
                distDown = distMid;
                minDown = minMid;
                distMid = distUp;
                minMid = minUp;
                distUp = tmpDist;
                minUp = keyPt;
            }
            else if( tmpDist < distMid )
            {
                distDown = distMid;
                minDown = minMid;
                distMid = tmpDist;
                minMid = keyPt;
            }
            else if( tmpDist < distDown )
            {
                distDown = tmpDist;
                minDown = keyPt;
            }
        }
    }

    sort3( minUp, minMid, minDown);
}

int main()
{

    calibcv::SVideoHandler* _videoHandler = calibcv::SVideoHandler::create();

    if ( !_videoHandler->openVideo( VIDEO_TEST ) )
    {
        cout << "couldn't open video file: " << VIDEO_TEST << endl;
        exit( -1 );
    }

    /*if ( !_videoHandler->openCamera( CAMERA_DEVICE_ID ) )
    {
        cout << "couldn't open webcam with deviceid: " << CAMERA_DEVICE_ID << endl;
        exit( -1 );
    }*/

    cv::namedWindow( WINDOW_ORIGINAL_FRAME );
    cv::namedWindow( "Cropped" );
    cv::Mat _frame, _grayScale, grad_x, grad_y, abs_grad_x, abs_grad_y, mask, result, cropped;
    int scale = 1;
    int delta = 0;
    vector< cv::KeyPoint > keypoints, keypointsCropped;
    vector< cv::KeyPoint > orderedKeypoints(20);
    cout<<"OpenCV version: "<<CV_MINOR_VERSION<<endl;
    cv::Ptr<cv::Tracker> tracker;
    tracker = cv::Tracker::create("KCF");
    cv::Rect2d* table;

    _videoHandler->setPlaybackAtFrameIndex( 5 );
    //_videoHandler->togglePause();

    while( 1 )
    {
        double timer = (double)cv::getTickCount();

        cv::Mat _frame;

        _videoHandler->takeFrame( _frame );

        if ( g_trackingStarted == false && _videoHandler->roi().size() == 4 )
        {
            g_trackingStarted = true;
            table = new cv::Rect2d(_videoHandler->roi()[0], _videoHandler->roi()[2]);
            tracker->init(_frame, *table);
        }

        cv::cvtColor(_frame, _grayScale, CV_RGB2GRAY);// cuda enabled
        cv::adaptiveThreshold(_grayScale, mask,  255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 41, 15);
        // -> from here maybe :o ***************************************
        cv::Scharr( mask, grad_x, CV_16S, 1, 0, scale, delta, cv::BORDER_DEFAULT );// cuda enabled
        cv::convertScaleAbs( grad_x, abs_grad_x );// cuda enabled

        cv::Scharr( mask, grad_y, CV_16S, 0, 1, scale, delta, cv::BORDER_DEFAULT );// cuda enabled
        cv::convertScaleAbs( grad_y, abs_grad_y );// cuda enabled

        cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, mask );// cuda enabled
        // *************************************************************

        cv::SimpleBlobDetector::Params params;
        params.filterByArea = true;
        params.minArea = 200;
        params.maxArea = 1000;
        params.filterByColor = true;
        params.blobColor = 0;
        params.filterByConvexity = true;
        params.minConvexity = 0.9;
        params.maxConvexity = 1;

        cv::Ptr< cv::SimpleBlobDetector > blobsDetector = cv::SimpleBlobDetector::create( params );
        blobsDetector->detect(mask, keypoints);

        /*for ( cv::KeyPoint ellipRect : keypoints )
        {
            cv::circle(_frame, ellipRect.pt, 4, cv::Scalar(255, 0, 255), -1);
            //cv::ellipse( result, ellipRect, cv::Scalar( 255, 0, 0 ), 2 );
        }*/
        //if ( g_trackingStarted )
        if ( false )
        {
            for( cv::KeyPoint keyPt : keypoints )
            {
                /*if( keyPt.pt.x > min( orderedKeypoints[0].pt.x, orderedKeypoints[15].pt.x ) &&
                    keyPt.pt.x < max( orderedKeypoints[4].pt.x, orderedKeypoints[19].pt.x ) &&
                    keyPt.pt.y > min( orderedKeypoints[0].pt.y, orderedKeypoints[4].pt.y ) &&
                    keyPt.pt.y < max( orderedKeypoints[15].pt.y, orderedKeypoints[19].pt.y ) )*/
                {
                    for( cv::KeyPoint &prevKey : orderedKeypoints )
                    {
                        if( distanceToPoint( prevKey.pt, keyPt.pt ) < 25 )
                        {
                            prevKey.pt = keyPt.pt;
                        }
                    }
                }
            }
            for( int i = 0; i < orderedKeypoints.size(); i++ )
            {
                cv::putText( _frame, SSTR( int(i) ), orderedKeypoints[i].pt, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,255), 2 );
            }
        }

        //if ( g_trackingStarted == false && keypoints.size() == 20 )
        if ( keypoints.size() == 20 )
        {
            g_trackingStarted = true;
            float sumX = 0, sumY = 0;
            for ( cv::KeyPoint keyPt : keypoints )
            {
                sumX += keyPt.pt.x;
                sumY += keyPt.pt.y;
            }
            middle.x = sumX / 20;
            middle.y = sumY / 20;
            //cv::circle(_frame, middle, 4, cv::Scalar(0, 0, 255), -1);

            sort( keypoints.begin(), keypoints.end(), cmp );
            sort( keypoints.begin(), keypoints.begin() + 4, cmp2 );
            if(keypoints[0].pt.x > keypoints[1].pt.x)
                swap( keypoints[0], keypoints[1] );
            if(keypoints[2].pt.x > keypoints[3].pt.x)
                swap( keypoints[2], keypoints[3] );
            /*cv::circle(_frame, keypoints[0].pt, 4, cv::Scalar(0, 255, 0), -1);
            cv::circle(_frame, keypoints[1].pt, 4, cv::Scalar(0, 255, 0), -1);
            cv::circle(_frame, keypoints[2].pt, 4, cv::Scalar(0, 255, 0), -1);
            cv::circle(_frame, keypoints[3].pt, 4, cv::Scalar(0, 255, 0), -1);*/
            orderedKeypoints[0] = keypoints[0];
            orderedKeypoints[4] = keypoints[1];
            orderedKeypoints[15] = keypoints[2];
            orderedKeypoints[19] = keypoints[3];

            computeBorders( keypoints, keypoints[0], keypoints[2], orderedKeypoints[5], orderedKeypoints[10] );
            computeBorders( keypoints, keypoints[1], keypoints[3], orderedKeypoints[9], orderedKeypoints[14] );

            computeLines( keypoints, orderedKeypoints[0], orderedKeypoints[4], orderedKeypoints[1], orderedKeypoints[2], orderedKeypoints[3] );
            computeLines( keypoints, orderedKeypoints[5], orderedKeypoints[9], orderedKeypoints[6], orderedKeypoints[7], orderedKeypoints[8] );
            computeLines( keypoints, orderedKeypoints[10], orderedKeypoints[14], orderedKeypoints[11], orderedKeypoints[12], orderedKeypoints[13] );
            computeLines( keypoints, orderedKeypoints[15], orderedKeypoints[19], orderedKeypoints[16], orderedKeypoints[17], orderedKeypoints[18] );

            //cv::circle(_frame, orderedKeypoints[17].pt, 4, cv::Scalar(255, 0, 0), -1);
            //cv::circle(_frame, keypoints[2].pt, 4, cv::Scalar(0, 255, 0), -1);
            cv::Rect2d roi( cv::Point(  max( min( orderedKeypoints[0].pt.x, orderedKeypoints[15].pt.x ) - 30, 0.0f ),
                                        max( min( orderedKeypoints[0].pt.y, orderedKeypoints[4].pt.y ) - 30, 0.0f ) ),
                            cv::Point(  min( max( orderedKeypoints[4].pt.x, orderedKeypoints[19].pt.x ) + 30, (float) _frame.cols ),
                                        min( max( orderedKeypoints[15].pt.y, orderedKeypoints[19].pt.y ) + 30, (float) _frame.rows ) ) );

            cropped = cv::clone(_frame(roi));

            //orderedKeypoints[]
            for( int i = 0; i < orderedKeypoints.size(); i++ )
            {
                cv::putText( _frame, SSTR( int(i) ), orderedKeypoints[i].pt, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,255), 2 );
            }

            cv::cvtColor(cropped, _grayScale, CV_RGB2GRAY);// cuda enabled
            cv::adaptiveThreshold(_grayScale, mask,  255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 41, 15);
            // -> from here maybe :o ***************************************
            cv::Scharr( mask, grad_x, CV_16S, 1, 0, scale, delta, cv::BORDER_DEFAULT );// cuda enabled
            cv::convertScaleAbs( grad_x, abs_grad_x );// cuda enabled

            cv::Scharr( mask, grad_y, CV_16S, 0, 1, scale, delta, cv::BORDER_DEFAULT );// cuda enabled
            cv::convertScaleAbs( grad_y, abs_grad_y );// cuda enabled

            cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, mask );// cuda enabled
            // *************************************************************

            cv::SimpleBlobDetector::Params params;
            params.filterByArea = true;
            params.minArea = 200;
            params.maxArea = 1000;
            params.filterByColor = true;
            params.blobColor = 0;
            params.filterByConvexity = true;
            params.minConvexity = 0.9;
            params.maxConvexity = 1;

            cv::Ptr< cv::SimpleBlobDetector > blobsDetector2 = cv::SimpleBlobDetector::create( params );
            blobsDetector2->detect(mask, keypointsCropped);

            for ( cv::KeyPoint ellipRect : keypointsCropped )
            {
                cv::circle(cropped, ellipRect.pt, 4, cv::Scalar(255, 0, 255), -1);
                //cv::ellipse( result, ellipRect, cv::Scalar( 255, 0, 0 ), 2 );
            }
            cv::imshow( "Cropped", cropped );
        }

        /*if ( g_trackingStarted )
        {
            bool ok = tracker->update(_frame, *table);
            cv::rectangle(_frame, *table, cv::Scalar( 255, 0, 0 ), 2, 1 );
        }*/


        float fps = cv::getTickFrequency() / ((double)cv::getTickCount() - timer);

        cv::putText(_frame, "FPS : " + SSTR(int(fps)), cv::Point(100,50), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50), 2);

        cv::imshow( WINDOW_ORIGINAL_FRAME, _frame );

        int _key = cv::waitKey( SAMPLE_TIME ) & 255;
        // std::cout << "key: " << _key << std::endl;

        if ( _key == KEY_SPACE )
        {
            _videoHandler->togglePause();
        }
        else if ( _key == KEY_ENTER )
        {
            // std::cout << "picking!" << std::endl;
            _videoHandler->togglePickingROI();
        }
        else if ( _key == KEY_ESCAPE )
        {
            break;
        }
        // _videoHandler->roi();

    }

    calibcv::SVideoHandler::release();
    _videoHandler = NULL;

    return 0;

}