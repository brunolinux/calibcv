#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
cv::Mat frame, _grayScale, _threshold, _blur, _canny, _contours;

float PIPELINE_MASKING_STAGE_MAX_VALUE, PIPELINE_MASKING_STAGE_C, block_sz, min_canny, max_canny, canny_win_size;
float PIPELINE_ELLIPSE_MIN_SIZE, PIPELINE_ELLIPSE_MAX_SIZE, PIPELINE_ELLIPSE_MIN_RATIO = 10, PIPELINE_ELLIPSE_MAX_RATIO;
int currentFrame;
bool isPaused = false;

cv::VideoCapture cap("../../res/fronto-parallel.avi");

enum types
{
    MASKING_MAX,
    MASKING_C,
    MASKING_BLOCK_SIZE,
    EDGES_MIN_CANNY,
    EDGES_BLOCK_SIZE,
    ELLIPSES_MIN_SIZE,
    ELLIPSES_MAX_SIZE,
    ELLIPSES_MIN_RATIO,
    FRAME
};

struct masking_data
{
    int PIPELINE_MASKING_STAGE_MAX_VALUE;
    int PIPELINE_MASKING_STAGE_C;
    int block_sz = 3;
};

struct edges_data
{
    int min_canny = 50;
    int canny_win_size = 3;
};

struct ellipses_data
{
    int PIPELINE_ELLIPSE_MIN_SIZE;
    int PIPELINE_ELLIPSE_MAX_SIZE;
    float PIPELINE_ELLIPSE_MIN_RATIO;
    float PIPELINE_ELLIPSE_MAX_RATIO;
};

struct masking_data maskingData;
struct edges_data edgesData;
struct ellipses_data ellipsesData;
void onTrackbarCallback( int value, void* ptr )
{
    types* type = reinterpret_cast<types*>(ptr);
    switch ( *type )
    {
        case MASKING_MAX:
            maskingData.PIPELINE_MASKING_STAGE_MAX_VALUE = value;
            break;
        case MASKING_C:
            maskingData.PIPELINE_MASKING_STAGE_C = value;
            break;
        case MASKING_BLOCK_SIZE:
            maskingData.block_sz = value * 2 + 1;
            break;
        case EDGES_MIN_CANNY:
            edgesData.min_canny = value;
            break;
        case EDGES_BLOCK_SIZE:
            edgesData.canny_win_size = value * 2 + 1;
            break;
        case ELLIPSES_MIN_SIZE:
            ellipsesData.PIPELINE_ELLIPSE_MIN_SIZE = value;
            break;
        case ELLIPSES_MAX_SIZE:
            ellipsesData.PIPELINE_ELLIPSE_MAX_SIZE = value;
            break;
        case ELLIPSES_MIN_RATIO:
            ellipsesData.PIPELINE_ELLIPSE_MIN_RATIO = (float)value / 100;
            ellipsesData.PIPELINE_ELLIPSE_MAX_RATIO = 2 - ellipsesData.PIPELINE_ELLIPSE_MIN_RATIO;
            break;
        case FRAME:
            cap.set( CV_CAP_PROP_POS_FRAMES, value );
            cv::Mat _mat;
            cap.read( _mat );
            cv::imshow( "fronto-parallel", _mat );

            currentFrame = value;
            cap.set( CV_CAP_PROP_POS_FRAMES, currentFrame );
            break;
    }
}

//

int main()
{
    freopen("in.yaml", "r", stdin);


    //41-26-26-26-0-1-25-55-17

    if( !cap.isOpened() )
    {
        cerr<<"error when opening video!"<<endl;
        return 0;
    }

    cv::namedWindow( "fronto-parallel", 1 );
    cv::createTrackbar( "Frame", "fronto-parallel",
                        NULL,
                        1000,
                        onTrackbarCallback,
                        new types(FRAME) );

    cv::createTrackbar( "masking max-value", "fronto-parallel",
                        NULL,
                        255,
                        onTrackbarCallback,
                        new types(MASKING_MAX) );
    cv::createTrackbar( "masking c", "fronto-parallel",
                        NULL,
                        100,
                        onTrackbarCallback,
                        new types(MASKING_C) );
    cv::createTrackbar( "masking block_size", "fronto-parallel",
                        NULL,
                        30,
                        onTrackbarCallback,
                        new types(MASKING_BLOCK_SIZE) );

    cv::createTrackbar( "edges min_canny", "fronto-parallel",
                        NULL,
                        100,
                        onTrackbarCallback,
                        new types(EDGES_MIN_CANNY)  );
    cv::createTrackbar( "edges canny_block_sz", "fronto-parallel",
                        NULL,
                        3,
                        onTrackbarCallback,
                        new types(EDGES_BLOCK_SIZE) );

    cv::createTrackbar( "ellipse min_size", "fronto-parallel",
                        NULL,
                        200,
                        onTrackbarCallback,
                        new types(ELLIPSES_MIN_SIZE) );
    cv::createTrackbar( "ellipse max_size", "fronto-parallel",
                        NULL,
                        200,
                        onTrackbarCallback,
                        new types(ELLIPSES_MAX_SIZE) );
    cv::createTrackbar( "ellipse min_ratio", "fronto-parallel",
                        NULL,
                        100,
                        onTrackbarCallback,
                        new types(ELLIPSES_MIN_RATIO) );


    int totalFrames = cap.get( CV_CAP_PROP_FRAME_COUNT );
    cv::setTrackbarMax( "Frame", "fronto-parallel", totalFrames );
    string msg;
    cin>>msg>>maskingData.PIPELINE_MASKING_STAGE_MAX_VALUE;
    cin>>msg>>maskingData.PIPELINE_MASKING_STAGE_C;
    cin>>msg>>maskingData.block_sz;
    cin>>msg>>edgesData.min_canny;
    cin>>msg>>max_canny;
    cin>>msg>>edgesData.canny_win_size;
    cin>>msg>>ellipsesData.PIPELINE_ELLIPSE_MIN_SIZE;
    cin>>msg>>ellipsesData.PIPELINE_ELLIPSE_MAX_SIZE;
    cin>>msg>>ellipsesData.PIPELINE_ELLIPSE_MIN_RATIO;
    cin>>msg>>ellipsesData.PIPELINE_ELLIPSE_MAX_RATIO;

    cv::setTrackbarPos( "masking max-value", "fronto-parallel", maskingData.PIPELINE_MASKING_STAGE_MAX_VALUE );
    cv::setTrackbarPos( "masking c", "fronto-parallel", maskingData.PIPELINE_MASKING_STAGE_C );
    cv::setTrackbarPos( "masking block_size", "fronto-parallel", maskingData.block_sz / 2 );
    cv::setTrackbarPos( "edges min_canny", "fronto-parallel", edgesData.min_canny );
    cv::setTrackbarPos( "edges canny_block_sz", "fronto-parallel", edgesData.canny_win_size / 2 );
    cv::setTrackbarPos( "ellipse min_size", "fronto-parallel", ellipsesData.PIPELINE_ELLIPSE_MIN_SIZE );
    cv::setTrackbarPos( "ellipse max_size", "fronto-parallel", ellipsesData.PIPELINE_ELLIPSE_MAX_SIZE );
    cv::setTrackbarPos( "ellipse min_ratio", "fronto-parallel", ellipsesData.PIPELINE_ELLIPSE_MIN_RATIO * 100 );

    while(1)
    {
        cap>>frame;
        if( frame.empty() )
            break;

        int key = cv::waitKey(5);
        if ( key == ' ' )
        {
            isPaused = !isPaused;
        }
        if ( key == 27 )
        {
            freopen("in.yaml", "w", stdout);
            cout<<"MASKING_MAX_VALUE: "<<maskingData.PIPELINE_MASKING_STAGE_MAX_VALUE<<endl;
            cout<<"MASKING_C: "<<maskingData.PIPELINE_MASKING_STAGE_C<<endl;
            cout<<"MASKING_BLOCK_SIZE: "<<maskingData.block_sz<<endl;
            cout<<"EDGES_MIN_CANNY: "<<edgesData.min_canny<<endl;
            cout<<"EDGES_MAX_CANNY: "<<edgesData.min_canny*3<<endl;
            cout<<"EDGES_BLOCK_SIZE: "<<edgesData.canny_win_size<<endl;
            cout<<"ELLIPSES_MIN_SIZE: "<<ellipsesData.PIPELINE_ELLIPSE_MIN_SIZE<<endl;
            cout<<"ELLIPSES_MAX_SIZE: "<<ellipsesData.PIPELINE_ELLIPSE_MAX_SIZE<<endl;
            cout<<"ELLIPSES_MIN_RATIO: "<<ellipsesData.PIPELINE_ELLIPSE_MIN_RATIO<<endl;
            cout<<"ELLIPSES_MAX_RATIO: "<<ellipsesData.PIPELINE_ELLIPSE_MAX_RATIO<<endl;
            break;
        }
        //255-26-30-0-2-25-55-80
        if(isPaused)
        {
            cap.read( frame );
            cap.set( CV_CAP_PROP_POS_FRAMES, currentFrame );
        }
        else
        {
            cap.read( frame );
            currentFrame++;
            cv::setTrackbarPos( "Frame", "fronto-parallel", currentFrame );
        }

        cv::cvtColor( frame, _grayScale, CV_RGB2GRAY );
        cv::adaptiveThreshold( _grayScale, _threshold, maskingData.PIPELINE_MASKING_STAGE_MAX_VALUE,
                               cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV,
                               maskingData.block_sz, maskingData.PIPELINE_MASKING_STAGE_C );

        cv::blur( _threshold, _blur, cv::Size( 3, 3 ) );
        cv::Canny( _blur, _canny, edgesData.min_canny, edgesData.min_canny * 3, edgesData.canny_win_size );



        vector< vector< cv::Point > > _contours;
        vector< cv::Vec4i > _hierarchy;
        vector< cv::RotatedRect > _ellipsesBB;

        cv::findContours( _canny, _contours, _hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE );

        for ( auto _contour : _contours )
        {
            if ( _contour.size() > 10 )
            {
                _ellipsesBB.push_back( cv::fitEllipse( cv::Mat( _contour ) ) );
            }
        }

        vector< cv::RotatedRect > _filteredEllipses;

        for ( int q = 0; q < _ellipsesBB.size(); q++ )
        {
            float _a = _ellipsesBB[q].size.width;
            float _b = _ellipsesBB[q].size.height;
            float _size = sqrt( _a * _a + _b * _b );
            float _ratio = _a / _b;

            if ( ( ellipsesData.PIPELINE_ELLIPSE_MIN_SIZE < _size && _size < ellipsesData.PIPELINE_ELLIPSE_MAX_SIZE ) &&
                 ( ellipsesData.PIPELINE_ELLIPSE_MIN_RATIO < _ratio && _ratio < ellipsesData.PIPELINE_ELLIPSE_MAX_RATIO ) )
            {
                _filteredEllipses.push_back( _ellipsesBB[q] );
            }

        }
        if( _filteredEllipses.size() != 80 )
        {

            cerr<<"Filered Ellipses: "<<_filteredEllipses.size()<<" at frame "<<currentFrame<<endl;
        }

        vector< cv::Point2f > _candidatePoints;
        for ( cv::RotatedRect _rect : _filteredEllipses )
        {
            cv::ellipse( frame, _rect, cv::Scalar( 255, 0, 0 ), 2 );
        }

        cv::imshow( "fronto-parallel", frame );

    }
    cv::destroyAllWindows();
}
