
#include <stages/SComputingStageEllipses.h>

using namespace std;

namespace calibcv
{


    SComputingStageEllipses::SComputingStageEllipses()
        : SComputingStage()
    {
    }

    SComputingStageEllipses::~SComputingStageEllipses()
    {
        m_keypoints.clear();
    }

    void SComputingStageEllipses::_run( const cv::Mat& input )
    {
        m_keypoints.clear();
        m_stageResult = m_frame.clone();

        vector< vector< cv::Point > > _contours;
        vector< cv::Vec4i > _hierarchy;
        vector< cv::RotatedRect > _ellipsesBB;

        cv::findContours( input, _contours, _hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE );

        for ( auto _contour : _contours )
        {
            if ( _contour.size() > 5 )
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

            if ( ( ELLIPSE_MIN_SIZE < _size && _size < ELLIPSE_MAX_SIZE ) &&
                 ( ELLIPSE_MIN_RATIO < _ratio && _ratio < ELLIPSE_MAX_RATIO ) )
            {
                _filteredEllipses.push_back( _ellipsesBB[q] );
            }

        }

        for ( cv::RotatedRect _rect : _filteredEllipses )
        {
            m_keypoints.push_back( cv::KeyPoint( _rect.center, 1 ) );
        }

        for ( auto _rect : _filteredEllipses )
        {
            cv::ellipse( m_stageResult, _rect, cv::Scalar( 255, 0, 0 ), 2 );
        }
    }

}
