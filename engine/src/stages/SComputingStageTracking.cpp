

#include <stages/SComputingStageTracking.h>
#include <stages/SComputingStageFeatures.h>

using namespace std;

namespace calibcv
{

    float dist( cv::Point2f start, cv::Point2f end )
    {
        return  sqrt( ( end.y - start.y ) * ( end.y - start.y ) + ( end.x - start.x ) * ( end.x - start.x ) );
    }

    SComputingStageTracking::SComputingStageTracking()
        : SComputingStage()
    {
        m_trackingPoints = vector< STrackingPoint >( 20 );
    }

    SComputingStageTracking::~SComputingStageTracking()
    {

    }

    void SComputingStageTracking::_run( const cv::Mat& input )
    {
        assert( false );// should not call this
    }

    void SComputingStageTracking::_run( SComputingStage* parent )
    {
        // Compute tracking here ****

        auto _keypoints = reinterpret_cast< SComputingStageFeatures* >( parent )->getKeypoints();

        if ( _keypoints.size() == 20 )
        {
            for ( cv::KeyPoint _newKeypoint : _keypoints )
            {
                for ( STrackingPoint& _oldTrackingPoint : m_trackingPoints  )
                {
                    if ( dist( _oldTrackingPoint.pos, _newKeypoint.pt + m_cropOrigin ) < 25 )
                    {
                        _oldTrackingPoint.vel = _newKeypoint.pt + m_cropOrigin - _oldTrackingPoint.pos;
                        _oldTrackingPoint.pos = _newKeypoint.pt + m_cropOrigin;
                    }
                }
            }
        }
        else
        {
            // TODO: Add estimation step

        }

        // **************************

        m_stageResult = m_originalFrame.clone();

        cv::rectangle( m_stageResult, m_cropROI, cv::Scalar( 255, 0, 0 ), 2 );

        for( int i = 0; i < m_trackingPoints.size(); i++ )
        {
            cv::putText( m_stageResult, to_string( i ), m_trackingPoints[i].pos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 255,0,255 ), 2 );
        }

        cv::line( m_stageResult, m_trackingPoints[0].pos, m_trackingPoints[4].pos, cv::Scalar( 255, 255, 0 ), 1 );
        cv::line( m_stageResult, m_trackingPoints[4].pos, m_trackingPoints[5].pos, cv::Scalar( 255, 255, 0 ), 1 );
        cv::line( m_stageResult, m_trackingPoints[5].pos, m_trackingPoints[9].pos, cv::Scalar( 255, 255, 0 ), 1 );
        cv::line( m_stageResult, m_trackingPoints[9].pos, m_trackingPoints[10].pos, cv::Scalar( 255, 255, 0 ), 1 );
        cv::line( m_stageResult, m_trackingPoints[10].pos, m_trackingPoints[14].pos, cv::Scalar( 255, 255, 0 ), 1 );
        cv::line( m_stageResult, m_trackingPoints[14].pos, m_trackingPoints[15].pos, cv::Scalar( 255, 255, 0 ), 1 );
        cv::line( m_stageResult, m_trackingPoints[15].pos, m_trackingPoints[19].pos, cv::Scalar( 255, 255, 0 ), 1 );
    }

    void SComputingStageTracking::initialize( vector< cv::KeyPoint > positions )
    {
        for ( int q = 0; q < positions.size(); q++ )
        {
            STrackingPoint _tpoint;
            _tpoint.pos = positions[q].pt;
            _tpoint.vel = cv::Point2f( 0, 0 );

            m_trackingPoints[q] = _tpoint;
        }
    }

    void SComputingStageTracking::getCroppedByWindow( const cv::Mat& src, cv::Mat& dst )
    {
        m_cropOrigin.x = max( min( m_trackingPoints[0].pos.x, m_trackingPoints[15].pos.x ) - 50, 0.0f );
        m_cropOrigin.y = max( min( m_trackingPoints[0].pos.y, m_trackingPoints[4].pos.y ) - 50, 0.0f );

        m_cropROI = cv::Rect2f( cv::Point(  max( min( m_trackingPoints[0].pos.x, m_trackingPoints[15].pos.x ) - 50, 0.0f ),
                                            max( min( m_trackingPoints[0].pos.y, m_trackingPoints[4].pos.y ) - 50, 0.0f ) ),
                                cv::Point(  min( max( m_trackingPoints[4].pos.x, m_trackingPoints[19].pos.x ) + 50, (float) src.cols ),
                                            min( max( m_trackingPoints[15].pos.y, m_trackingPoints[19].pos.y ) + 50, (float) src.rows ) ) );

        dst = src( m_cropROI ).clone();
    }

}