

#include <stages/SComputingStageTracking.h>
#include <stages/SComputingStageFeatures.h>

using namespace std;

namespace calibcv
{

    float dist( cv::Point2f start, cv::Point2f end )
    {
        float _dx = end.x - start.x;
        float _dy = end.y - start.y;

        return _dx * _dx + _dy * _dy;
    }

    SComputingStageTracking::SComputingStageTracking()
        : SComputingStage()
    {
        m_trackingPoints = vector< STrackingPoint >( 20 );
        cv::namedWindow( "perspective" );
    }

    SComputingStageTracking::~SComputingStageTracking()
    {

    }

    void SComputingStageTracking::grabParamsFromParent( SComputingStage* parent )
    {
        m_detectedKeypoints = reinterpret_cast< SComputingStageFeatures* >( parent )->getKeypoints();
    }

    void SComputingStageTracking::_estimateLostSingle( int indx )
    {

    }

    bool comparatorDist( cv::KeyPoint a, cv::KeyPoint b )
    {
        float _dx1 = a.pt.x - s_detectionPoint.x;
        float _dy1 = a.pt.y - s_detectionPoint.y;

        float _dx2 = b.pt.x - s_detectionPoint.x;
        float _dy2 = b.pt.y - s_detectionPoint.y;

        float _dist1 = _dx1 * _dx1 + _dy1 * _dy1;
        float _dist2 = _dx2 * _dx2 + _dy2 * _dy2;

        return _dist1 > _dist2;
    }

    void SComputingStageTracking::_estimateLostPattern()
    {
        // compensate with odometry

        for ( int q = 0; q < m_trackingPoints.size(); q++ )
        {
            if ( m_trackingPoints[q].found )
            {
                continue;
            }

            m_trackingPoints[q].pos += m_trackingPoints[q].vel;
        }


        for ( int i = -2; i <= 2; i++ )
        {
            for ( int j = -2; j <= 2; j++ )
            {

            }
        }






        // find 4 points for perspective trasnform

        for ( int q = 0; q <  m_trackingPoints.size(); q++ )
        {
            if ( m_trackingPoints[q].found )
            {
                continue;
            }

            _estimateLostSingle( q );
        }
    }

    void SComputingStageTracking::_run( const cv::Mat& input )
    {
        // Compute tracking here ****
        vector< bool > _assigned( m_detectedKeypoints.size() );

        int _matched = 0;

        for ( STrackingPoint& _trackingPoint : m_trackingPoints  )
        {
            for ( int q = 0; q < m_detectedKeypoints.size(); q++ )
            {
                cv::KeyPoint& _newKeypoint = m_detectedKeypoints[q];

                if ( !_assigned[q] && dist( _trackingPoint.pos, _newKeypoint.pt + m_cropOrigin ) < 400 )
                {
                    _trackingPoint.vel = _newKeypoint.pt + m_cropOrigin - _trackingPoint.pos;
                    _trackingPoint.pos = _newKeypoint.pt + m_cropOrigin;
                    _trackingPoint.found = true;
                    _assigned[q] = true;
                    _matched++;
                    break;
                }
            }
        }

        if ( _matched < 20 )
        {
            //_estimateLostPattern();
            m_success = false;
            return;
        }
        else
        {
            m_success = true;
        }


        // **************************

        m_stageResult = m_originalFrame.clone();

        cv::rectangle( m_stageResult, m_cropROI, cv::Scalar( 255, 0, 0 ), 2 );

        for( int i = 0; i < m_trackingPoints.size(); i++ )
        {
            cv::putText( m_stageResult, to_string( i ), m_trackingPoints[i].pos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 255,0,255 ), 2 );
            cv::circle( m_stageResult, m_trackingPoints[i].pos, 4, cv::Scalar( 255, 0, 0 ), -1 );
            cv::circle( m_stageResult, m_trackingPoints[i].pos, 25, cv::Scalar( 0, 0, 255 ), 1 );
        }

        cv::line( m_stageResult, m_trackingPoints[0].pos, m_trackingPoints[4].pos, cv::Scalar( 255, 255, 0 ), 1 );
        cv::line( m_stageResult, m_trackingPoints[4].pos, m_trackingPoints[5].pos, cv::Scalar( 255, 255, 0 ), 1 );
        cv::line( m_stageResult, m_trackingPoints[5].pos, m_trackingPoints[9].pos, cv::Scalar( 255, 255, 0 ), 1 );
        cv::line( m_stageResult, m_trackingPoints[9].pos, m_trackingPoints[10].pos, cv::Scalar( 255, 255, 0 ), 1 );
        cv::line( m_stageResult, m_trackingPoints[10].pos, m_trackingPoints[14].pos, cv::Scalar( 255, 255, 0 ), 1 );
        cv::line( m_stageResult, m_trackingPoints[14].pos, m_trackingPoints[15].pos, cv::Scalar( 255, 255, 0 ), 1 );
        cv::line( m_stageResult, m_trackingPoints[15].pos, m_trackingPoints[19].pos, cv::Scalar( 255, 255, 0 ), 1 );

        cv::Point2f _srcPoints[4] = { m_trackingPoints[6].pos, m_trackingPoints[7].pos,
                                     m_trackingPoints[12].pos, m_trackingPoints[11].pos };
        cv::Point2f _dstPoints[4] = { cv::Point2f( 100, 100 ), cv::Point2f( 200, 100 ),
                                      cv::Point2f( 200, 200 ), cv::Point2f( 100, 200 ) };

        auto _transform = cv::getPerspectiveTransform( _srcPoints, _dstPoints );

        cv::Mat _perspective;
        cv::warpPerspective( m_originalFrame, _perspective, _transform, cv::Size( 400, 300 ) );

        cv::imshow( "perspective", _perspective );
    }

    void SComputingStageTracking::setDetectedKeypoints( const vector< cv::KeyPoint >& keypoints )
    {
        m_detectedKeypoints = keypoints;
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
        vector<cv::Point2f> corners = { m_trackingPoints[0].pos,
                                        m_trackingPoints[4].pos,
                                        m_trackingPoints[15].pos,
                                        m_trackingPoints[19].pos };
        float minX = 1000, maxX = -1000, minY = 1000, maxY = -1000;

        for( cv::Point2f tPoint : corners )
        {
            minX = min( tPoint.x, minX );
            maxX = max( tPoint.x, maxX );
            minY = min( tPoint.y, minY );
            maxY = max( tPoint.y, maxY );
        }
        minX = max( minX - 80, 0.0f );
        minY = max( minY - 80, 0.0f );
        maxX = min( maxX + 80, (float) src.cols );
        maxY = min( maxY + 80, (float) src.rows );

        m_cropOrigin.x = minX;
        m_cropOrigin.y = minY;

        m_cropROI = cv::Rect2f( cv::Point( minX , minY ), cv::Point( maxX, maxY ) );
/*
        m_cropROI = cv::Rect2f( cv::Point(  max( min( m_trackingPoints[0].pos.x, m_trackingPoints[15].pos.x ) - 100, 0.0f ),
                                            max( min( m_trackingPoints[0].pos.y, m_trackingPoints[4].pos.y ) - 100, 0.0f ) ),
                                cv::Point(  min( max( m_trackingPoints[4].pos.x, m_trackingPoints[19].pos.x ) + 100, (float) src.cols ),
                                            min( max( m_trackingPoints[15].pos.y, m_trackingPoints[19].pos.y ) + 100, (float) src.rows ) ) );
*/
        dst = src( m_cropROI ).clone();
    }

    void SComputingStageTracking::reset()
    {
        m_trackingPoints.clear();
    }
}
