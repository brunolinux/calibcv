
#include <pipelines/SPatternDetectorPipeline.h>
#include <algorithm>

using namespace std;

namespace calibcv
{

    bool comparator1( cv::KeyPoint a, cv::KeyPoint b )
    {
        float aCmp = abs( a.pt.x - s_middle.x ) + abs( a.pt.y - s_middle.y );
        float bCmp = abs( b.pt.x - s_middle.x ) + abs( b.pt.y - s_middle.y );

        return aCmp > bCmp;
    }

    bool comparator2( cv::KeyPoint a, cv::KeyPoint b )
    {
        return a.pt.y < b.pt.y;
    }

    void sort3( cv::KeyPoint &a, cv::KeyPoint &b, cv::KeyPoint &c )
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
        return abs( (end.y - start.y) * from.x - ( end.x - start.x ) * from.y + end.x * start.y - end.y * start.x ) /
                sqrt( ( end.y - start.y ) * ( end.y - start.y ) + ( end.x - start.x ) * ( end.x - start.x ) );
    }

    float distanceToPoint( cv::Point2f start, cv::Point2f end )
    {
        return sqrt( ( end.y - start.y ) * ( end.y - start.y ) + ( end.x - start.x ) * ( end.x - start.x ) );
    }

    void computeBorders( vector< cv::KeyPoint > &keypoints, cv::KeyPoint &st, cv::KeyPoint &en, cv::KeyPoint &minUp, cv::KeyPoint &minDown )
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

    void computeLines( vector< cv::KeyPoint > &keypoints, cv::KeyPoint &st, cv::KeyPoint &en, cv::KeyPoint &minUp, cv::KeyPoint &minMid, cv::KeyPoint &minDown )
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







    SPatternDetectorPipeline::SPatternDetectorPipeline()
        : SComputingPipeline()
    {
        addStage( new SComputingStageMasking() );
        addStage( new SComputingStageEdges() );
        //addStage( new SComputingStageFeatures() );
        addStage( new SComputingStageEllipses() );
        addStage( new SComputingStageTracking() );

        m_isInitializing = true;
    }

    void SPatternDetectorPipeline::_preProcessing( const cv::Mat& input, cv::Mat& output )
    {
        // TODO: Add here implementation for initial region of interest extraction
        SPatternDetectorPanel::INSTANCE->cleanInfo();

        // If tracking, pass the cropped version by the ROI

        reinterpret_cast< SComputingStageTracking* >( m_stages[3] )->getCroppedByWindow( input, output );
    }

    void SPatternDetectorPipeline::_postProcessing( SComputingStage* lastStage )
    {
        // TODO: Add here implementation for final results to be drawn

        SPatternDetectorPanel::INSTANCE->setStageCost( m_stages[0]->getTimeCost() * 1000, MASK );
        SPatternDetectorPanel::INSTANCE->setStageCost( m_stages[1]->getTimeCost() * 1000, EDGES );
        SPatternDetectorPanel::INSTANCE->setStageCost( m_stages[2]->getTimeCost() * 1000, BLOBS );
        SPatternDetectorPanel::INSTANCE->setStageCost( m_stages[3]->getTimeCost() * 1000, TRACKING );

        SPatternDetectorPanel::INSTANCE->showMask( m_stages[0]->getStageResult() );
        SPatternDetectorPanel::INSTANCE->showEdges( m_stages[1]->getStageResult() );
        SPatternDetectorPanel::INSTANCE->showBlobs( m_stages[2]->getStageResult() );
        SPatternDetectorPanel::INSTANCE->showTracking( m_stages[3]->getStageResult() );

        // std::cout << "running pipeline" << std::endl;
    }

    void SPatternDetectorPipeline::_runInitialCalibration( const cv::Mat& input, const SPipelineParams& params )
    {
        SPatternDetectorPanel::INSTANCE->cleanInfo();

        // std::cout << "running initial calibration" << std::endl;
        // Run the pipeline in the ROI
        auto _roi = params.roi;

        for ( int q = 0; q < m_stages.size(); q++ )
        {
            m_stages[q]->begin( input );
        }

        m_stages[0]->run( input );
        m_stages[1]->run( m_stages[0]->getStageResult() );
        m_stages[2]->run( m_stages[1]->getStageResult() );

        SPatternDetectorPanel::INSTANCE->showMask( m_stages[0]->getStageResult() );
        SPatternDetectorPanel::INSTANCE->showEdges( m_stages[1]->getStageResult() );
        SPatternDetectorPanel::INSTANCE->showBlobs( m_stages[2]->getStageResult() );

        SPatternDetectorPanel::INSTANCE->setStageCost( m_stages[0]->getTimeCost() * 1000, MASK );
        SPatternDetectorPanel::INSTANCE->setStageCost( m_stages[1]->getTimeCost() * 1000, EDGES );
        SPatternDetectorPanel::INSTANCE->setStageCost( m_stages[2]->getTimeCost() * 1000, BLOBS );

        auto _keypoints = reinterpret_cast< SComputingStageFeatures* >( m_stages[2] )->getKeypoints();

        int _count = 0;

        if ( _keypoints.size() >= 20 )
        {
            bool _foundPattern = false;
            for ( auto _keypoint : _keypoints )
            {
                if ( _roi.contains( _keypoint.pt ) )
                {
                    _count++;
                    if ( _count == 20 )
                    {
                        _foundPattern = true;
                        break;
                    }
                }
            }

            if ( _foundPattern )
            {
                m_isInitializing = false;
                _computeInitialPattern( _keypoints );
                SPatternDetectorPanel::INSTANCE->setLogInfo( "TRACKING" );
                return;
            }
        }

        SPatternDetectorPanel::INSTANCE->setLogInfo( "FINDING PATTERN" );
    }

    void SPatternDetectorPipeline::_computeInitialPattern( vector< cv::KeyPoint >& keypoints )
    {
        vector< cv::KeyPoint > _ordererKeypoints( 20 );

        float sumX = 0, sumY = 0;
        for ( cv::KeyPoint keyPt : keypoints )
        {
            sumX += keyPt.pt.x;
            sumY += keyPt.pt.y;
        }

        s_middle.x = sumX / 20;
        s_middle.y = sumY / 20;

        sort( keypoints.begin(), keypoints.end(), comparator1 );
        sort( keypoints.begin(), keypoints.begin() + 4, comparator2 );

        if(keypoints[0].pt.x > keypoints[1].pt.x)
            swap( keypoints[0], keypoints[1] );
        if(keypoints[2].pt.x > keypoints[3].pt.x)
            swap( keypoints[2], keypoints[3] );

        _ordererKeypoints[0] = keypoints[0];
        _ordererKeypoints[4] = keypoints[1];
        _ordererKeypoints[15] = keypoints[2];
        _ordererKeypoints[19] = keypoints[3];

        computeBorders( keypoints, keypoints[0], keypoints[2], _ordererKeypoints[5], _ordererKeypoints[10] );
        computeBorders( keypoints, keypoints[1], keypoints[3], _ordererKeypoints[9], _ordererKeypoints[14] );

        computeLines( keypoints, _ordererKeypoints[0], _ordererKeypoints[4], _ordererKeypoints[1], _ordererKeypoints[2], _ordererKeypoints[3] );
        computeLines( keypoints, _ordererKeypoints[5], _ordererKeypoints[9], _ordererKeypoints[6], _ordererKeypoints[7], _ordererKeypoints[8] );
        computeLines( keypoints, _ordererKeypoints[10], _ordererKeypoints[14], _ordererKeypoints[11], _ordererKeypoints[12], _ordererKeypoints[13] );
        computeLines( keypoints, _ordererKeypoints[15], _ordererKeypoints[19], _ordererKeypoints[16], _ordererKeypoints[17], _ordererKeypoints[18] );

        reinterpret_cast< SComputingStageTracking* >( m_stages[3] )->initialize( _ordererKeypoints );
    }



    void SPatternDetectorPipeline::reset()
    {
        cout << "reset!!!" << endl;
        m_isInitializing = true;
    }



}
