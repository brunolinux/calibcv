
#pragma once

#include "calibrationCommon.h"

using namespace std;

#define RANGE_CLOSE 10

namespace utils {


    struct ComparatorKeyPointsCentric
    {

        private :

        float m_x;
        float m_y;

        public :

        bool operator()( const cv::KeyPoint& a, const cv::KeyPoint& b )
        {
            float aCmp = abs( a.pt.x - m_x ) + abs( a.pt.y - m_y );
            float bCmp = abs( b.pt.x - m_x ) + abs( b.pt.y - m_y );

            return aCmp > bCmp;
        }

        void setCenter( float x, float y );
    };

    struct ComparatorKeyPointsY
    {
        public :

        bool operator()( const cv::KeyPoint& a, const cv::KeyPoint& b )
        {
            return a.pt.y > b.pt.y;
        }
    };

    struct ComparatorKeyPointsX
    {
        public :

        bool operator()( const cv::KeyPoint& a, const cv::KeyPoint& b )
        {
            return a.pt.x < b.pt.x;
        }
    };


    bool isGridPatternFit( const vector< cv::Point2f >& candidates,
                           vector< cv::Point2f >& matchedPattern,
                           const cv::Size boardSize,
                           int indxp0, int indxp1,
                           int indxp2, int indxp3 );

    void constructGridPattern( const cv::Point2f& p0,
                               const cv::Point2f& p1,
                               const cv::Point2f& p2,
                               const cv::Point2f& p3,
                               const cv::Size boardSize,
                               vector< cv::Point2f >& vGridPattern );

    float dist( const cv::Point2f& p1, const cv::Point2f& p2 );

    float checkEnd2EndColinearity( const vector< cv::Point2f >& points );

    void computeReprojectionErrors( const cv::Mat& cameraMatrix, const cv::Mat& distortionCoefficients,
                                    const vector< vector< cv::Point3f > >& worldPoints,
                                    const vector< vector< cv::Point2f > >& imagePoints,
                                    const vector< cv::Mat >& rvecs, const vector< cv::Mat >& tvecs,
                                    float& reprojectionError,
                                    vector<float>& perViewErrors );

    void computeColinearityErrors( const cv::Mat& cameraMatrix, const cv::Mat& distortionCoefficients,
                                   const vector< vector< cv::Point3f > >& worldPoints,
                                   const vector< vector< cv::Point2f > >& imagePoints,
                                   const vector< cv::Mat >& rvecs, const vector< cv::Mat >& tvecs,
                                   float& oldColinearity, float& newColinearity );

    void distortPoints( const vector< cv::Point2f >& undistorted,
                        vector< cv::Point2f >& distorted,
                        const cv::Mat& cameraMatrix, const cv::Mat& distortionCoefficients );

}