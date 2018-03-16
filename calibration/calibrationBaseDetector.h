
#pragma once

#include "calibrationCommon.h"

#include <pthread.h>

namespace calibration
{


    class BaseDetector
    {

        protected :

        bool m_hasRefinedPoints;
        pthread_t m_threadHandle;

        BaseDetector( const cv::Size& size );

        public :

        ~BaseDetector();

        virtual bool run() = 0;

        // Shared steps for fronto parallel transformation
        void refineBatch( const vector< cv::Mat >& batchImagesToRefine,
                          const cv::Mat& cameraMatrix,
                          const cv::Mat& distortionCoefficients );
        bool isRefining();
        bool hasRefinationToPick();
        void grabRefinationBatch( vector< cv::Mat >& batchRefinedImages,
                                  vector< CalibrationBucket >& batchBuckets );


        void _frontoConversionDirect();
        void _frontoConversionInverse();

        static void* refinerWorker( void* pDetector );
        static void* detectorWorker( void* pDetector );

        bool hasRefinedPoints() { return m_hasRefinedPoints; }



    };


}