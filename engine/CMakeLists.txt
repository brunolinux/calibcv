
message( "Configuring pipeline" )

if ( WIN32 )

	include_directories( ${OpenCV_INCLUDE_DIRS} include/ ../ext/felixcam/lib/include )

	set( EngineSources
		 src/SVideoHandler.cpp
		 src/SUtils.cpp
		 src/SComputingStage.cpp
		 src/SComputingPipeline.cpp
		 src/pipelines/SPatternDetectorPipeline.cpp
		 src/panels/SPatternDetectionPanel.cpp
		 src/stages/SComputingStageEdges.cpp
		 src/stages/SComputingStageEllipses.cpp
		 src/stages/SComputingStageFeatures.cpp
		 src/stages/SComputingStageLightCompensation.cpp
		 src/stages/SComputingStageMasking.cpp
		 src/stages/SComputingStageTracking.cpp )

	add_library( calibcvEngine
	             ${EngineSources} )

	target_link_libraries( calibcvEngine
	                       ${OpenCV_LIBS} )

else()

	include_directories( ${OpenCV_INCLUDE_DIRS} include/ ../ext/felixcam/lib/include )

	FILE( GLOB_RECURSE EngineSources src *.cpp )

	add_library( calibcvEngine
	             ${EngineSources} )

	target_link_libraries( calibcvEngine
	                       ${OpenCV_LIBS}
	                       felixcam )

endif()