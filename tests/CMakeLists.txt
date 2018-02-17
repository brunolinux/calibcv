
message( "configuring tests" )

include_directories( ../engine/include ${OpenCV_INCLUDE_DIRS} ../ext/felixcam/lib/include )

# TODO: add checks for windows support

#link_directories( ../engine/ )

function( FnBuildTest pSources pExecutableId )

    add_executable( ${pExecutableId}
                    ${pSources} )
    target_link_libraries( ${pExecutableId}
                           calibcvEngine )
    
endfunction()

if( WIN32 )

    set( TestSources
         testPipeline.cpp
         testPipelineVideo.cpp
         testVideoHandler.cpp
         testLightCompensation.cpp )

else()

    FILE( GLOB TestSources *.cpp )

endif()



foreach( testFile ${TestSources} )

    string( REPLACE ".cpp" ""  executableId ${testFile} )
    get_filename_component( execId ${executableId} NAME )

    # message( "fileId: ${testFile}" )
    # message( "fullPathExecutableID: ${executableId}" )
    # message( "executableId: ${execId}")

    FnBuildTest( ${testFile} ${execId} )

endforeach( testFile )

message( "TestSources ${TestSources}" )#