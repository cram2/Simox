# Locate Gstreamer
# Gstreamer depends on glib-2.0 and libxml2

# start with 'not found'
set(GStreamer_FOUND 0 CACHE BOOL "Do we have all required GStreamer?")

#message(STATUS "GStreamer_HOME ${GStreamer_HOME}")
find_path( GStreamer_HOME_INCLUDE gst/gst.h
   # installation selected by user
    /usr/include/gstreamer-0.10
  ${GStreamer_DIR}/include/gstreamer-0.10/
  $ENV{GStreamer_DIR}/include/gstreamer-0.10/
  )

message(STATUS "GStreamer_HOME_INCLUDE ${GStreamer_HOME_INCLUDE}")
if( GStreamer_HOME_INCLUDE )

    set( GStreamer_FOUND 1 CACHE BOOL "Do we have all required Gstreamer?" FORCE )



    # include and lib dirs are easy
    set( GStreamer_INCLUDE_DIRS ${GStreamer_HOME_INCLUDE})
    set( GStreamer_LIB_DIRS /usr/lib/x86_64-linux-gnu/gstreamer-0.10/ /usr/lib/x86_64-linux-gnu)
    
    set( GStreamer_LIBRARY_NAMES
        gstreamer-0.10
        gstbase-0.10
        gstcheck-0.10
        gstcontroller-0.10
        gstdataprotocol-0.10
        gstnet-0.10
        gstapp-0.10
    )


    foreach(GStreamer_LIBRARY_NAME ${GStreamer_LIBRARY_NAMES})
        find_library(GStreamer_${GStreamer_LIBRARY_NAME}_LIBRARY NAMES "${GStreamer_LIBRARY_NAME}" PATHS ${GStreamer_LIB_DIRS})
        if(GStreamer_${GStreamer_LIBRARY_NAME}_LIBRARY)
            set(GStreamer_${GStreamer_LIBRARY_NAME}_LIBRARY_FOUND TRUE)
            list(APPEND GStreamer_LIBRARIES "${GStreamer_${GStreamer_LIBRARY_NAME}_LIBRARY}")
        else()
            set(GStreamer_${GStreamer_LIBRARY_NAME}_LIBRARY_FOUND FALSE)
            set(GStreamer_FOUND 0 CACHE BOOL "Do we have all required GStreamer?" FORCE)
        endif()
    endforeach()




endif( GStreamer_HOME_INCLUDE )
