#
# - Find MongoClient headers and library
#
#  MongoClient_FOUND        - True if mongo driver found.

set(MongoClient_INCLUDE_PATHS
    "${MongoClient_DIR}/include"
    "$ENV{MongoClient_DIR}/include"
)
find_path(MongoClient_INCLUDE_DIRS
    NAMES mongo/client/connpool.h mongo/client/dbclient.h
    PATHS ${MongoClient_INCLUDE_PATHS})

set(MongoClient_LIBRARY_PATHS
    "${MongoClient_DIR}/lib"
    "$ENV{MongoClient_DIR}/lib"
)
find_library(MongoClient_LIBRARY mongoclient PATHS ${MongoClient_LIBRARY_PATHS})

find_library(ssl_LIBRARY NAMES ssl PATHS ${MongoClient_LIBRARY_PATHS})
find_library(crypto_LIBRARY NAMES crypto PATHS ${MongoClient_LIBRARY_PATHS})

# handle the QUIETLY and REQUIRED arguments and set MONGODDRIVER_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)

set(MongoClient_LIBRARIES "${MongoClient_LIBRARY}" "${ssl_LIBRARY}" "${crypto_LIBRARY}")
find_package_handle_standard_args(MongoClient MongoClient_INCLUDE_DIRS MongoClient_LIBRARY MongoClient_LIBRARIES)
set(MongoClient_FOUND ${MONGOCLIENT_FOUND})
