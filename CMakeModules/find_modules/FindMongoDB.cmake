#
# - Find MongoDB executables
#
#  MongoDB_FOUND        - True if mongo found.

set(H2T_MONGODB_PATH /org/share/archive/SFB588_RefDist/mca_12.04/64bit/mongodb)

set(MongoDB_EXECUTABLES mongo mongod mongorestore mongoexport mongodump mongoimport)

foreach(Mongo_EXE ${MongoDB_EXECUTABLES})
    find_program("MongoDB_${Mongo_EXE}_EXECUTABLE" ${Mongo_EXE} "${H2T_MONGODB_PATH}/bin" "${MongoDB_DIR}/bin" "${MongoDBTools_DIR}/bin")
endforeach()


# handle the QUIETLY and REQUIRED arguments and set MONGODB_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MongoDB MongoDB_mongo_EXECUTABLE MongoDB_mongod_EXECUTABLE MongoDB_mongorestore_EXECUTABLE MongoDB_mongoexport_EXECUTABLE MongoDB_mongodump_EXECUTABLE)
set(MongoDB_FOUND ${MONGODB_FOUND})
