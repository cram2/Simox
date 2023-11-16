PROJECT ( SimoxUtility )

MESSAGE (STATUS "\n***** CONFIGURING Simox project SimoxUtility *****")

#################################### FILES ##################################
SET(SOURCES
    SimoxUtility.cpp

    algorithm/string/string_tools.cpp
    algorithm/string/fuzzy_match.cpp

    caching/CacheMap.cpp

    color/cmaps/colormaps.cpp
    color/cmaps/Named.cpp
    color/Color.cpp
    color/ColorMap.cpp
    color/GlasbeyLUT.cpp
    color/hsv.cpp
    color/interpolation.cpp
    color/json.cpp

    error/SimoxError.cpp

    filesystem/list_directory.cpp

    json/eigen_conversion.cpp
    json/error.cpp
    json/io.cpp
    json/converters.cpp
    json/util.cpp

    math/SoftMinMax.cpp

    math/periodic/periodic_diff.cpp
    math/periodic/periodic_mean.cpp

    math/pose/align_box_orientation.cpp
    math/pose/check_rotation_matrix.cpp
    math/pose/invert.cpp
    math/pose/orthogonalize.cpp
    math/pose/interpolate.cpp

    math/regression/linear.cpp

    math/statistics/Histogram1D.cpp

    meta/type_name.cpp

    simox/SimoxPath.cpp

    shapes/AxisAlignedBoundingBox.cpp
    shapes/json_conversions.cpp

    threads/system_thread_id.cpp
    threads/CountingSemaphore.cpp
)

SET(INCLUDES
    SimoxUtility.h

    preprocessor/variadic_for_each.h
    preprocessor/drop_front.h

    algorithm/advanced.h
    algorithm/apply.hpp
    algorithm/contains.h
    algorithm/for_each_if.h
    algorithm/fuzzy_find.h
    algorithm/get_map_keys_values.h
    algorithm/ordered_circular_buffer.h
    algorithm/minmax.h
    algorithm/string/fuzzy_match.h
    algorithm/string/fuzzy_match.hpp
    algorithm/string/fuzzy_multi_match.hpp
    algorithm/string/string_tools.h
    algorithm/string/string_conversion.h
    algorithm/string/string_conversion_eigen.h
    algorithm/vector.hpp

    backport/span/gcc_header.h

    caching/CacheMap.h

    color/cmaps/colormaps.h
    color/cmaps/Named.h
    color/convert_byte_float.hpp
    color/Color.h
    color/ColorMap.h
    color/GlasbeyLUT.h
    color/KellyLUT.h
    color/hsv.h
    color/interpolation.h
    color/json.h

    EigenStdVector.h
    eigen/EigenStdVector.h
    eigen/AssertAsException.h

    error/SimoxError.h

    filesystem/make_relative.h
    filesystem/list_directory.h
    filesystem/remove_trailing_separator.h

    json.h
    json/converters.h
    json/eigen_conversion.h
    json/error.h
    json/io.h
    json/json.hpp
    json/json.h
    json/util.h

    math/mean.h
    math/sum.h
    math/SoftMinMax.h

    math/scale_value.h
    math/rescale.h
    math/project_to_plane.h

    math/isfinite.h
    math/zero.h

    math/compare/is_equal.h
    math/compare/is_greater.h
    math/compare/is_greater_equal.h
    math/compare/is_inequal.h
    math/compare/is_less.h
    math/compare/is_less_equal.h
    math/compare/value_in_limits_of.h

    math/convert/rad_to_deg.h
    math/convert/deg_to_rad.h

    math/convert/aa_to_mat3f.h
    math/convert/aa_to_mat4f.h
    math/convert/aa_to_quat.h
    math/convert/aa_to_rpy.h

    math/convert/mat3f_to_aa.h
    math/convert/mat3f_to_mat4f.h
    math/convert/mat3f_to_quat.h
    math/convert/mat3f_to_rpy.h

    math/convert/mat4f_to_aa.h
    math/convert/mat4f_to_mat3f.h
    math/convert/mat4f_to_pos.h
    math/convert/mat4f_to_quat.h
    math/convert/mat4f_to_rpy.h

    math/convert/pos_aa_to_mat4f.h
    math/convert/pos_mat3f_to_mat4f.h
    math/convert/pos_quat_to_mat4f.h
    math/convert/pos_rpy_to_mat4f.h
    math/convert/pos_to_mat4f.h

    math/convert/quat_to_aa.h
    math/convert/quat_to_mat3f.h
    math/convert/quat_to_mat4f.h
    math/convert/quat_to_rpy.h

    math/convert/rpy_to_aa.h
    math/convert/rpy_to_mat3f.h
    math/convert/rpy_to_mat4f.h
    math/convert/rpy_to_quat.h

    math/convert/mat4f_to_xyyaw.h
    math/convert/xyyaw_to_mat4f.h

    math/distance/delta_angle.h
    math/distance/angle_between.h

    math/norm/norm_clamp.h
    math/norm/norm_max.h
    math/norm/norm_min.h

    math/normal/orthogonal_vector.h
    math/normal/normal_to_mat3.h
    math/normal/normal_to_mat4.h

    math/periodic_clamp.h  # Legacy header.
    math/periodic/periodic_clamp.h
    math/periodic/periodic_diff.h
    math/periodic/periodic_mean.h

    math/pose/align_box_orientation.h
    math/pose/check_rotation_matrix.h
    math/pose/is_homogeneous_transform.h
    math/pose/invert.h
    math/pose/orthogonalize.h
    math/pose/pose.h
    math/pose/transform.h
    math/pose/interpolate.h

    math/regression/linear.h
    math/regression/linear.hpp

    math/similarity/cosine_similarity.h
    math/similarity/angular_similarity.h

    math/statistics/BoxPlotStats.h
    math/statistics/Histogram1D.h
    math/statistics/measures.h

    meta/undefined_t/undefined_t.h
    meta/undefined_t/is_set.h
    meta/eigen/enable_if_compile_time_size.h
    meta/eigen/is_eigen_array.h
    meta/eigen/is_eigen_matrix.h
    meta/has_member_macros/has_member.hpp
    meta/type_name.h
    meta/boost_hana/has_hana_accessor.h
    meta/boost_hana/hana_member_name.h
    meta/type_traits/is_any_of.h
    meta/type_traits/is_std_array.h
    meta/type_traits/are_arithmetic.h
    meta/type_traits/is_string_like.h
    meta/template_traits/is_instantiation_of.h

    meta/EnumNames.hpp
    meta/enum/adapt_enum.h
    meta/enum/EnumNames.hpp
    meta/key_type.h

    random/choice.h

    simox/SimoxPath.h

    shapes/AxisAlignedBoundingBox.h
    shapes/OrientedBoxBase.h
    shapes/OrientedBox.h
    shapes/XYConstrainedOrientedBox.h
    shapes/json_conversions.h

    threads/system_thread_id.h
    threads/CountingSemaphore.h

    xml/rapidxml/rapidxml.hpp
    xml/rapidxml/rapidxml_print.hpp
    xml/rapidxml/RapidXMLWrapper.h
    xml/rapidxml/RapidXMLForwardDecl.h
)


# Generate subdirectory headers
simox_generate_subdir_headers(
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}
    INCLUDES
)


#######################################################################
# Add library
#######################################################################

set(CMAKE_INCLUDE_CURRENT_DIR OFF)
add_library(SimoxUtility SHARED
    ${SOURCES}
    ${INCLUDES}
)

target_include_directories(SimoxUtility PUBLIC
    $<BUILD_INTERFACE:${Simox_SOURCE_DIR}>
    $<BUILD_INTERFACE:${Simox_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(SimoxUtility PUBLIC ${Simox_OS_compile_options})
target_link_libraries(SimoxUtility PUBLIC stdc++fs)


#######################################################################
# Setup dependencies
#######################################################################
# Eigen
target_link_libraries(SimoxUtility PUBLIC Eigen3::Eigen)

# BOOST
target_link_libraries(SimoxUtility PUBLIC
        Boost::boost
        Boost::program_options
        Boost::thread
        Boost::regex
)

# difflib
if (difflib_FOUND)
    target_link_libraries(SimoxUtility PUBLIC difflib::difflib)
    target_compile_definitions(SimoxUtility PUBLIC DIFFLIB_FOUND)
endif()


target_compile_options(SimoxUtility PUBLIC -include SimoxUtility/eigen/AssertAsException.h)

# disable boost auto linking
if (Boost_USE_STATIC_LIBS)
    target_compile_definitions(SimoxUtility PUBLIC -DBOOST_ALL_NO_LIB -DBOOST_TEST_MAIN)
else (Boost_USE_STATIC_LIBS)
    # enable dynamic linking for specific boost libraries
    target_compile_definitions(SimoxUtility PUBLIC
        -DBOOST_ALL_NO_LIB
        -DBOOST_PROGRAM_OPTIONS_DYN_LINK
        -DBOOST_FILESYSTEM_DYN_LINK
        -DBOOST_SYSTEM_DYN_LINK
        -DBOOST_THREAD_DYN_LINK)
endif ()

##########################################################################################################
#################################### generate meta functions ##################################

macro(make_check_for_member)
    macro(make_check_for_member_sub category name)
        set(targrel meta/type_traits/member/${category}/${name}.h)
        set(ofile ${CMAKE_CURRENT_LIST_DIR}/${targrel})
        set(ifile ${CMAKE_SOURCE_DIR}/CMakeModules/check_for_member/${category}.in.h)
        set(NAME ${name})
        configure_file(${ifile} ${ofile} @ONLY)
        list(APPEND INCLUDES ${targrel})
    endmacro()
    set(GEN_CHECK_MEMBER_META_CODE_KEYWORDS ${ARGN})
    list(REMOVE_DUPLICATES GEN_CHECK_MEMBER_META_CODE_KEYWORDS)
    set(GEN_CHECK_MEMBER_META_CODE_CATEGORIES
        method static_method
        variable static_variable
        type
    )
    foreach(v ${GEN_CHECK_MEMBER_META_CODE_KEYWORDS})
        foreach(c ${GEN_CHECK_MEMBER_META_CODE_CATEGORIES})
            make_check_for_member_sub(${c} ${v})
        endforeach()
    endforeach()
endmacro()
make_check_for_member(
    #terator
    begin end cbegin cend
    #container
    size empty reserve clear resize
    emplace_back emplace_front emplace
    push_back push_front
    insert
    at front back data
    #point members (wh sometimes are homogeneous coords)
    point vertex x y z w h _0 _1 _2
    #quat members
    qx qy qz qw
    #points members
    vertices points
    #other
    value val npos
    value_t value_type size_type size_t iterator
    first second
    exact extract str string c_str swap
    min max
)


##########################################################################################################
##########################################################################################################
##########################################################################################################
if(BUILD_TESTING)
    # include unit tests
    ADD_SUBDIRECTORY(tests)
endif()

if(Simox_BUILD_EXAMPLES)
    # include examples
    ADD_SUBDIRECTORY(examples)
endif()

# Finished adding UI files
#-----------------------------------------------------------------------------------

SET_TARGET_PROPERTIES(SimoxUtility PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})
SET_TARGET_PROPERTIES(SimoxUtility PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${Simox_LIB_DIR})
SET_TARGET_PROPERTIES(SimoxUtility PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${Simox_LIB_DIR})


#######################################################################################
############################ Setup for installation ###################################
#######################################################################################

INSTALL(TARGETS SimoxUtility
    # IMPORTANT: Add the library to the "export-set"
    EXPORT SimoxTargets
    RUNTIME DESTINATION bin COMPONENT bin
    LIBRARY DESTINATION lib COMPONENT shlib
    ARCHIVE DESTINATION lib COMPONENT library
    COMPONENT dev
)

INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} DESTINATION "${Simox_INSTALL_HEADER_DIR}"
    COMPONENT Headers
        FILES_MATCHING
        PATTERN "*.h"
        PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE
        PATTERN ".git" EXCLUDE
        PATTERN "CMakeModules" EXCLUDE
        PATTERN "tests" EXCLUDE
        PATTERN "build*" EXCLUDE
        PATTERN "data" EXCLUDE
        PATTERN "examples" EXCLUDE
)

INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/data DESTINATION "."
    COMPONENT ApplicationData
        FILES_MATCHING
        PATTERN "*.xml"
        PATTERN "*.XML"
        PATTERN "*.iv"
        PATTERN "*.vrml"
        PATTERN "*.wrl"
        PATTERN "*.stl"
        PATTERN "*.urdf"
        PATTERN "*.dae"
        PATTERN "*.png"
        PATTERN "*.jpg"
        PATTERN "*.bin"
        PATTERN ".svn" EXCLUDE
        PATTERN ".git" EXCLUDE
)

MESSAGE (STATUS "***** Finished CONFIGURING Simox project SimoxUtility *****\n")
