# Build and helper macros

function(VirtualRobotApplication name srcs incs)
    ################################## EXECUTABLE ##############################
    ADD_EXECUTABLE(${name} ${srcs} ${incs})
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
    TARGET_LINK_LIBRARIES(${name} PUBLIC VirtualRobot)
endfunction()


function(VirtualRobotQtApplication name srcs incs mocFiles uiFiles)
    set(CMAKE_AUTOMOC "YES")
    set(CMAKE_AUTOUIC "YES")
    set(generatedUiFiles ${uiFiles})
    set(generatedMocFiles ${mocFiles})
    ################################## EXECUTABLE ##############################
    ADD_EXECUTABLE(${name} ${srcs} ${incs} ${generatedUiFiles} ${generatedMocFiles})
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
    FIND_PACKAGE(SoQt REQUIRED)
    TARGET_LINK_LIBRARIES(${name} PUBLIC VirtualRobot SoQt::SoQt)
endfunction()


function(VirtualRobotQtLibrary name srcs incs mocFiles uiFiles)
    set(generatedUiFiles ${uiFiles})
    set(generatedMocFiles ${mocFiles})
    set(CMAKE_AUTOMOC "YES")
    set(CMAKE_AUTOUIC "YES")

    ################################## LIBRARY ##############################
    ADD_LIBRARY(${name} SHARED ${srcs} ${incs} ${generatedUiFiles} ${generatedMocFiles})
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
    TARGET_LINK_LIBRARIES(${name} PUBLIC VirtualRobot)
endfunction()


function(SimoxApplication name srcs incs)
    VirtualRobotApplication("${name}" "${srcs}" "${incs}")
    # add Saba and GraspStudio
    TARGET_LINK_LIBRARIES(${name} PUBLIC GraspStudio Saba)
endfunction()


function(SimoxQtApplication name srcs incs mocFiles uiFiles)
    VirtualRobotQtApplication("${name}" "${srcs}" "${incs}" "${mocFiles}" "${uiFiles}")
    # add Saba and GraspStudio
    TARGET_LINK_LIBRARIES(${name} PUBLIC GraspStudio Saba)
endfunction()


function(SimoxQtLibrary name srcs incs mocFiles uiFiles)
    VirtualRobotQtLibrary("${name}" "${srcs}" "${incs}" "${mocFiles}" "${uiFiles}")
    # add Saba and GraspStudio
    TARGET_LINK_LIBRARIES(${name} PUBLIC GraspStudio Saba)
endfunction()

function(simox_subdirs RESULT DIRECTORY)
    file(GLOB CHILDREN ${DIRECTORY}/*)
    # message("## children of ${DIRECTORY}: ${CHILDREN}")

    set(DIR_CHILDREN "")
    foreach(CHILD ${CHILDREN})
        if (IS_DIRECTORY "${CHILD}")
            list(APPEND DIR_CHILDREN ${CHILD})
        endif()
    endforeach()

    set(${RESULT} "${DIR_CHILDREN}" PARENT_SCOPE)
endfunction()


function(simox_subdirs_recursive RESULT DIRECTORY)
    simox_subdirs(TOP_CHILDREN "${DIRECTORY}")
    set(ALL_CHILDREN ${TOP_CHILDREN})

    foreach(TOP_CHILD ${TOP_CHILDREN})
        simox_subdirs_recursive(REC_CHILDREN ${TOP_CHILD})
        list(APPEND ALL_CHILDREN ${REC_CHILDREN})
    endforeach()

    set(${RESULT} "${ALL_CHILDREN}" PARENT_SCOPE)

endfunction()


macro(simox_update_file file content) #macro since we want to provide simox_file_up_to_date
    set(simox_file_up_to_date 0)
    if(EXISTS "${file}")
        file(SHA512 "${file}" _simox_file_sha)
        string(SHA512 _simox_cont_sha "${content}")
        if("${_simox_cont_sha}" STREQUAL "${_simox_file_sha}")
            set(simox_file_up_to_date 1)
        endif()
    endif()
    #write file
    if(${simox_file_up_to_date} EQUAL 0)
        file(WRITE "${file}" "${content}")
    endif()
endmacro()

macro(simox_generate_subdir_headers dir prefix_to_remove out_headers)
    string(REPLACE "//" "/" prefix_to_remove_fixed "${prefix_to_remove}/")
    string(REPLACE "${prefix_to_remove_fixed}" "" dir_rel "${dir}")
    _simox_generate_subdir_headers_impl("${dir}" "${prefix_to_remove_fixed}" ${out_headers})
endmacro()

macro(_simox_generate_subdir_headers_impl dir prefix_to_remove out_headers)
    simox_subdirs(subdirs ${dir})
    foreach(subdir ${subdirs})
        #recurse
        _simox_generate_subdir_headers_impl("${subdir}" "${prefix_to_remove}" ${out_headers})

        file(GLOB headers ${subdir}/*.h)
        list(LENGTH headers n)
        string(REGEX REPLACE ".*/" "" subdir_name "${subdir}")
        if(
                NOT ${n} EQUAL 0                          AND
                NOT "${subdir_name}" STREQUAL "detail"    AND
                NOT "${subdir_name}" STREQUAL "_detail"   AND
                NOT "${subdir_name}" STREQUAL "internal"  AND
                NOT "${subdir_name}" STREQUAL "_internal"
        )
            set(subdir_header_abs "${subdir}.h")
            string(REPLACE "${prefix_to_remove}" "" subdir_header "${subdir_header_abs}")
            list(APPEND ${out_headers} ${subdir_header})
            #create file content
            set(content "#pragma once\n\n// This file is generated!\n\n")
            foreach(header ${headers})
                string(REPLACE "${subdir}/" "${subdir_name}/" h "${header}")
                set(content "${content}#include \"${h}\"\n")
            endforeach()
            #check for file change
            simox_update_file("${subdir_header_abs}" "${content}")
        endif()
    endforeach()
endmacro()
