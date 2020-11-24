set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
if (NOT OPENCV_FOUND)
    find_package(OpenCV  REQUIRED)
endif ()
if (NOT PCL_FOUND)
    find_package(PCL REQUIRED)
endif ()
file(GLOB SRC_LIST_ISC *.cpp)
add_library(isc SHARED ${SRC_LIST_ISC})
target_include_directories(isc PUBLIC ${PCL_INCLUDE_DIRS} ./)
target_link_libraries(isc
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        )