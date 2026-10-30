cmake_minimum_required(VERSION 3.10)

project(TOPO_GRAPH_2D)

# 设置CMP0011策略
cmake_policy(SET CMP0011 NEW)

# 设置CMAKE_BUILD_TYPE
set(CMAKE_BUILD_TYPE Release)

# 添加-fPIC标志
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

# 查找Boost库
find_package(Boost REQUIRED COMPONENTS filesystem)
if(NOT Boost_FOUND)
  message(STATUS "This project requires the Boost library, and will not be compiled.")
  return()
endif()

# 查找CGAL库及其组件
find_package(CGAL REQUIRED)
if(NOT CGAL_FOUND)
  message(STATUS "This program requires the CGAL library, and will not be compiled.")
  return()
endif()

# 查找Qt库 (优先Qt6，回退Qt5)
find_package(Qt6 COMPONENTS Core Widgets Gui QUIET)
if(Qt6_FOUND)
  message(STATUS "Found Qt6: ${Qt6_VERSION}")
  set(QT_LIBS Qt6::Widgets Qt6::Core Qt6::Gui)
  set(CMAKE_AUTOMOC ON)
else()
  find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)
  if(NOT Qt5_FOUND)
    message(STATUS "This program requires the Qt5/Qt6 library, and will not be compiled.")
    return()
  endif()
  message(STATUS "Found Qt5: ${Qt5_VERSION}")
  set(QT_LIBS Qt5::Widgets Qt5::Core Qt5::Gui)
endif()

# 查找yaml-cpp库
find_package(yaml-cpp REQUIRED)

# 包含CGAL和Qt5的配置文件 (CGAL_USE_FILE在老版本有效)
if(DEFINED CGAL_USE_FILE)
  include(${CGAL_USE_FILE})
endif()

# 设置RPATH
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# 添加库文件
add_library(topo_graph_2d STATIC
  src/VoriConfig.cpp
  src/VoriGraph.cpp
  src/TopoGraph.cpp
  src/RoomDect.cpp
  src/roomGraph.cpp
  src/Denoise.cpp
  src/passageSearch.cpp
  src/cgal/CgalVoronoi.cpp
  src/cgal/AlphaShape.cpp
  src/qt/QImageVoronoi.cpp
  src/cgal/AlphaShapeRemoval.cpp
  src/osmAGExport.cpp
  src/geometry/GeometryUtils.cpp
  src/polygon/PolygonProcessor.cpp
  src/room/RoomProcessor.cpp
  src/passage/PassageProcessor.cpp
  src/export/OsmAGExporter.cpp)

# 为topo_graph_2d库添加依赖
target_link_libraries(topo_graph_2d yaml-cpp ${QT_LIBS} CGAL::CGAL)

# 包含头文件目录
target_include_directories(topo_graph_2d PUBLIC
  ${TOPO_GRAPH_2D_SOURCE_DIR}/include
  ${CGAL_INCLUDE_DIRS}
)

# 设置输出目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# 添加可执行文件
add_executable(area_graph_segmentation examples/area_graph_segmentation.cpp)

# 添加测试可执行文件
add_executable(test_area_graph test/test_area_graph.cpp)

# 链接库文件
set(BOOST_LINK_LIBS Boost::filesystem)
if(TARGET Boost::system)
  list(APPEND BOOST_LINK_LIBS Boost::system)
endif()

target_link_libraries(area_graph_segmentation 
    topo_graph_2d 
    ${QT_LIBS}
    CGAL::CGAL
    ${BOOST_LINK_LIBS}
    yaml-cpp
)

target_link_libraries(test_area_graph 
    topo_graph_2d 
    ${QT_LIBS}
    CGAL::CGAL
    ${BOOST_LINK_LIBS}
)

# Includes regression coverage for interior and endpoint polygon-edge cuts.
enable_testing()
add_test(NAME area_graph_components COMMAND test_area_graph)
