# We put python binding into a seperate lib folder 
pybind11_add_module(PyVis PyVis.cpp)
set_target_properties(PyVis PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE)
target_include_directories(PyVis PRIVATE ${EIGEN3_INCLUDE_DIR} ${LOGGER_INCLUDE_DIRS})
target_link_libraries(PyVis PRIVATE RVS_Vis ${OSG_LIBS})
set_target_properties(PyVis PROPERTIES INSTALL_RPATH "/home/rvbust/Rvbust/Install/RVS/Lib")
set_target_properties(PyVis PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
set_target_properties(PyVis
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/Python/RVS/Vis"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/Python/RVS/Vis"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
    )
  
# add_sanitizers(PyVis)
