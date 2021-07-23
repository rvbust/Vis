add_subdirectory(pybind11)

if(BUILD_TESTS)
  add_subdirectory(Tests)
endif(BUILD_TESTS)
