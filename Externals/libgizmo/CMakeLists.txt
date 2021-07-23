cmake_minimum_required(VERSION 3.5)
cmake_policy(VERSION 3.5)

project(libgizmo CXX C)

set(SRC
    Src/GizmoTransformMove.cpp Src/GizmoTransformRotate.cpp Src/ZBaseMaths.cpp
    Src/GizmoTransformRender.cpp Src/GizmoTransformScale.cpp Src/ZMathsFunc.cpp
    )

add_library(libgizmo ${SRC})
target_include_directories(libgizmo PUBLIC Include)

add_library(libgizmo::libgizmo ALIAS libgizmo)
