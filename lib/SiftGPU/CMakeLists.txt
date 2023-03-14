add_definitions("-DSIFTGPU_NO_DEVIL")

set(SIFT_GPU_SOURCE_FILES
    FrameBufferObject.cpp
    FrameBufferObject.h
    GlobalUtil.cpp
    GlobalUtil.h
    GLTexImage.cpp
    GLTexImage.h
    ProgramGLSL.cpp
    ProgramGLSL.h
    ProgramGPU.h
    PyramidGL.cpp
    PyramidGL.h
    ShaderMan.cpp
    ShaderMan.h
    SiftGPU.cpp
    SiftGPU.h
    SiftMatch.cpp
    SiftMatch.h
    SiftPyramid.cpp
    SiftPyramid.h
)

if(CUDA_ENABLED)
    add_definitions("-DCUDA_SIFTGPU_ENABLED")

    set(SIFT_GPU_SOURCE_FILES
        ${SIFT_GPU_SOURCE_FILES}
        CuTexImage.cpp
        CuTexImage.h
        ProgramCU.cu
        ProgramCU.h
        PyramidCU.cpp
        PyramidCU.h
        SiftMatchCU.cpp
        SiftMatchCU.h
    )

    COLMAP_ADD_CUDA_LIBRARY(sift_gpu ${SIFT_GPU_SOURCE_FILES})
else()
    COLMAP_ADD_LIBRARY(sift_gpu ${SIFT_GPU_SOURCE_FILES})
endif()
message("for sift_gpu enabled GLOG_INCLUDE_DIRS \"${GLOG_INCLUDE_DIRS}\"")
target_include_directories(sift_gpu PRIVATE ${GLOG_INCLUDE_DIRS})
target_link_libraries(sift_gpu
    ${SIFT_GPU_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${OPENGL_gl_LIBRARY}
    ${OPENGL_glu_LIBRARY}
)
