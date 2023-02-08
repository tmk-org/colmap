if(NOT IS_MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-narrowing")
endif()

if(NOT SIMD_ENABLED)
    add_definitions("-DDISABLE_CPU_NEON")
    add_definitions("-DDISABLE_CPU_AVX")
    add_definitions("-DDISABLE_CPU_SSE")
endif()

if(CUDA_ENABLED)
    COLMAP_ADD_CUDA_LIBRARY(pba
        ConfigBA.cpp
        ConfigBA.h
        CuTexImage.cpp
        CuTexImage.h
        DataInterface.h
        pba.cpp
        pba.h
        ProgramCU.cu
        ProgramCU.h
        SparseBundleCPU.cpp
        SparseBundleCPU.h
        SparseBundleCU.cpp
        SparseBundleCU.h
        util.h
    )
    if((CMAKE_VERSION VERSION_GREATER_EQUAL 3.18.0) AND (CUDA_ENABLED))
        message("set CUDA_ARCHITECTURES \"${CUDA_ARCH_SM_CC}\" to target \"pba\"")
        set_property(TARGET pba PROPERTY CUDA_ARCHITECTURES "${CUDA_ARCH_SM_CC}")
    endif()
else()
    add_definitions("-DPBA_NO_GPU")

    COLMAP_ADD_LIBRARY(pba
        ConfigBA.cpp
        ConfigBA.h
        DataInterface.h
        pba.cpp
        pba.h
        SparseBundleCPU.cpp
        SparseBundleCPU.h
        util.h
    )
endif()
