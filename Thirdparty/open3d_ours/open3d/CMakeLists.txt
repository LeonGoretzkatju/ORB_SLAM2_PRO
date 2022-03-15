find_package(OpenMP)
    if(TARGET OpenMP::OpenMP_CXX)
        message(STATUS "Building with OpenMP")
        set(OPENMP_TARGET "OpenMP::OpenMP_CXX")
        list(APPEND Open3D_3RDPARTY_PRIVATE_TARGETS "${OPENMP_TARGET}")
        if(NOT BUILD_SHARED_LIBS)
            list(APPEND Open3D_3RDPARTY_EXTERNAL_MODULES "OpenMP")
        endif()
    endif()
add_library(
        open3d_part_lib
        SHARED

        pipelines/integration/UniformTSDFVolume.cpp
        pipelines/integration/ScalableTSDFVolume.cpp
        pipelines/integration/TSDFVolume.h

        pipelines/odometry/Odometry.cpp
        pipelines/odometry/Odometry.h
        pipelines/odometry/RGBDOdometryJacobian.cpp
        pipelines/odometry/RGBDOdometryJacobian.h
        pipelines/odometry/OdometryOption.h


        camera/PinholeCameraIntrinsic.cpp

        geometry/Image.cpp
        geometry/ImageFactory.cpp
        geometry/RGBDImage.cpp
        geometry/RGBDImageFactory.cpp
        geometry/Geometry3D.cpp
        geometry/PointCloud.cpp
        geometry/PointCloudFactory.cpp
        geometry/MeshBase.cpp
        geometry/TriangleMesh.cpp

        utility/Console.cpp
        utility/Console.h
        utility/Eigen.cpp
        # utility/Helper.cpp
        utility/FileSystem.cpp

        #visualization/DrawGeometry.cpp

        io/FilePLY.cpp
)


target_link_libraries(open3d_part_lib rply_lib ${OpenCV_LIBS} ${OPENMP_TARGET} -fopenmp)