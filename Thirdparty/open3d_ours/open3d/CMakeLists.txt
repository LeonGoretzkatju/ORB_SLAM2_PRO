

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
        utility/Eigen.cpp
        utility/Helper.cpp

        #visualization/DrawGeometry.cpp

        io/FilePLY.cpp
)


target_link_libraries(open3d_part_lib rply_lib ${OpenCV_LIBS})