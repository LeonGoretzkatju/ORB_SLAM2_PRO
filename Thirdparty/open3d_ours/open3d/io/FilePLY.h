//
// Created by xin on 2021/1/12.
//

#ifndef OPEN3D_PART_FILEPLY_H
#define OPEN3D_PART_FILEPLY_H

#include "../geometry/TriangleMesh.h"

namespace open3d {

    namespace io {
        bool WriteTriangleMeshToPLY(const std::string &filename,
                                    const geometry::TriangleMesh &mesh,
                                    bool write_ascii = false,
                                    bool compressed = false,
                                    bool write_vertex_normals = true,
                                    bool write_vertex_colors = true,
                                    bool write_triangle_uvs = true,
                                    bool print_progress = false);
    }
}

#endif //OPEN3D_PART_FILEPLY_H
