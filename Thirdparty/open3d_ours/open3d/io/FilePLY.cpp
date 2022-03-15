// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

//#include <rply.h>
#include "../3dpart/rply/rply/rply.h"
//#include "open3d/io/FileFormatIO.h"
//#include "open3d/io/LineSetIO.h"
//#include "open3d/io/PointCloudIO.h"
//#include "open3d/io/TriangleMeshIO.h"
//#include "open3d/io/VoxelGridIO.h"
//#include "open3d/utility/Console.h"
//#include "open3d/utility/ProgressReporters.h"

#include "../geometry/TriangleMesh.h"
#include "FilePLY.h"

namespace open3d {

namespace io {

bool WriteTriangleMeshToPLY(const std::string &filename,
                            const geometry::TriangleMesh &mesh,
                            bool write_ascii /* = false*/,
                            bool compressed /* = false*/,
                            bool write_vertex_normals /* = true*/,
                            bool write_vertex_colors /* = true*/,
                            bool write_triangle_uvs /* = true*/,
                            bool print_progress) {
    if (write_triangle_uvs && mesh.HasTriangleUvs()) {
        utility::LogWarning(
                "This file format currently does not support writing textures "
                "and uv coordinates. Consider using .obj");
    }

    if (mesh.IsEmpty()) {
        utility::LogWarning("Write PLY failed: mesh has 0 vertices.");
        return false;
    }

    p_ply ply_file = ply_create(filename.c_str(),
                                write_ascii ? PLY_ASCII : PLY_LITTLE_ENDIAN,
                                NULL, 0, NULL);
    if (!ply_file) {
        utility::LogWarning("Write PLY failed: unable to open file: {}",
                            filename);
        return false;
    }

    write_vertex_normals = write_vertex_normals && mesh.HasVertexNormals();
    write_vertex_colors = write_vertex_colors && mesh.HasVertexColors();

    ply_add_comment(ply_file, "Created by Open3D");
    ply_add_element(ply_file, "vertex",
                    static_cast<long>(mesh.vertices_.size()));
    ply_add_property(ply_file, "x", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
    ply_add_property(ply_file, "y", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
    ply_add_property(ply_file, "z", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
    if (write_vertex_normals) {
        ply_add_property(ply_file, "nx", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
        ply_add_property(ply_file, "ny", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
        ply_add_property(ply_file, "nz", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
    }
    if (write_vertex_colors) {
        ply_add_property(ply_file, "red", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
        ply_add_property(ply_file, "green", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
        ply_add_property(ply_file, "blue", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
    }
    ply_add_element(ply_file, "face",
                    static_cast<long>(mesh.triangles_.size()));
    ply_add_property(ply_file, "vertex_indices", PLY_LIST, PLY_UCHAR, PLY_UINT);
    if (!ply_write_header(ply_file)) {
        utility::LogWarning("Write PLY failed: unable to write header.");
        ply_close(ply_file);
        return false;
    }

    utility::ConsoleProgressBar progress_bar(
            static_cast<size_t>(mesh.vertices_.size() + mesh.triangles_.size()),
            "Writing PLY: ", print_progress);
    bool printed_color_warning = false;
    for (size_t i = 0; i < mesh.vertices_.size(); i++) {
        const auto &vertex = mesh.vertices_[i];
        ply_write(ply_file, vertex(0));
        ply_write(ply_file, vertex(1));
        ply_write(ply_file, vertex(2));
        if (write_vertex_normals) {
            const auto &normal = mesh.vertex_normals_[i];
            ply_write(ply_file, normal(0));
            ply_write(ply_file, normal(1));
            ply_write(ply_file, normal(2));
        }
        if (write_vertex_colors) {
            const auto &color = mesh.vertex_colors_[i];
            if (!printed_color_warning &&
                (color(0) < 0 || color(0) > 1 || color(1) < 0 || color(1) > 1 ||
                 color(2) < 0 || color(2) > 1)) {
                utility::LogWarning(
                        "Write Ply clamped color value to valid range");
                printed_color_warning = true;
            }
            auto rgb = utility::ColorToUint8(color);
            ply_write(ply_file, rgb(0));
            ply_write(ply_file, rgb(1));
            ply_write(ply_file, rgb(2));
        }
        ++progress_bar;
    }
    for (size_t i = 0; i < mesh.triangles_.size(); i++) {
        const auto &triangle = mesh.triangles_[i];
        ply_write(ply_file, 3);
        ply_write(ply_file, triangle(0));
        ply_write(ply_file, triangle(1));
        ply_write(ply_file, triangle(2));
        ++progress_bar;
    }

    ply_close(ply_file);
    return true;
}

}  // namespace io
}  // namespace open3d
