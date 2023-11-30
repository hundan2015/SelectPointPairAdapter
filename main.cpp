#include <igl/barycentric_coordinates.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/point_mesh_squared_distance.h>
#include <igl/slice.h>

#include "igl/readOFF.h"
using namespace Eigen;
int main(int argc, char *argv[]) {
    Eigen::MatrixXd inputModelVertices;
    Eigen::MatrixXi inputModelFaces;
    // Load a mesh in OFF format
    igl::readOFF("./dd.off", inputModelVertices, inputModelFaces);

    // Plot the mesh
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(inputModelVertices, inputModelFaces);
    viewer.launch();

    Eigen::MatrixXd inputPointsVertices;
    Eigen::MatrixXi inputPointsFaces;

    igl::readOFF("./inputPoints.off", inputPointsVertices, inputPointsFaces);

    Eigen::VectorXd squaredDistanceFromP2Mesh;
    Eigen::VectorXi elementI;
    Eigen::MatrixXd closetPointFromP2Mesh;
    igl::point_mesh_squared_distance(inputPointsVertices, inputModelVertices,
                                     inputModelFaces, squaredDistanceFromP2Mesh,
                                     elementI, closetPointFromP2Mesh);
    Eigen::MatrixXd L;  // barycentric coordinates
    MatrixXd Va, Vb, Vc;
    MatrixXi F2;
    VectorXi xyz(3);
    xyz << 0, 1, 2;
    igl::slice(inputModelFaces, elementI, xyz, F2);
    igl::slice(inputModelVertices, F2.col(0), xyz, Va);
    igl::slice(inputModelVertices, F2.col(1), xyz, Vb);
    igl::slice(inputModelVertices, F2.col(2), xyz, Vc);
    igl::barycentric_coordinates(closetPointFromP2Mesh, Va, Vb, Vc, L);
}