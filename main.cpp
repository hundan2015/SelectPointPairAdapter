#include <igl/barycentric_coordinates.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/point_mesh_squared_distance.h>

#include <iostream>
#include <ostream>
#include <vector>

#include "igl/readOFF.h"

using namespace Eigen;
using std::cout;
using std::vector;

int main(int argc, char *argv[]) {
    Eigen::MatrixXd inputModelVertices;
    Eigen::MatrixXi inputModelFaces;
    // Load a mesh in OFF format
    igl::readOFF("./dd.off", inputModelVertices, inputModelFaces);

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
    std::cout << L;

    cout << std::endl;
    // Make select pair part
    int size = elementI.size();
    vector<vector<double>> shit;
    for (int i = 0; i < L.rows(); ++i) {
        vector<double> temp;
        temp.push_back(elementI(i));
        temp.push_back(L(i, 0));
        temp.push_back(L(i, 1));
        shit.push_back(temp);
    }

    for (int i = 0; i < shit.size(); ++i) {
        for (int j = 0; j < 3; ++j) {
            cout << shit[i][j] << " ";
        }
        cout << std::endl;
    }

    // Plot the mesh
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(inputModelVertices, inputModelFaces);
    viewer.data().add_points(inputPointsVertices, Eigen::RowVector3d(1, 0, 0));
    viewer.launch();
}