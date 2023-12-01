// #include <igl/barycentric_coordinates.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/point_mesh_squared_distance.h>
#include <igl/readOBJ.h>

#include <args.hxx>
#include <iostream>
#include <ostream>
#include <tuple>
#include <vector>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "igl/readOFF.h"

using namespace Eigen;
using std::cout;
using std::vector;

void process_barycentric(const Eigen::MatrixXd& inputModelVertices,
                         const Eigen::MatrixXi& inputModelFaces,
                         const Eigen::MatrixXd& inputPointsVertices) {
    // Load a mesh in OFF format
    // igl::readOFF("./dd.off", inputModelVertices, inputModelFaces);

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

    cout << std::endl;
    // Make select pair part
    int size = elementI.size();
    vector<std::tuple<int, double, double>> shit;
    for (int i = 0; i < L.rows(); ++i) {
        shit.push_back({elementI(i), L(i, 0), L(i, 1)});
    }
    // Output part
    json jVector(shit);
    cout << jVector;
}


// Struct to transfer off to json easily
struct vertex {
    double x;
    double y;
    double z;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(vertex, x, y, x);
void transfer_off_json(const Eigen::MatrixXd& inputPointsVertices) {
    vector<vertex> result;
    for (int i = 0; i < inputPointsVertices.cols(); ++i) {
        result.push_back({inputPointsVertices(i, 0), inputPointsVertices(i, 1),
                          inputPointsVertices(i, 2)});
    }
    json jVector(result);
    // Output part
    cout << jVector;
}

int main(int argc, char* argv[]) {
    args::ArgumentParser parser(
        "A simple program for wrap4d coordinates transfer.");
    args::HelpFlag(parser, "help",
                   "If you need help, check below:", {'h', "help"});
    args::Group guiGroup(parser,
                         "If you use -g, you can view the points in imgui:",
                         args::Group::Validators::AllOrNone);
    args::Flag isUsingGUI(guiGroup, "gui", "The GUI flag.", {'g', "gui"});

    args::Group engineGroup(
        parser,
        "If you use this group, you are using the engine provided by me:");
    args::Flag isUsingInsertedEngine(
        engineGroup, "engine", "The inserted engine flag", {'e', "engine"});

    try {
        parser.ParseCLI(argc, argv);
    } catch (const args::Completion& e) {
        std::cout << e.what();
        return 0;
    } catch (const args::ParseError& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << parser;
        return 1;
    } catch (const args::Help&) {
        std::cout << parser;
        return 0;
    }

    // Setup input mesh and points matrix
    Eigen::MatrixXd inputModelVertices;
    Eigen::MatrixXi inputModelFaces;
    Eigen::MatrixXd inputPointsVertices;
    Eigen::MatrixXi inputPointsFaces;
    // Load Meshes
    if (isUsingGUI || isUsingInsertedEngine) {
        igl::readOBJ("./input.obj", inputModelVertices, inputModelFaces);
    }
    igl::readOFF("./input.off", inputPointsVertices, inputPointsFaces);

    if (isUsingInsertedEngine) {
        process_barycentric(inputModelVertices, inputModelFaces,
                            inputPointsVertices);
    } else {
        transfer_off_json(inputPointsVertices);
    }

    if (isUsingGUI) {
        // Plot the mesh
        igl::opengl::glfw::Viewer viewer;
        viewer.data().set_mesh(inputModelVertices, inputModelFaces);
        viewer.data().add_points(inputPointsVertices,
                                 Eigen::RowVector3d(1, 0, 0));
        viewer.launch();
    }
}