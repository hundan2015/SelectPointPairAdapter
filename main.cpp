// #include <igl/barycentric_coordinates.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/point_mesh_squared_distance.h>
#include <igl/readOBJ.h>

#include <args.hxx>
#include <exception>
#include <fstream>
#include <ios>
#include <iostream>
#include <nlohmann/json.hpp>
#include <ostream>
#include <string>
#include <tuple>
#include <vector>
using json = nlohmann::json;

#include "igl/readOFF.h"

using namespace Eigen;
using std::cout;
using std::vector;

json process_barycentric(const Eigen::MatrixXd& inputModelVertices,
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
    return jVector;
}

// Struct to transfer off to json easily
struct vertex {
    double x;
    double y;
    double z;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(vertex, x, y, z);
json transfer_off_json(const Eigen::MatrixXd& inputPointsVertices) {
    vector<vertex> result;
    for (int i = 0; i < inputPointsVertices.rows(); ++i) {
        result.push_back({inputPointsVertices(i, 0), inputPointsVertices(i, 1),
                          inputPointsVertices(i, 2)});
    }
    json jVector(result);
    // Output part
    cout << jVector;
    return jVector;
}

void transfer_model(const std::string& inputFilePosition,
                    const std::string& inputMeshFilePosition,
                    const std::string& outputFileLocation,
                    bool isUsingInsertedEngine, bool isUsingGUI) {
    // Setup input mesh and points matrix
    Eigen::MatrixXd inputModelVertices;
    Eigen::MatrixXi inputModelFaces;
    Eigen::MatrixXd inputPointsVertices;
    Eigen::MatrixXi inputPointsFaces;
    if (isUsingGUI || isUsingInsertedEngine) {
        cout << "Reading input mesh file..." << std::endl;
        igl::readOBJ(inputMeshFilePosition, inputModelVertices,
                     inputModelFaces);
    }
    cout << "Reading input points file...:" << inputFilePosition << std::endl;
    igl::readOFF(inputFilePosition, inputPointsVertices, inputPointsFaces);
    cout << "Read input points success!";
    json res;

    if (isUsingInsertedEngine) {
        res = process_barycentric(inputModelVertices, inputModelFaces,
                                  inputPointsVertices);
    } else {
        // cout << "Not using engine. The input points should be:" << std::endl;
        // cout << inputPointsVertices;
        res = transfer_off_json(inputPointsVertices);
    }
    std::fstream outputStream;
    try {
        outputStream.open(outputFileLocation,
                          std::ios_base::out | std::ios_base::trunc);
    } catch (std::exception) {
        std::cerr << "Failed to open file.";
    }

    outputStream << res;
    outputStream.close();
    if (isUsingGUI) {
        // Plot the mesh
        igl::opengl::glfw::Viewer viewer;
        viewer.data().set_mesh(inputModelVertices, inputModelFaces);
        viewer.data().add_points(inputPointsVertices,
                                 Eigen::RowVector3d(1, 0, 0));
        viewer.launch();
    }
}

// TODO: Make a output project json function.
void generate_select_pair_json(const std::string kSelectPairNodeName,
                               const std::string kFileName,
                               const std::string kOutFileName,
                               const std::string kLeftPointFileName,
                               const std::string kRightPointFileName,
                               const bool kIsLeftUsingB = false,
                               const bool kIsRightUsingB = false) {
    cout << "Current wrap4d file:" << kFileName;
    std::ifstream jsonStream(kFileName);
    json result = json::parse(jsonStream);
    jsonStream.close();
    json& selectPointPair2Param =
        result["nodes"][kSelectPairNodeName]["params"];
    selectPointPair2Param["fileNameLeft"]["value"] =
        "$PROJECT_DIR/" + kLeftPointFileName;
    selectPointPair2Param["fileNameRight"]["value"] =
        "$PROJECT_DIR/" + kRightPointFileName;
    selectPointPair2Param["sourceLeft"]["value"] = 1;
    selectPointPair2Param["sourceRight"]["value"] = 1;
    if (kIsLeftUsingB) {
        cout << "Is using left.." << std::endl;
        selectPointPair2Param["pointsFormatLeft"]["value"] = 1;
    } else {
        selectPointPair2Param["pointsFormatLeft"]["value"] = 0;
    }

    if (kIsRightUsingB) {
        cout << "Is using right.." << std::endl;
        selectPointPair2Param["pointsFormatRight"]["value"] = 1;
    } else {
        selectPointPair2Param["pointsFormatRight"]["value"] = 0;
    }
    // result[kSelectPairNodeName]["params"] = selectPointPair2Param;
    std::ofstream ano(kOutFileName, std::ios_base::trunc);
    result >> ano;
    ano.close();
}
int main(int argc, char* argv[]) {
    args::ArgumentParser parser(
        "A simple program for wrap4d coordinates transfer.");
    args::HelpFlag(parser, "help",
                   "If you need help, check below:", {'h', "help"});
    args::Group commands(parser, "commands",
                         args::Group::Validators::AllOrNone);
    args::Command chroot(
        parser, "generate", "Generate specific wrap4d project file",
        [&](args::Subparser& subParser) {
            args::Group isUsingBGroup(subParser, "is Using b algo?");
            args::Flag isUsingLeft(isUsingBGroup, "engine", "The left flag",
                                   {'l', "left"});
            args::Flag isUsingRight(isUsingBGroup, "engine", "The right flag",
                                    {'r', "right"});
            args::Positional<std::string> name(subParser, "pair_name",
                                               "Select pair node name.");
            args::Positional<std::string> inputFile(subParser, "input_wrap",
                                                    "Input wrap file dir");
            args::Positional<std::string> outputFile(subParser, "output_wrap",
                                                     "Output wrap file dir");
            args::Positional<std::string> leftPoints(subParser, "left_points",
                                                     "Output wrap file dir");
            args::Positional<std::string> rightPoints(subParser, "right_points",
                                                      "Output wrap file dir");
            subParser.Parse();
            generate_select_pair_json(
                args::get(name), args::get(inputFile), args::get(outputFile),
                args::get(leftPoints), args::get(rightPoints),
                args::get(isUsingLeft), args::get(isUsingRight));
            // cout << args::get(temp);
            //  auto res = output_project_json();
        });
    args::Command transfer(
        parser, "transfer", "transfer file itself.",
        [&](args::Subparser& subParser) {
            args::Group guiGroup(
                subParser, "If you use -g, you can view the points in imgui:");
            args::Flag isUsingGUI(guiGroup, "gui", "The GUI flag.",
                                  {'g', "gui"});

            /*args::Group engineGroup(subParser,
                                    "If you use this group, you are using the "
                                    "engine provided by me:");*/
            args::Flag isUsingInsertedEngine(guiGroup, "engine",
                                             "The inserted engine flag",
                                             {'e', "engine"});
            args::Positional<std::string> bar(subParser, "inputMesh",
                                              "The input mesh file position");
            args::Positional<std::string> foo(subParser, "inputPoints",
                                              "The input points file position");
            args::Positional<std::string> tar(subParser, "outputfile",
                                              "The out points file position");
            subParser.Parse();
            std::string inputFilePosition = "./input.off";
            if (foo) {
                inputFilePosition = args::get(foo);
            } else {
                cout << "No input file location!" << std::endl;
                return;
            }
            std::string inputMeshFilePosition = "./input.obj";
            if (bar) {
                inputMeshFilePosition = args::get(bar);
            } else {
                cout << "No input mesh file location!" << std::endl;
                return;
            }
            std::string outputFileLocation = "./output";
            if (tar) {
                outputFileLocation = args::get(tar);
            } else {
                cout << "No output file location!" << std::endl;
                return;
            }

            transfer_model(inputFilePosition, inputMeshFilePosition,
                           outputFileLocation, isUsingInsertedEngine,
                           isUsingGUI);
        });
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
}