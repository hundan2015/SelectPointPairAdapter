#include <igl/opengl/glfw/Viewer.h>
#include <igl/readOFF.h>

Eigen::MatrixXd V;
Eigen::MatrixXi F;

int main(int argc, char *argv[]) {
    // Load a mesh in OFF format
    igl::readOFF("/bunny.off", V, F);

    // Plot the mesh
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    viewer.launch();
}