#include "ARDL/Visualization/MFTVisualizer.hpp"

int main(int argc, char **argv) {
    // VisualizerBase vis("/home/joshua/Projects/Projects/PhD/DL_ws/src/DynamicsLib/tests/models/STL/model.urdf");
    // VisualizerBase vis("/home/joshua/Projects/Projects/PhD/DL_ws/src/DynamicsLib/tests/models/kuka-lwr-4plus/model.urdf");
    std::string urdf, json;
    if (argc > 2) {
        urdf = std::string(argv[1]);
        json = std::string(argv[2]);
    }
    std::cout << urdf << std::endl;
    std::cout << json << std::endl;
    MFTVisualizer vis(json, urdf);
    // MFTVisualizer vis("/home/joshua/Projects/Projects/PhD/DL_ws/src/ARDL/build/outputtest.json", "/home/joshua/Projects/Projects/PhD/DL_ws/src/DynamicsLib/tests/models/STL/model.urdf");

    vis.run();

    return 0;
}
