#include "ARDL/Visualization/MFTVisualizer.hpp"

#include "ARDL/Util/ModelPath.hpp"
int main(int argc, char **argv) {
    // VisualizerBase vis("/home/joshua/Projects/Projects/PhD/DL_ws/src/DynamicsLib/tests/models/STL/model.urdf");
    // VisualizerBase vis("/home/joshua/Projects/Projects/PhD/DL_ws/src/DynamicsLib/tests/models/kuka-lwr-4plus/model.urdf");
    std::string urdf, json;
    std::vector<std::string> jsons;
    if (argc == 3) {
        urdf = std::string(argv[1]);
    urdf= ARDL::Util::getModelFromGazeboPath(urdf);
        json = std::string(argv[2]);
        std::cout << urdf << std::endl;
        std::cout << json << std::endl;
        MFTVisualizer vis(json, urdf);
        vis.run();
    } else {
        urdf = std::string(argv[1]);
    urdf= ARDL::Util::getModelFromGazeboPath(urdf);
        for (size_t i = 2; i < argc; i++) {
            jsons.push_back(std::string(argv[i]));
        }

        MFTVisualizer vis(jsons, urdf);
        vis.run();
    }
    // MFTVisualizer vis("/home/joshua/Projects/Projects/PhD/DL_ws/src/ARDL/build/outputtest.json", "/home/joshua/Projects/Projects/PhD/DL_ws/src/DynamicsLib/tests/models/STL/model.urdf");

    return 0;
}
