#include "ARDL/IdentificationTrajectory/MFTOptimizer.hpp"

#include <string>
#include <limits>

#include <pagmo/io.hpp>
#include <pagmo/problem.hpp>
#include <pagmo/types.hpp>
#include <pagmo/algorithm.hpp>
#include <pagmo/algorithms/gaco.hpp>
#include <pagmo/algorithms/cstrs_self_adaptive.hpp>
#include <pagmo/archipelago.hpp>
#include <pagmo/algorithms/nlopt.hpp>
#include <pagmo/algorithms/compass_search.hpp>
#include <pagmo/utils/constrained.hpp>
#include <pagmo/topologies/fully_connected.hpp>
#include <pagmo/islands/fork_island.hpp>

#include <rapidjson/rapidjson.h>

#include <sstream>
using namespace rapidjson;

using namespace pagmo;
int main(int argc, char **argv) {
    // Constructing a problem
    // MFTOptimizer::robotUrdf = "/home/joshua/Projects/Projects/PhD/DL_ws/src/DynamicsLib/tests/models/STL/model.urdf";
    // MFTOptimizer::configJson = "/home/joshua/Projects/Projects/PhD/DL_ws/src/ARDL/ARDL-Trajectory/src/IdentificationTrajectory/test.json";
    // "/home/joshua/Projects/Projects/PhD/DL_ws/src/DynamicsLib/tests/models/STL/model.urdf", "/home/joshua/Projects/Projects/PhD/DL_ws/src/ARDL/ARDL-Trajectory/src/IdentificationTrajectory/test.json"
    // MFTOptimizer tmp = MFTOptimizer("/home/joshua/Projects/Projects/PhD/DL_ws/src/DynamicsLib/tests/models/STL/model.urdf", "/home/joshua/Projects/Projects/PhD/DL_ws/src/ARDL/ARDL-Trajectory/src/IdentificationTrajectory/test2.json");

    // MFTOptimizer tmp = MFTOptimizer("/home/joshua/Projects/Projects/PhD/DL_ws/src/DynamicsLib/tests/models/kuka-lwr-4plus/model.urdf", "/home/joshua/Projects/Projects/PhD/DL_ws/src/ARDL/ARDL-Trajectory/src/IdentificationTrajectory/test2.json");
    std::string urdf = "", config = "", optim = "";
    if (argc > 3) {
        optim = argv[1];
        urdf = argv[2];
        config = argv[3];
    } else {
        return -3;
    }

    Document optimConfig;
    std::ifstream ifs(optim);
    IStreamWrapper isw(ifs);
    optimConfig.ParseStream(isw);
    if (!optimConfig.IsObject()) {
        return -1;
    }
    MFTOptimizer tmp(urdf, config);
    problem p0{tmp};

    algorithm algo;
    algorithm algosub;
    if (optimConfig.HasMember("optimizer")) {
        if (optimConfig["optimizer"].GetString() == std::string("cgaco")) {
            rapidjson::Value gaco = optimConfig["gaco"].GetObject();
            if (!gaco.IsObject()) {
                return -2;
            }
            algosub = algorithm{pagmo::gaco(gaco["generations"].GetInt(), gaco["kernel_size"].GetInt(), gaco["q"].GetDouble(), gaco["oracle"].GetDouble(), gaco["acc"].GetDouble(), gaco["thresh"].GetInt(), gaco["n_gen_mark"].GetInt(), gaco["impstop"].GetInt(), gaco["evalstop"].GetInt(), gaco["focus"].GetDouble(), true)};
            algo = algorithm{cstrs_self_adaptive(optimConfig["iters"].GetInt(), algosub)};
        }
    }
    if (argc > 5) {
        algo.set_verbosity(atof(argv[5]));
    }
    pagmo::archipelago arch{fully_connected(), 6, algo, p0, 40};
    // for (island &isl : arch) {
    //     isl.get_population().get_problem().extract<MFTOptimizer>()->init();
    // }
    // std::cout << "The population: \n" << arch;

    arch.evolve();
    arch.wait_check();
    // std::cout << p0 << std::endl;

    size_t best_solution_index = 0;
    double best_solution_value = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < arch.get_champions_f().size(); i++) {
        Eigen::VectorXd tmp2 = Eigen::Map<Eigen::VectorXd>(arch.get_champions_f()[i].data(), arch.get_champions_f()[i].size());
        std::cout << "i: " << i << " :" << tmp2.transpose() << std::endl;
        if ((tmp2.tail(tmp2.size() - 1).array() <= 0).all() && best_solution_value > arch.get_champions_f()[i][0]) {
            best_solution_index = i;
            best_solution_value = arch.get_champions_f()[i][0];
        }
    }
    std::cout << "COST: " << best_solution_value << std::endl;

    pagmo::archipelago arch2{fully_connected()};

    algorithm local;
    if (optimConfig.HasMember("localoptimizer")) {
        if (optimConfig["localoptimizer"]["type"].GetString() == std::string("compass")) {
            // local = algorithm{nlopt("cobyla")};
            // local.extract<nlopt>()->set_maxeval(10000);

            // local.extract<nlopt>()->set_ftol_abs(1e-3);

            // local.extract<nlopt>()->set_selection("best");
            // local.extract<nlopt>()->set_replacement("best");
            local = algorithm(compass_search{optimConfig["localoptimizer"]["maxEvals"].GetUint(), optimConfig["localoptimizer"]["startRange"].GetDouble(), optimConfig["localoptimizer"]["stopRange"].GetDouble()});
            local.extract<compass_search>()->set_selection("best");
            local.extract<compass_search>()->set_replacement("best");
        }
        if (argc > 6) {
            local.set_verbosity(atof(argv[6]));
        }
        for (size_t i = 0; i < 6; i++) {
            arch2.push_back(island{local, arch[i].get_population()});
        }
        arch2.evolve();
        arch2.wait_check();
    }
    std::cout << arch2 << std::endl;

    best_solution_index = 0;
    best_solution_value = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < arch2.get_champions_f().size(); i++) {
        Eigen::VectorXd tmp2 = Eigen::Map<Eigen::VectorXd>(arch2.get_champions_f()[i].data(), arch2.get_champions_f()[i].size());
        std::cout << "i: " << i << " :" << tmp2.transpose() << std::endl;
        if ((tmp2.tail(tmp2.size() - 1).array() <= 0).all() && best_solution_value > arch2.get_champions_f()[i][0]) {
            best_solution_index = i;
            best_solution_value = arch2.get_champions_f()[i][0];
        }
    }

    std::shared_ptr<ModifiedFourierTrajectory<double> > output = tmp.getMFT();
    Eigen::VectorXd params = Eigen::VectorXd::Zero(arch2.get_champions_x()[0].size() + 1);
    output->getParameters(params);

    for (size_t i = 0; i < arch2.get_champions_f().size(); i++) {
        Eigen::VectorXd tmp2 = Eigen::Map<Eigen::VectorXd>(arch2.get_champions_f()[i].data(), arch2.get_champions_f()[i].size());
        if ((tmp2.tail(tmp2.size() - 1).array() <= 0).all()) {
            Eigen::VectorXd answer = Eigen::Map<Eigen::VectorXd>(arch2.get_champions_x()[i].data(), arch2.get_champions_x()[i].size());
            std::cout << "SOLUTION i " << i << " : " << answer.transpose() << std::endl;
            std::cout << "COST: " << arch2.get_champions_f()[i][0] << std::endl;
            params.tail(answer.size()) = answer;
            output->setFromParameters(params);
            if (argc > 4) {
                std::ostringstream out;
                out.precision(2);
                out << std::string(argv[4]) << i << "_" << std::fixed << arch2.get_champions_f()[i][0] << ".json";
                output->saveToJSON(out.str());
            }
        }
    }
}
