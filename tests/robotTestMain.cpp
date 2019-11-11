#pragma once
#define CATCH_CONFIG_RUNNER
#include "catch.hpp"

std::string urdfModel = "";

int main(int argc, char *argv[]) {
    Catch::Session session;
    using namespace Catch::clara;
    auto cli = session.cli() | Opt(urdfModel, "urdfModel")["--urdf"]("The URDF model for the test case");

    session.cli(cli);

    int returnCode = session.applyCommandLine(argc, argv);
    if (returnCode != 0) {
        return returnCode;
    }
    return session.run();
}
