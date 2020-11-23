#pragma once
// #define EIGEN_USE_BLAS ON
#define EIGEN_RUNTIME_NO_MALLOC ON
// #define ARDL_NO_VARIANT ON
#include "robotTestMain.cpp"
#include "Math/common.hpp"
#include "ARDL/Util/Logger.hpp"
#include "ARDL/Util/Timer.hpp"

using namespace ARDL::Util;
using namespace Eigen;
// size_t iters = 1000;

TEST_CASE("Initializations", "[adjoint][math]") {
    SECTION("X") {
        Eigen::Matrix3d R = createRotationMatrix(0.25 * M_PI, 0 * M_PI, 0 * M_PI);
        Eigen::Vector3d p;
        p << 0, 0, 0;
        Pose<double> ad(R, p);
        checkApproxMatrix(ad.getR(), R);
    }
    SECTION("Y") {}
    SECTION("Z") {}
    SECTION("MIX") {}
}

TEST_CASE("Adjoint applyInverseTo", "[adjoint][math]") {
    //Init
    Eigen::Matrix3d R = createRotationMatrix(0.25 * M_PI, 0 * M_PI, 0 * M_PI);
    Eigen::Vector3d p;
    p << 0, 0, 0;
    SECTION("Identity Test") {
        //Result
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Vector3d zero = Vector3d::Zero();
        Pose<double> ad(R, p), adInv(R, p), result;
        adInv.applyInverseTo(ad, result);
        checkApproxMatrix(result.getR(), I, 1e-10);
        checkApproxMatrix(result.getP(), zero, 1e-10);
    }
    SECTION("Rotation Test") {
        Pose<double> ad(R, p), ad2(R, p);
        Eigen::Matrix3d resR = createRotationMatrix(-0.5 * M_PI, 0, 0);
        Vector3d resP = Vector3d::Zero();
        ad.inverse();
        ad2.applyInverse(ad, ad);
        checkApproxMatrix(ad.getR(), resR, 1e-10);
        checkApproxMatrix(ad.getP(), resP, 1e-10);
    }
    SECTION("Identity Test 1") {
        //Result
      p<<0.1,0.5,0;
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Vector3d zero = Vector3d::Zero();
        Pose<double> ad(R, p), adInv(R, p), result;
        adInv.applyInverseTo(ad, result);
        std::cout<<result.getP().transpose()<<std::endl;
        checkApproxMatrix(result.getR(), I, 1e-10);
        checkApproxMatrix(result.getP(), zero, 1e-10);
    }
    SECTION("Identity Test 2") {
        //Result
      p<<0.1,0.5,0;
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Vector3d zero = Vector3d::Zero();
        Pose<double> ad(R, p), adInv(R, p), result;
        adInv.inverse();
        adInv.applyTo(ad, result);
        std::cout<<result.getP().transpose()<<std::endl;
        checkApproxMatrix(result.getR(), I, 1e-10);
        checkApproxMatrix(result.getP(), zero, 1e-10);
    }
    SECTION("Identity Test 3") {
        //Result
      p<<0.1,0.5,0;
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Vector3d zero = Vector3d::Zero();
        Pose<double> ad(R, p), adInv(R, p), result;
        adInv.applyInverse(ad, result);
        std::cout<<result.getP().transpose()<<std::endl;
        checkApproxMatrix(result.getR(), I, 1e-10);
        checkApproxMatrix(result.getP(), zero, 1e-10);
    }
    SECTION("Identity Test 4") {
        //Result
      p<<0.1,0.5,0;
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Vector3d zero = Vector3d::Zero();
        Pose<double> ad(R, p), adInv(R, p);
        adInv.inverse();
        Eigen::Matrix<double,6,6> result = ad.getMatrix()*adInv.getMatrix();
        Eigen::Matrix<double,6,6> iden;
        iden.setIdentity();
        checkApproxMatrix(result, iden, 1e-10);
        result = adInv.getMatrix()*ad.getMatrix();
        checkApproxMatrix(result, iden, 1e-10);
    }
}
TEST_CASE("Adjoint apply", "[adjoint][math]") {
    //Init
    Eigen::Matrix3d R = createRotationMatrix(0.25 * M_PI, 0 * M_PI, 0 * M_PI);
    Eigen::Vector3d p;
    p << 0, 0, 0;
    //Result
    Eigen::Matrix3d resR = createRotationMatrix(0.5 * M_PI, 0, 0);
    Vector3d resP = Vector3d::Zero();
    Pose<double> ad(R, p), ad2(R, p);
    ad.apply(ad2);
    checkApproxMatrix(ad.getR(), resR, 1e-10);
    checkApproxMatrix(ad.getP(), resP, 1e-10);
}

TEST_CASE("Adjoint inverse", "[adjoint][math]") {
    //Init
    Eigen::Matrix3d R = createRotationMatrix(0.5 * M_PI, 0 * M_PI, 0 * M_PI);
    Eigen::Vector3d p;
    p << 0, 0, 0;
    //Result
    Eigen::Matrix3d resR = createRotationMatrix(-0.5 * M_PI, 0, 0);
    Vector3d resP = Vector3d::Zero();
    Pose<double> ad(R, p);
    ad.inverse();
    checkApproxMatrix(ad.getR(), resR, 1e-10);
    checkApproxMatrix(ad.getP(), resP, 1e-10);
}
 
