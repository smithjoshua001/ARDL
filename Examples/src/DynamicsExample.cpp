
#include <ARDL/Util/ModelPath.hpp>
#include <ARDL/Dynamics/DynamicsTree.hpp>
#include <string>

int main(int argc, const char **argv) {
    using T = double;
    using namespace ARDL::Model;
    using namespace ARDL;
    std::string inputRobot;
    std::string urdfModelPath;
    if(argc<2){
        return -1;
    }
    inputRobot=std::string(argv[1]);
    urdfModelPath= ARDL::Util::getModelFromGazeboPath(inputRobot);
    std::shared_ptr<Tree<T>> tree;
    std::shared_ptr<ForwardKinematicsTree<T>> fk;
    std::shared_ptr<DynamicsTree<T>> dyn;
    tree = std::shared_ptr<Tree<T> >(new Tree<T>(urdfModelPath));
    fk = std::shared_ptr<ForwardKinematicsTree<T> >(new ForwardKinematicsTree<T>(tree));
    dyn = std::shared_ptr<DynamicsTree<T> >(new DynamicsTree<T>(tree));

    size_t dof = tree->getNumOfJoints();
    std::cout <<"ARDL DOF: " <<dof << std::endl;
    VectorX<T> q(dof), qd(dof), qdd(dof);

    aligned_vector<Pose<T>> poses;
    aligned_vector<Motion<T>> vels;
    aligned_vector<Jacobian<T>> jacobians, jacobianDots;
    aligned_vector<aligned_vector<Jacobian<T>>> jacobiansDq, jacobianDotsDq;

    Eigen::VectorXd G(dof), GP(dof), CP(dof), MP(dof);
    Eigen::MatrixXd M(dof, dof), C(dof, dof), CMP(dof, dof);

    q.setZero();
    qd.setZero();
    qdd.setZero();
    qdd.setRandom();
    qdd*=0.5;

    poses.resize(dof + 1);
    vels.resize(dof);

    ARDL::Util::init(jacobians, tree->getNumOfJoints());
    ARDL::Util::init(jacobianDots, tree->getNumOfJoints());
    ARDL::Util::init(jacobiansDq, tree->getNumOfJoints());
    ARDL::Util::init(jacobianDotsDq, tree->getNumOfJoints());
    ARDL::Util::init(M, tree->getNumOfJoints());
    ARDL::Util::init(C, tree->getNumOfJoints());
    ARDL::Util::init(G, tree->getNumOfJoints());

    tree->random();
    q = tree->getQ();
    qd= tree->getQd();
    tree->updateMatricesOptim();
    fk->getPosesOptim<Frame::SPATIAL>(poses);
    fk->getJacobians<Frame::SPATIAL>(poses, jacobians);
    dyn->calcGravityVectorOptim<Frame::SPATIAL>(jacobians, poses, G);
    std::cout << "ARDL G: " << G.transpose() << std::endl;


    fk->getVelocities(vels, jacobians);
    fk->getJacobianDots<Frame::SPATIAL>(poses, vels, jacobians,
                                        jacobianDots);
    dyn->calcCoriolisMatrixOptim(jacobians, jacobianDots, vels, poses, C);

    std::cout << "ARDL C*qd: " << (C * qd).transpose() << std::endl;

    dyn->calcJointInertiaMatrixOptim<Frame::SPATIAL>(poses, jacobians, M);

    std::cout << "ARDL M*qdd: " << (M * qdd).transpose() << std::endl;

    MatrixX<double> regressor(dof, dof * 10);
    VectorX<double> params(dof * 10);
    tree->getParams(params);
    dyn->calcSlotineLiRegressor(qd, qdd, poses, vels, jacobians,
                                jacobianDots, regressor);
    std::cout <<"Full torque: "<< (M * qdd + C * qd + G).transpose() << std::endl;
    std::cout <<"Regressor torque: "<< (regressor * params).transpose() << std::endl;
    dyn->calcBaseProjection(50000, M_PI, 1e-5);

    std::cout << "Num of base parameters: "<<dyn->getNumOfBaseParams() << std::endl;
    MatrixX<double> Yb;
    Yb.resize(tree->getNumOfJoints(), dyn->getNumOfBaseParams());

    VectorX<double> baseParams = dyn->getParameterProjector() * params;
    Yb = regressor * dyn->getRegressorProjector();

    std::cout <<"Base regressor torque: "<< (Yb * baseParams).transpose() << std::endl;


    return 0;
}