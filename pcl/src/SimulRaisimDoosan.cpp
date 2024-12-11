// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#if WIN32
#include <timeapi.h>
#endif

int main(int argc, char* argv[]) {
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");

    /// create raisim world
    raisim::World world;
    world.setTimeStep(0.001);

    /// create objects
    world.addGround();
//    auto cartPole = world.addArticulatedSystem(binaryPath.getDirectory() + "..\\rsc\\m1013\\dsr_description2\\m1013.urdf");
    auto cartPole = world.addArticulatedSystem(binaryPath.getDirectory() + "..\\rsc\\m1013\\m1013.urdf");
//    auto cartPole = world.addArticulatedSystem(binaryPath.getDirectory() + "..\\rsc\\doosan\\Assembly_URDF.urdf");

    /// cartPole state
    int pGain = 1000000000;
    int dGain = 100000000;
    Eigen::VectorXd jointNominalConfig(cartPole->getGeneralizedCoordinateDim());
    Eigen::VectorXd jointPosition(cartPole->getGeneralizedCoordinateDim());
    Eigen::VectorXd desiredJointPosition(cartPole->getGeneralizedCoordinateDim());
    Eigen::VectorXd desiredTorque(cartPole->getGeneralizedCoordinateDim());
    Eigen::VectorXd jointVelocity(cartPole->getGeneralizedVelocityDim());
    Eigen::VectorXd desiredJointVelocity(cartPole->getGeneralizedVelocityDim());
    std::cout << cartPole->getGeneralizedCoordinateDim() << std::endl;
    jointNominalConfig.setZero();
    jointNominalConfig << 0.0,0.0,0.0,0.0,0.0,0.0;
    cartPole->setGeneralizedCoordinate(jointNominalConfig);

    desiredJointPosition << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    desiredJointVelocity << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    /// launch raisim server
    raisim::RaisimServer server(&world);
    server.focusOn(cartPole);
    server.launchServer();

    for (int i=0; i<200000; i++) {
        raisim::MSLEEP(1);
        server.integrateWorldThreadSafe();
        for(int idx=0; idx<6; idx++)
        {
            jointPosition[idx] = cartPole->getGeneralizedCoordinate()[idx];
            jointVelocity[idx] = cartPole->getGeneralizedVelocity()[idx];

            desiredTorque[idx] = pGain * (desiredJointPosition[idx] - jointPosition[idx]) + dGain * (desiredJointVelocity[idx] - jointVelocity[idx]);
        }
//        std::cout << std::endl;
        cartPole->setGeneralizedForce(desiredTorque);
    }

    server.killServer();
}
