//
// Created by jh on 24. 11. 29.
//


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
    auto ground = world.addGround();
    ground->setAppearance("steel");
    auto aliengo = world.addArticulatedSystem(binaryPath.getDirectory() + "..\\rsc\\aliengo\\aliengo.urdf");

    /// aliengo joint PD controller
    Eigen::VectorXd jointNominalConfig(aliengo->getGeneralizedCoordinateDim()), jointVelocityTarget(aliengo->getDOF());
    jointNominalConfig << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
    jointVelocityTarget.setZero();

    Eigen::VectorXd jointPgain(aliengo->getDOF()), jointDgain(aliengo->getDOF());
    jointPgain.tail(12).setConstant(100.0);
    jointDgain.tail(12).setConstant(1.0);

    aliengo->setGeneralizedCoordinate(jointNominalConfig);
    aliengo->setGeneralizedForce(Eigen::VectorXd::Zero(aliengo->getDOF()));
    aliengo->setPdGains(jointPgain, jointDgain);
    aliengo->setPdTarget(jointNominalConfig, jointVelocityTarget);
    aliengo->setName("aliengo");

    /// launch raisim server
    raisim::RaisimServer server(&world);
    server.focusOn(aliengo);
    server.launchServer();

    for (int i=0; i<2000000; i++) {
        raisim::MSLEEP(1);
        server.integrateWorldThreadSafe();
    }

    std::cout<<"total mass "<<aliengo->getCompositeMass()[0]<<std::endl;

    server.killServer();
}
