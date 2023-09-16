#include <ocs2_pros_interface/ProsInterface.h>
#include <ocs2_pros_controllers/ProsDummyVisualization.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <ros/init.h>
#include <ros/package.h>

using namespace ocs2;
using namespace pros;

int main(int argc, char **argv)
{
    const std::string robotName = "pros";

    // Initialize ros node
    ros::init(argc, argv, robotName + "_mrt");
    ros::NodeHandle nodeHandle;
    // Get node parameters
    std::string taskFile, libFolder, urdfFile;
    nodeHandle.getParam("/taskFile", taskFile);
    nodeHandle.getParam("/libFolder", libFolder);
    nodeHandle.getParam("/urdfFile", urdfFile);
    std::cerr << "Loading task file: " << taskFile << std::endl;
    std::cerr << "Loading library folder: " << libFolder << std::endl;
    std::cerr << "Loading urdf file: " << urdfFile << std::endl;
    // Robot Interface
    pros::ProsInterface interface(taskFile, libFolder, urdfFile);

    // MRT
    MRT_ROS_Interface mrt(robotName);
    mrt.initRollout(&interface.getRollout());
    mrt.launchNodes(nodeHandle);

    // Visualization
    auto dummyVisualization = std::make_shared<pros::MobileManipulatorDummyVisualization>(nodeHandle, interface);

    // Dummy MRT
    MRT_ROS_Dummy_Loop dummy(mrt, interface.mpcSettings().mrtDesiredFrequency_, interface.mpcSettings().mpcDesiredFrequency_);
    dummy.subscribeObservers({dummyVisualization});

    // initial state
    SystemObservation initObservation;
    initObservation.state = interface.getInitialState();
    initObservation.input.setZero(interface.getManipulatorModelInfo().inputDim);
    initObservation.time = 0.0;

    // initial command
    vector_t initTarget(7);
    initTarget.head(3) << 1, 0, 1;
    initTarget.tail(4) << Eigen::Quaternion<scalar_t>(1, 0, 0, 0).coeffs();
    const vector_t zeroInput = vector_t::Zero(interface.getManipulatorModelInfo().inputDim);
    const TargetTrajectories initTargetTrajectories({initObservation.time}, {initTarget}, {zeroInput});

    // Run dummy (loops while ros is ok)
    dummy.run(initObservation, initTargetTrajectories);

    // Successful exit
    return 0;
}