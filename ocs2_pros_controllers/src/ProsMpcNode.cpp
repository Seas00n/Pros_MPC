#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>


#include <ocs2_pros_interface/ProsInterface.h>

using namespace ocs2;
using namespace pros;
int main(int argc, char **argv){
    const std::string robotName = "pros";

    // Initialize ros node
    ros::init(argc, argv, robotName + "_mpc");
    ros::NodeHandle nodeHandle;
    // Get node parameters
    std::string taskFile, libFolder, urdfFile;
    nodeHandle.getParam("/taskFile", taskFile);
    nodeHandle.getParam("/libFolder", libFolder);
    nodeHandle.getParam("/urdfFile", urdfFile);
    std::cerr << "Loading task file: " << taskFile << std::endl;
    std::cerr << "Loading library folder: " << libFolder << std::endl;
    std::cerr << "Loading urdf file: " << urdfFile << std::endl;
    // Robot interface
    std::cerr << "=========================RobotInterface=========================" << std::endl;
    ProsInterface interface(taskFile, libFolder, urdfFile);
    auto rosReferenceManagerPtr = std::make_shared<ocs2::RosReferenceManager>(robotName, interface.getReferenceManagerPtr());
    rosReferenceManagerPtr->subscribe(nodeHandle);
    // MPC
    std::cerr << "=========================MPC=========================" << std::endl;
    ocs2::GaussNewtonDDP_MPC mpc(interface.mpcSettings(), interface.ddpSettings(), interface.getRollout(),
                                 interface.getOptimalControlProblem(), interface.getInitializer());
    std::cerr << "========================setReferenceManager==========================" << std::endl;
    mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

    // Launch MPC ROS node
    std::cerr << "========================mpcROSInterface==========================" << std::endl;
    MPC_ROS_Interface mpcNode(mpc, robotName);
    std::cerr << "========================mpcNodeLaunch==========================" << std::endl;
    mpcNode.launchNodes(nodeHandle);
    return 0;
}