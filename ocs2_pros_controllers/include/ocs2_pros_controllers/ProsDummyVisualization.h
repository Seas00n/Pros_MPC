#pragma once

#include <robot_state_publisher/robot_state_publisher.h>
#include <tf/transform_broadcaster.h>

#include <ocs2_ros_interfaces/mrt/DummyObserver.h>

#include <ocs2_pros_interface/ProsModelInfo.h>
#include <ocs2_pros_interface/ProsInterface.h>
#include <ocs2_self_collision_visualization/GeometryInterfaceVisualization.h>

namespace ocs2
{
    namespace pros
    {

        class MobileManipulatorDummyVisualization final : public DummyObserver
        {
        public:
            MobileManipulatorDummyVisualization(ros::NodeHandle &nodeHandle, const ProsInterface &interface)
                : pinocchioInterface_(interface.getPinocchioInterface()), modelInfo_(interface.getManipulatorModelInfo())
            {
                launchVisualizerNode(nodeHandle);
            }

            ~MobileManipulatorDummyVisualization() override = default;

            void update(const SystemObservation &observation, const PrimalSolution &policy, const CommandData &command) override;

        private:
            void launchVisualizerNode(ros::NodeHandle &nodeHandle);

            void publishObservation(const ros::Time &timeStamp, const SystemObservation &observation);
            void publishTargetTrajectories(const ros::Time &timeStamp, const TargetTrajectories &targetTrajectories);
            void publishOptimizedTrajectory(const ros::Time &timeStamp, const PrimalSolution &policy);

            PinocchioInterface pinocchioInterface_;
            const ProsModelInfo modelInfo_;
            std::vector<std::string> removeJointNames_;

            std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;
            tf::TransformBroadcaster tfBroadcaster_;

            ros::Publisher stateOptimizedPublisher_;
            ros::Publisher stateOptimizedPosePublisher_;

            std::unique_ptr<GeometryInterfaceVisualization> geometryVisualization_;
        };

    } // namespace mobile_manipulator
} // namespace ocs2
