#pragma once
#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pros_interface/ProsModelInfo.h>

namespace ocs2{
namespace pros{

class ProsInterface final:public RobotInterface{
    public:
        ProsInterface(const std::string& taskFile, const std::string& libraryFolder, const std::string& urdfFile);
        const vector_t& getInitialState() { return initialState_; }

        ddp::Settings& ddpSettings() { return ddpSettings_; }

        mpc::Settings& mpcSettings() { return mpcSettings_; }

        const OptimalControlProblem& getOptimalControlProblem() const override { return problem_; }

        std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }

        const Initializer& getInitializer() const override { return *initializerPtr_; }

        const RolloutBase& getRollout() const { return *rolloutPtr_; }

        const PinocchioInterface& getPinocchioInterface() const { return *pinocchioInterfacePtr_; }

        const ProsModelInfo& getManipulatorModelInfo() const { return manipulatorModelInfo_; }
    private:
        std::unique_ptr<StateInputCost> getQuadraticInputCost(const std::string& taskFile);
        std::unique_ptr<StateCost> getEndEffectorConstraint(const PinocchioInterface& pinocchioInterface, const std::string& taskFile,
                                                      const std::string& prefix, bool useCaching, const std::string& libraryFolder,
                                                      bool recompileLibraries);
        std::unique_ptr<StateCost> getSelfCollisionConstraint(const PinocchioInterface& pinocchioInterface, const std::string& taskFile,
                                                        const std::string& urdfFile, const std::string& prefix, bool useCaching,
                                                        const std::string& libraryFolder, bool recompileLibraries);
        std::unique_ptr<StateInputCost> getJointLimitSoftConstraint(const PinocchioInterface& pinocchioInterface, const std::string& taskFile);

        ddp::Settings ddpSettings_;
        mpc::Settings mpcSettings_;

        OptimalControlProblem problem_;
        std::shared_ptr<ReferenceManager> referenceManagerPtr_;

        std::unique_ptr<RolloutBase> rolloutPtr_;
        std::unique_ptr<Initializer> initializerPtr_;

        std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
        ProsModelInfo manipulatorModelInfo_;

        vector_t initialState_;
};



}
}