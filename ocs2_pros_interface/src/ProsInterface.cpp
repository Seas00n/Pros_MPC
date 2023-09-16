#include <string>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>

#include "ocs2_pros_interface/FactoryFunction.h"
#include "ocs2_pros_interface/ProsModelInfo.h"
#include "ocs2_pros_interface/ProsInterface.h"
#include "ocs2_pros_interface/ProsPinocchioMapping.h"
#include "ocs2_pros_interface/cost/QuadraticInputCost_Pros.h"
#include "ocs2_pros_interface/constraint/EndEffectorConstraint_Pros.h"
#include "ocs2_pros_interface/constraint/SelfCollisionConstraint_Pros.h"
#include "ocs2_pros_interface/dynamics/DefaultManipulatorDynamics.h"
#include "ocs2_pros_interface/dynamics/FloatingArmManipulatorDynamics.h"
#include "ocs2_pros_interface/dynamics/FullyActuatedFloatingArmManipulatorDynamics.h"
#include "ocs2_pros_interface/dynamics/WheelBasedMobileManipulatorDynamics.h"

#include <ocs2_core/initialization/DefaultInitializer.h>
//loadData记录如何从info中读取数据
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/soft_constraint/StateInputSoftBoxConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_pinocchio_interface/urdf.h>
#include <ocs2_self_collision/SelfCollisionConstraint.h>
#include <ocs2_self_collision/SelfCollisionConstraintCppAd.h>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace ocs2{
namespace pros{
    ProsInterface::ProsInterface(const std::string& taskFile, const std::string& libraryFolder, const std::string& urdfFile){
          // check that task file exists
        boost::filesystem::path taskFilePath(taskFile);
        if (boost::filesystem::exists(taskFilePath)) {
            std::cerr << "[MobileManipulatorInterface] Loading task file: " << taskFilePath << std::endl;
        } else {
            throw std::invalid_argument("[MobileManipulatorInterface] Task file not found: " + taskFilePath.string());
        }
          // check that urdf file exists
        boost::filesystem::path urdfFilePath(urdfFile);
        if (boost::filesystem::exists(urdfFilePath)) {
            std::cerr << "[MobileManipulatorInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
        } else {
            throw std::invalid_argument("[MobileManipulatorInterface] URDF file not found: " + urdfFilePath.string());
        }
        // create library folder if it does not exist
        boost::filesystem::path libraryFolderPath(libraryFolder);
        boost::filesystem::create_directories(libraryFolderPath);
        std::cerr << "[MobileManipulatorInterface] Generated library path: " << libraryFolderPath << std::endl;

          // read the task file
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);

          // 处理task.info中的各种模型信息
          // 1. 读取模型类型：1-Wheel-Based 2-Floating-Based 3-Fully actuated floating-arm
          // Floating: 基座6DoF with world
          // Fully-Actuated: 所有自由度可控
        ManipulatorModelType modelType = loadManipulatorType(taskFile, "model_information.manipulatorModelType");
    
          // 读取需要设置为固定的关节
        std::vector<std::string> removeJointNames;
        loadData::loadStdVector<std::string>(taskFile, "model_information.removeJoints", removeJointNames, false);
          // 读取基座名称和末端执行器名称（wheel base）
        std::string baseFrame, eeFrame;
        loadData::loadPtreeValue<std::string>(pt, baseFrame, "model_information.baseFrame", false);
        loadData::loadPtreeValue<std::string>(pt, eeFrame, "model_information.eeFrame", false);
        
          // pinocchio导入urdf文件并根据ModelType在基座添加或删除关节
        pinocchioInterfacePtr_.reset(new PinocchioInterface(createPinocchioInterface(urdfFile, modelType, removeJointNames)));
        std::cerr << *pinocchioInterfacePtr_;

          // ManipulatorModelInfo
          //使用manipulatorModelInfo_指针获得模型状态数量，输入数量等信息
        manipulatorModelInfo_ = createProsModelInfo(*pinocchioInterfacePtr_, modelType, baseFrame, eeFrame);

        bool usePreComputation  = true;
        bool recompileLibraries = true;
        loadData::loadPtreeValue(pt, usePreComputation, "model_settings.usePreComputation", true);
        loadData::loadPtreeValue(pt, recompileLibraries, "model_settings.recompileLibraries", true);

          // Default initial state
        initialState_.setZero(manipulatorModelInfo_.stateDim);
          // For FA Floating, baseStateDim = StateDim-ArmDim=6
        const int baseStateDim = manipulatorModelInfo_.stateDim - manipulatorModelInfo_.armDim;
        const int armStateDim  = manipulatorModelInfo_.armDim;
        
          // arm base DOFs initial state
        if (baseStateDim > 0) {
            vector_t initialBaseState = vector_t::Zero(baseStateDim);
            loadData::loadEigenMatrix(taskFile, "initialState.base." + modelTypeEnumToString(modelType), initialBaseState);
            initialState_.head(baseStateDim) = initialBaseState;
        }

          // arm joints DOFs velocity limits
        vector_t initialArmState = vector_t::Zero(armStateDim);
        loadData::loadEigenMatrix(taskFile, "initialState.arm", initialArmState);
        initialState_.tail(armStateDim) = initialArmState;

          // DDP-MPC settings
        ddpSettings_ = ddp::loadSettings(taskFile, "ddp");
        mpcSettings_ = mpc::loadSettings(taskFile, "mpc");

          // Reference Manager
        referenceManagerPtr_.reset(new ReferenceManager);

          /*
        * Optimal control problem
        */
          // Optimal Control Problem
          // CostPtr使得可以进行增加cost任务
        problem_.costPtr->add("inputCost", getQuadraticInputCost(taskFile));
        
          // set Constraints
          // joint limits constraint
        problem_.softConstraintPtr->add("jointLimits", getJointLimitSoftConstraint(*pinocchioInterfacePtr_, taskFile));
          // end-effector state constraint
        problem_.stateSoftConstraintPtr->add("endEffector", getEndEffectorConstraint(*pinocchioInterfacePtr_, taskFile, "endEffector",
                                                                               usePreComputation, libraryFolder, recompileLibraries));
        problem_.finalSoftConstraintPtr->add("finalEndEffector", getEndEffectorConstraint(*pinocchioInterfacePtr_, taskFile, "finalEndEffector",
                                                                                    usePreComputation, libraryFolder, recompileLibraries));
          // self-collision avoidance constraint
        bool activateSelfCollision = true;
        loadData::loadPtreeValue(pt, activateSelfCollision, "selfCollision.activate", true);
        if (activateSelfCollision) {
            problem_.stateSoftConstraintPtr->add(
                "selfCollision", getSelfCollisionConstraint(*pinocchioInterfacePtr_, taskFile, urdfFile, "selfCollision", usePreComputation,
                                                    libraryFolder, recompileLibraries));
        }

        // Dynamics
        switch (manipulatorModelInfo_.manipulatorModelType)
        {
        case ManipulatorModelType::DefaultManipulator:
        {
          problem_.dynamicsPtr.reset(
              new DefaultManipulatorDynamics(manipulatorModelInfo_, "dynamics", libraryFolder, recompileLibraries, true));
          break;
        }
        case ManipulatorModelType::FloatingArmManipulator:
        {
          problem_.dynamicsPtr.reset(
              new FloatingArmManipulatorDynamics(manipulatorModelInfo_, "dynamics", libraryFolder, recompileLibraries, true));
          break;
        }
        case ManipulatorModelType::FullyActuatedFloatingArmManipulator:
        {
          problem_.dynamicsPtr.reset(
              new FullyActuatedFloatingArmManipulatorDynamics(manipulatorModelInfo_, "dynamics", libraryFolder, recompileLibraries, true));
          break;
        }
        case ManipulatorModelType::WheelBasedMobileManipulator:
        {
          problem_.dynamicsPtr.reset(
              new WheelBasedMobileManipulatorDynamics(manipulatorModelInfo_, "dynamics", libraryFolder, recompileLibraries, true));
          break;
        }
        default:
          throw std::invalid_argument("Invalid manipulator model type provided.");
        }
        /*
         * Pre-computation
         */
        if (usePreComputation)
        {
          problem_.preComputationPtr.reset(new MobileManipulatorPreComputation(*pinocchioInterfacePtr_, manipulatorModelInfo_));
        }

        // Rollout
        const auto rolloutSettings = rollout::loadSettings(taskFile, "rollout");
        rolloutPtr_.reset(new TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings));

        // Initialization
        initializerPtr_.reset(new DefaultInitializer(manipulatorModelInfo_.inputDim));
    }

    std::unique_ptr<StateInputCost> ProsInterface::getQuadraticInputCost(const std::string& taskFile) {
        matrix_t R                = matrix_t::Zero(manipulatorModelInfo_.inputDim, manipulatorModelInfo_.inputDim);
        const    int baseInputDim = manipulatorModelInfo_.inputDim - manipulatorModelInfo_.armDim;
        const    int armStateDim  = manipulatorModelInfo_.armDim;

          // arm base DOFs input costs
        if (baseInputDim > 0) {
            matrix_t R_base = matrix_t::Zero(baseInputDim, baseInputDim);
            loadData::loadEigenMatrix(taskFile, "inputCost.R.base." + modelTypeEnumToString(manipulatorModelInfo_.manipulatorModelType), R_base);
            R.topLeftCorner(baseInputDim, baseInputDim) = R_base;
        }

          // arm joints DOFs input costs
        matrix_t R_arm = matrix_t::Zero(armStateDim, armStateDim);
        loadData::loadEigenMatrix(taskFile, "inputCost.R.arm", R_arm);
        R.bottomRightCorner(armStateDim, armStateDim) = R_arm;

        return std::make_unique<QuadraticInputCost>(std::move(R), manipulatorModelInfo_.stateDim);
    }
    /// @brief 
    /// @param pinocchioInterface 
    /// @param taskFile 
    /// @return 
    std::unique_ptr<StateInputCost> ProsInterface::getJointLimitSoftConstraint(const PinocchioInterface& pinocchioInterface,
                                                                                        const std::string& taskFile) {

        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);

        bool activateJointPositionLimit = true;
        bool activateJointVelocityLimit = true;
        loadData::loadPtreeValue(pt, activateJointPositionLimit, "jointPositionLimits.activate", true);

        const int baseStateDim = manipulatorModelInfo_.stateDim - manipulatorModelInfo_.armDim;
        const int armStateDim  = manipulatorModelInfo_.armDim;
        const int baseInputDim = manipulatorModelInfo_.inputDim - manipulatorModelInfo_.armDim;
        const int armInputDim  = manipulatorModelInfo_.armDim;
        const auto& model      = pinocchioInterface.getModel();

            // Load position limits
        std::vector<StateInputSoftBoxConstraint::BoxConstraint> stateLimits;
        if (activateJointPositionLimit) {
            scalar_t muPositionLimits    = 1e-2;
            scalar_t deltaPositionLimits = 1e-3;

                  // arm joint DOF limits from the parsed URDF
            const vector_t lowerBound = model.lowerPositionLimit.tail(armStateDim);
            const vector_t upperBound = model.upperPositionLimit.tail(armStateDim);

            loadData::loadPtreeValue(pt, muPositionLimits, "jointPositionLimits.mu", true);
            loadData::loadPtreeValue(pt, deltaPositionLimits, "jointPositionLimits.delta", true);

            stateLimits.reserve(armStateDim);
            for (int i = 0; i < armStateDim; ++i) {
                StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
                boxConstraint.index      = baseStateDim + i;
                boxConstraint.lowerBound = lowerBound(i);
                boxConstraint.upperBound = upperBound(i);
                boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muPositionLimits, deltaPositionLimits}));
                stateLimits.push_back(std::move(boxConstraint));
            }
        }

              // load velocity limits
        std::vector<StateInputSoftBoxConstraint::BoxConstraint> inputLimits;
        if(activateJointVelocityLimit){
            vector_t lowerBound          = vector_t::Zero(manipulatorModelInfo_.inputDim);
            vector_t upperBound          = vector_t::Zero(manipulatorModelInfo_.inputDim);
            scalar_t muVelocityLimits    = 1e-2;
            scalar_t deltaVelocityLimits = 1e-3;

                  // Base DOFs velocity limits
            if (baseInputDim > 0) {
                vector_t lowerBoundBase = vector_t::Zero(baseInputDim);
                vector_t upperBoundBase = vector_t::Zero(baseInputDim);
                loadData::loadEigenMatrix(taskFile,
                                "jointVelocityLimits.lowerBound.base." + modelTypeEnumToString(manipulatorModelInfo_.manipulatorModelType),
                                lowerBoundBase);
                loadData::loadEigenMatrix(taskFile,
                                "jointVelocityLimits.upperBound.base." + modelTypeEnumToString(manipulatorModelInfo_.manipulatorModelType),
                                upperBoundBase);
                lowerBound.head(baseInputDim) = lowerBoundBase;
                upperBound.head(baseInputDim) = upperBoundBase;
            }

                  // arm joint DOFs velocity limits
            vector_t lowerBoundArm = vector_t::Zero(armInputDim);
            vector_t upperBoundArm = vector_t::Zero(armInputDim);
            loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.lowerBound.arm", lowerBoundArm);
            loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.upperBound.arm", upperBoundArm);
            lowerBound.tail(armInputDim) = lowerBoundArm;
            upperBound.tail(armInputDim) = upperBoundArm;
            loadData::loadPtreeValue(pt, muVelocityLimits, "jointVelocityLimits.mu", true);
            loadData::loadPtreeValue(pt, deltaVelocityLimits, "jointVelocityLimits.delta", true);

            inputLimits.reserve(manipulatorModelInfo_.inputDim);
            for (int i = 0; i < manipulatorModelInfo_.inputDim; ++i) {
                StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
                boxConstraint.index      = i;
                boxConstraint.lowerBound = lowerBound(i);
                boxConstraint.upperBound = upperBound(i);
                boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muVelocityLimits, deltaVelocityLimits}));
                inputLimits.push_back(std::move(boxConstraint));
            }
        }

    auto boxConstraints = std::make_unique<StateInputSoftBoxConstraint>(stateLimits, inputLimits);
    boxConstraints->initializeOffset(0.0, vector_t::Zero(manipulatorModelInfo_.stateDim), vector_t::Zero(manipulatorModelInfo_.inputDim));
    return boxConstraints;
    }

    /// @brief 
    /// @param pinocchioInterface 
    /// @param taskFile 
    /// @param prefix 
    /// @param usePreComputation 
    /// @param libraryFolder 
    /// @param recompileLibraries 
    /// @return 
    std::unique_ptr<StateCost> ProsInterface::getEndEffectorConstraint(const PinocchioInterface &pinocchioInterface,
                                                                                    const std::string &taskFile, const std::string &prefix,
                                                                                    bool usePreComputation, const std::string &libraryFolder,
                                                                                    bool recompileLibraries)
    {
      scalar_t muPosition = 1.0;
      scalar_t muOrientation = 1.0;
      const std::string name = "toe";

      boost::property_tree::ptree pt;
      boost::property_tree::read_info(taskFile, pt);
      loadData::loadPtreeValue(pt, muPosition, prefix + ".muPosition", true);
      loadData::loadPtreeValue(pt, muOrientation, prefix + ".muOrientation", true);
      if (referenceManagerPtr_ == nullptr)
      {
        throw std::runtime_error("[getEndEffectorConstraint] referenceManagerPtr_ should be set first!");
      }

      std::unique_ptr<StateConstraint> constraint;
      if (usePreComputation)
      {
        MobileManipulatorPinocchioMapping pinocchioMapping(manipulatorModelInfo_);//pinocchioMapping 会存储ModelInformation, 同时提供
        PinocchioEndEffectorKinematics eeKinematics(pinocchioInterface, pinocchioMapping, {manipulatorModelInfo_.eeFrame});
        constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_));
      }
      else
      {
        MobileManipulatorPinocchioMappingCppAd pinocchioMappingCppAd(manipulatorModelInfo_);
        PinocchioEndEffectorKinematicsCppAd eeKinematics(pinocchioInterface, pinocchioMappingCppAd, {manipulatorModelInfo_.eeFrame},
                                                         manipulatorModelInfo_.stateDim, manipulatorModelInfo_.inputDim,
                                                         "end_effector_kinematics", libraryFolder, recompileLibraries, false);
        constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_));
      }

      std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(6);
      std::generate_n(penaltyArray.begin(), 3, [&]
                      { return std::make_unique<QuadraticPenalty>(muPosition); });
      std::generate_n(penaltyArray.begin() + 3, 3, [&]
                      { return std::make_unique<QuadraticPenalty>(muOrientation); });

      return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penaltyArray));
    }
    /// @brief 
    /// @param pinocchioInterface 
    /// @param taskFile 
    /// @param urdfFile 
    /// @param prefix 
    /// @param usePreComputation 
    /// @param libraryFolder 
    /// @param recompileLibraries 
    /// @return 
    std::unique_ptr<StateCost> ProsInterface::getSelfCollisionConstraint(const PinocchioInterface &pinocchioInterface,
                                                                                      const std::string &taskFile, const std::string &urdfFile,
                                                                                      const std::string &prefix, bool usePreComputation,
                                                                                      const std::string &libraryFolder,
                                                                                      bool recompileLibraries)
    {
      std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
      std::vector<std::pair<std::string, std::string>> collisionLinkPairs;
      scalar_t mu = 1e-2;
      scalar_t delta = 1e-3;
      scalar_t minimumDistance = 0.0;

      boost::property_tree::ptree pt;
      boost::property_tree::read_info(taskFile, pt);
      // std::cerr << "\n #### SelfCollision Settings: ";
      // std::cerr << "\n #### =============================================================================\n";
      loadData::loadPtreeValue(pt, mu, prefix + ".mu", true);
      loadData::loadPtreeValue(pt, delta, prefix + ".delta", true);
      loadData::loadPtreeValue(pt, minimumDistance, prefix + ".minimumDistance", true);
      loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionObjectPairs", collisionObjectPairs, true);
      loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionLinkPairs", collisionLinkPairs, true);
      // std::cerr << " #### =============================================================================\n";

      PinocchioGeometryInterface geometryInterface(pinocchioInterface, collisionLinkPairs, collisionObjectPairs);

      const size_t numCollisionPairs = geometryInterface.getNumCollisionPairs();
      std::cerr << "SelfCollision: Testing for " << numCollisionPairs << " collision pairs\n";

      std::unique_ptr<StateConstraint> constraint;
      if (usePreComputation)
      {
        constraint = std::make_unique<MobileManipulatorSelfCollisionConstraint>(MobileManipulatorPinocchioMapping(manipulatorModelInfo_),
                                                                                std::move(geometryInterface), minimumDistance);
      }
      else
      {
        constraint = std::make_unique<SelfCollisionConstraintCppAd>(
            pinocchioInterface, MobileManipulatorPinocchioMapping(manipulatorModelInfo_), std::move(geometryInterface), minimumDistance,
            "self_collision", libraryFolder, recompileLibraries, false);
      }

      auto penalty = std::make_unique<RelaxedBarrierPenalty>(RelaxedBarrierPenalty::Config{mu, delta});

      return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penalty));
    }
}
}