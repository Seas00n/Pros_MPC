#pragma once
#include <string>
#include <vector>

namespace ocs2 {
namespace pros {

/**
 * @brief Defines various manipulator models.
 */
enum class ManipulatorModelType {
  DefaultManipulator = 0,                   // default model from the parsed URDF directly
  WheelBasedMobileManipulator = 1,          // adds actuatable XY-Yaw joints to the model parsed from URDF
  FloatingArmManipulator = 2,               // adds dummy XYZ-RPY joints to the model parsed from URDF
  FullyActuatedFloatingArmManipulator = 3,  // adds actuatable XYZ-RPY joints to the model parsed from URDF
};

/**
 * @brief A data structure to store manipulator information.
 *
 * The attributes are filled by resolving the URDF model parsed.
 */
struct ProsModelInfo {
  ManipulatorModelType manipulatorModelType;  // type of manipulator: floating-base, fully-actuated floating-base, wheel-base, default
  size_t stateDim;                            // number of states needed to define the system flow map 控制方程的状态
  size_t inputDim;                            // number of inputs needed to define the system flow map 控制方程的输入
  size_t armDim;                              // number of DOFs in the robot arm 机构自由度
  std::string baseFrame;                      // name of the root frame of the robot
  std::string eeFrame;                        // name of the end-effector frame of the robot
  std::vector<std::string> dofNames;          // name of the actuated DOFs in the robot
};

/**
 * @brief Returns a string for a ManipulatorModelType for retrieving data from a .info file
 */
static std::string modelTypeEnumToString(ManipulatorModelType manipulatorModelType) {
  std::string manipulatorModelTypeString;

  switch (manipulatorModelType) {
    case ManipulatorModelType::DefaultManipulator: {
      manipulatorModelTypeString = "defaultManipulator";
      break;
    }
    case ManipulatorModelType::FloatingArmManipulator: {
      manipulatorModelTypeString = "floatingArmManipulator";
      break;
    }
    case ManipulatorModelType::FullyActuatedFloatingArmManipulator: {
      manipulatorModelTypeString = "fullyActuatedFloatingArmManipulator";
      break;
    }
    case ManipulatorModelType::WheelBasedMobileManipulator: {
      manipulatorModelTypeString = "wheelBasedMobileManipulator";
      break;
    }
    default:
      throw std::invalid_argument("Invalid manipulator model type provided.");
      break;
  }

  return manipulatorModelTypeString;
}

}  // namespace mobile_manipulator
}  // namespace ocs2