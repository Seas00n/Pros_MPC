
#pragma once

#include <string>
#include <vector>

#include <urdf_parser/urdf_parser.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_pros_interface/ProsModelInfo.h>

namespace ocs2 {
namespace pros {

/** Create a MobileManipulatorModel PinocchioInterface from a URDF
 * @param [in] robotUrdfPath: The robot URDF path.
 * @param [in] type: Type of robot model (fixed-arm or wheel-based)
 * @return PinocchioInterface
 */
PinocchioInterface createPinocchioInterface(const std::string& robotUrdfPath, const ManipulatorModelType& type);

/** Create a MobileManipulatorModel PinocchioInterface from a URDF
 * @param [in] robotUrdfPath: The robot URDF path.
 * @param [in] type: Type of robot model (fixed-arm or wheel-based)
 * @param [in] jointNames: The joint names from URDF to make fixed/unactuated.
 * @return PinocchioInterface
 */
PinocchioInterface createPinocchioInterface(const std::string& robotUrdfPath, const ManipulatorModelType& type,
                                            const std::vector<std::string>& jointNames);

/**
 * Create a scalar-typed ManipulatorModelInfo.
 * @param [in] interface: Pinocchio interface
 * @param [in] type: Type of template model (default-arm or wheel-based or floating-arm)
 * @param [in] baseFrame: Name of the root frame.
 * @param [in] eeFrame: Name of the end-effector frame.
 * @return ProsModelInfo
 */
ProsModelInfo createProsModelInfo(const PinocchioInterface &interface, const ManipulatorModelType &type,
                                                const std::string &baseFrame, const std::string &eeFrame);

/** Load ManipulatorModelType for a config file */
ManipulatorModelType loadManipulatorType(const std::string &configFilePath, const std::string &fieldName = "manipulatorModelType");

}  // namespace mobile_manipulator
}  // namespace ocs2
