#pragma once

#include <memory>

#include <ocs2_pros_interface/ProsPreComputation.h>
#include <ocs2_self_collision/SelfCollisionConstraint.h>

namespace ocs2
{
    namespace pros
    {

        class MobileManipulatorSelfCollisionConstraint final : public SelfCollisionConstraint
        {
        public:
            MobileManipulatorSelfCollisionConstraint(const PinocchioStateInputMapping<scalar_t> &mapping,
                                                     PinocchioGeometryInterface pinocchioGeometryInterface, scalar_t minimumDistance)
                : SelfCollisionConstraint(mapping, std::move(pinocchioGeometryInterface), minimumDistance) {}
            ~MobileManipulatorSelfCollisionConstraint() override = default;
            MobileManipulatorSelfCollisionConstraint(const MobileManipulatorSelfCollisionConstraint &other) = default;
            MobileManipulatorSelfCollisionConstraint *clone() const { return new MobileManipulatorSelfCollisionConstraint(*this); }

            const PinocchioInterface &getPinocchioInterface(const PreComputation &preComputation) const override
            {
                return cast<MobileManipulatorPreComputation>(preComputation).getPinocchioInterface();
            }
        };

    } // namespace mobile_manipulator
} // namespace ocs2
