//////////////////////////////////////////////////////////////////////////////////////////////
/// \file RotationEvaluator.hpp
///
/// Based on Sean Anderson's TransformEvaluator.hpp
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_ROTATION_EVALUATOR_HPP
#define STEAM_ROTATION_EVALUATOR_HPP

#include <Eigen/Core>

#include <steam/evaluator/blockauto/BlockAutomaticEvaluator.hpp>
#include <steam/state/LieGroupStateVar.hpp>
#include <steam/state/VectorSpaceStateVar.hpp>
#include <steam/common/Time.hpp>

namespace steam {
namespace so3 {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluator for a rotation matrix
//////////////////////////////////////////////////////////////////////////////////////////////
typedef BlockAutomaticEvaluator<lgmath::so3::Rotation, 3, 3> RotationEvaluator;

} // so3
} // steam

#endif // STEAM_ROTATION_EVALUATOR_HPP