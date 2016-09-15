#include <fcl/fcl.h>

typedef fcl::detail::GJKSolver_libccd<double> NarrowPhaseSolver;
// Instantiating this function causes lots of warnings, so we keep it in this separate file.
namespace fcl {

template detail::CollisionFunctionMatrix<NarrowPhaseSolver>& getCollisionFunctionLookTable< NarrowPhaseSolver>();

}  // namespace fcl
