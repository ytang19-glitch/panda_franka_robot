#include "fr3_nmpc_controller/solver_worker.hpp"

namespace fr3_nmpc_controller
{
bool SolverWorker::ready() const noexcept { return false; }
std::optional<SolverResult> SolverWorker::latest() const noexcept { return std::nullopt; }
}  // namespace fr3_nmpc_controller
