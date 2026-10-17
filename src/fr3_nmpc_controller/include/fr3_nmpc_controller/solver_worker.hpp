#pragma once
#include "fr3_nmpc_controller/optimizer.hpp"
#include <optional>

namespace fr3_nmpc_controller
{
// Interface placeholder only: no thread, solver or shared buffer is started.
// Implement bounded, nonblocking exchange and lifecycle teardown before use.
class SolverWorker
{
public:
  bool ready() const noexcept;
  std::optional<SolverResult> latest() const noexcept;
};
}  // namespace fr3_nmpc_controller
