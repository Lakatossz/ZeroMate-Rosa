// ---------------------------------------------------------------------------------------------------------------------
/// \file coprocessor.cpp
/// \date 29. 05. 2023
/// \author Jakub Silhavy (jakub.silhavy.cz@gmail.com)
///
/// \brief This file implements the common functionality of a general interface of a coprocessor.
// ---------------------------------------------------------------------------------------------------------------------

// Project file imports

#include "coprocessor.hpp"

#include "../arm1176jzf_s/core.hpp"

namespace zero_mate::coprocessor
{
    ICoprocessor::ICoprocessor(arm1176jzf_s::CCPU_Context& cpu_context)
    : m_cpu_context{ cpu_context }
    {
    }

    void ICoprocessor::Set_CPU_Core(std::weak_ptr<arm1176jzf_s::CCPU_Core> cpu_core)
    {
        m_cpu_core = cpu_core;
    }

} // namespace zero_mate::coprocessor