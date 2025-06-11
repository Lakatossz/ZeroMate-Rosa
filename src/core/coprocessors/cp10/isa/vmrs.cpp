// ---------------------------------------------------------------------------------------------------------------------
/// \file vmrs.cpp
/// \date 10. 09. 2023
/// \author Jakub Silhavy (jakub.silhavy.cz@gmail.com)
///
/// \brief This file implements a VMRS instruction (FPU special register to an ARM register).
// ---------------------------------------------------------------------------------------------------------------------

// Project file imports

#include "vmrs.hpp"
#include <iostream>

namespace zero_mate::coprocessor::cp10::isa
{
    CVMRS::CVMRS(std::uint32_t value) noexcept
    : m_value{ value }
    {
        std::cout << "CVMRS\n";
    }

    bool CVMRS::Transfer_To_APSR() const noexcept
    {
        std::cout << "Get_Rt_Idx() == 0b1111U\n";
        return Get_Rt_Idx() == 0b1111U;
    }

    std::uint32_t CVMRS::Get_Rt_Idx() const noexcept
    {
        std::cout << "(m_value >> 12U) & 0b1111U\n";
        return (m_value >> 12U) & 0b1111U;
    }

} // namespace zero_mate::coprocessor::cp10::isa