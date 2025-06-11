// ---------------------------------------------------------------------------------------------------------------------
/// \file gpio.cpp
/// \date 28. 05. 2023
/// \author Jakub Silhavy (jakub.silhavy.cz@gmail.com)
///
/// \brief This file implements the GPIO controller used in BCM2835 (defined in gpio.hpp).
///
/// To find more information about this peripheral, please visit
/// https://www.raspberrypi.org/app/uploads/2012/02/BCM2835-ARM-Peripherals.pdf (chapter 6)
// ---------------------------------------------------------------------------------------------------------------------

// STL imports (excluded from Doxygen)
/// \cond
#include <algorithm>
/// \endcond

// 3rd party library includes

#include "fmt/format.h"
#include "magic_enum.hpp"

// Project file imports

#include "gpio.hpp"
#include "igpio.hpp"
#include "zero_mate/utils/math.hpp"
#include "zero_mate/utils/singleton.hpp"

namespace zero_mate::peripheral
{
    CGPIO_Manager::CPin::CPin()
    : m_state{ NState::Low }
    , m_function{ NFunction::Input }
    , m_enabled_interrupts{}
    , m_pending_irq{ false }
    {
    }

    IGPIO_Manager::IPin::NFunction CGPIO_Manager::CPin::Get_Function() const noexcept
    {
        return m_function;
    }

    void CGPIO_Manager::CPin::Set_Function(NFunction function) noexcept
    {
        m_function = function;
    }

    IGPIO_Manager::IPin::NState CGPIO_Manager::CPin::Get_State() const noexcept
    {
        return m_state;
    }

    void CGPIO_Manager::CPin::Set_State(NState state) noexcept
    {
        m_state = state;
    }

    bool CGPIO_Manager::CPin::Has_Pending_IRQ() const noexcept
    {
        return m_pending_irq;
    }

    void CGPIO_Manager::CPin::Set_Pending_IRQ(bool set)
    {
        m_pending_irq = set;
    }

    void CGPIO_Manager::CPin::Enable_Interrupt_Type(IGPIO_Manager::IPin::NInterrupt_Type type)
    {
        m_enabled_interrupts[static_cast<std::size_t>(type)] = true;
    }

    void CGPIO_Manager::CPin::Disable_Interrupt_Type(IGPIO_Manager::IPin::NInterrupt_Type type)
    {
        m_enabled_interrupts[static_cast<std::size_t>(type)] = false;
    }

    bool CGPIO_Manager::CPin::Is_Interrupt_Enabled(IGPIO_Manager::IPin::NInterrupt_Type type) const
    {
        return m_enabled_interrupts[static_cast<std::size_t>(type)];
    }

    bool CGPIO_Manager::CPin::Is_Interrupt_Detected(NState new_state) const noexcept
    {
        // Rising edge
        if (Is_Interrupt_Enabled(IGPIO_Manager::IPin::NInterrupt_Type::Rising_Edge) && Get_State() == CPin::NState::Low &&
            new_state == CPin::NState::High)
        {
            return true;
        }

        // Falling edge
        if (Is_Interrupt_Enabled(IGPIO_Manager::IPin::NInterrupt_Type::Falling_Edge) && Get_State() == CPin::NState::High &&
            new_state == CPin::NState::Low)
        {
            return true;
        }

        // Low state
        if (Is_Interrupt_Enabled(IGPIO_Manager::IPin::NInterrupt_Type::Low) && new_state == NState::Low)
        {
            return true;
        }

        // High state
        if (Is_Interrupt_Enabled(IGPIO_Manager::IPin::NInterrupt_Type::High) && new_state == NState::High)
        {
            return true;
        }

        return false;
    }

    CGPIO_Manager::CGPIO_Manager(std::shared_ptr<CInterrupt_Controller> interrupt_controller) noexcept
        : m_interrupt_controller{ interrupt_controller }
        , m_logging_system{ *utils::CSingleton<utils::CLogging_System>::Get_Instance() }
    {
        Reset();
    }

    void CGPIO_Manager::Reset() noexcept
    {
        std::fill(m_regs.begin(), m_regs.end(), 0);
        std::fill(m_pins.begin(), m_pins.end(), CPin());
    }

    std::uint32_t CGPIO_Manager::Get_Size() const noexcept
    {
        return static_cast<std::uint32_t>(sizeof(m_regs));
    }

    void CGPIO_Manager::Update_Pin_Function(std::size_t reg_idx, bool last_reg)
    {
        // Number of pings in a GPFSEL register (3 bits per function).
        static constexpr std::size_t Number_Of_Pins_In_SEL_Reg = 10;

        // Last bit index considering there is 3 bits per function
        static constexpr std::size_t Last_Bit_Idx =
        (Number_Of_Pins_In_Reg / Number_Of_Pins_In_SEL_Reg) * Number_Of_Pins_In_SEL_Reg;

        // Calculate the last bit index (the last register only uses 12 bits).
        const std::uint32_t last_bit_idx = last_reg ? 12U : Last_Bit_Idx;

        // Iterate through the bits of the GPFSEL register (3 bits per pin function)
        for (std::uint32_t idx = 0; idx < last_bit_idx; idx += 3U)
        {
            // Retrieve the pin function.
            const auto function = static_cast<CPin::NFunction>((m_regs[reg_idx] >> idx) & 0b111U);

            // Calculate the pin index the function corresponds to.
            const auto pin_idx = (reg_idx * Number_Of_Pins_In_SEL_Reg) + idx / 3;

            // Check if the function is already assigned to the corresponds pin.
            if (m_pins[pin_idx].Get_Function() != function)
            {
                // clang-format off
                m_logging_system.Debug(fmt::format("Function of pin {} is set to {}",
                                                   pin_idx, magic_enum::enum_name(function)).c_str());
                // clang-format on

                // Set the pin function.
                m_pins[pin_idx].Set_Function(function);
            }
        }
    }

    void CGPIO_Manager::Update_Pin_State(std::size_t reg_idx, CPin::NState state, bool last_reg)
    {
        // Calculate the index of the last bit as there is only 54 GPIO pins.
        const std::uint32_t last_bit_idx =
        last_reg ? (Number_of_GPIO_Pins - Number_Of_Pins_In_Reg) : Number_Of_Pins_In_Reg;

        // Iterate through the pins of the register.
        for (std::uint32_t idx = 0; idx < last_bit_idx; ++idx)
        {
            // Calculate the pin index.
            const auto pin_idx = last_reg ? (Number_Of_Pins_In_Reg + idx) : idx;

            // Check if the bit is set.
            if (utils::math::Is_Bit_Set(m_regs[reg_idx], idx))
            {
                // Make sure the pin function has been as to output.
                if (m_pins[pin_idx].Get_Function() == CPin::NFunction::Output)
                {
                    // Check if the state of the pin would change at all.
                    if (m_pins[pin_idx].Get_State() != state)
                    {
                        // clang-format off
                        m_logging_system.Debug(fmt::format("State of pin {} is set to {}",
                                                           pin_idx, magic_enum::enum_name(state)).c_str());
                        // clang-format on

                        // Update the state of the pin.
                        m_pins[pin_idx].Set_State(state);
                        Mirror_Pin_State_In_GPLEVn(pin_idx, state);
                    }

                    // Notify all external peripherals subscribing to the pin
                    Notify_External_Peripherals(pin_idx);
                }
                else
                {
                    // Cannot change the state of a non-output pin.
                    // TODO there might be exception? (alternative functions)
                    // clang-format off
                    m_logging_system.Warning(fmt::format("Cannot change the state of pin {} as its "
                                                         "function has not been to output", pin_idx).c_str());
                    // clang-format on
                }
            }
        }

        // RS latch - only the last write defines the state of the pin (SW emulation).
        m_regs[reg_idx] = 0;
    }

    void CGPIO_Manager::Notify_External_Peripherals(std::uint32_t pin_idx)
    {
        std::for_each(m_external_peripherals.begin(), m_external_peripherals.end(), [pin_idx, this](const auto& peripheral) {
            const auto subscription = peripheral->Get_GPIO_Subscription();

            if (subscription.contains(pin_idx))
            {
                peripheral->GPIO_Subscription_Callback(pin_idx);
            }
        });
    }

    std::size_t CGPIO_Manager::Get_Register_Index(std::size_t& pin_idx, IGPIO_Manager::IPin::NRegister reg_0, IGPIO_Manager::IPin::NRegister reg_1) noexcept
    {
        if (pin_idx >= Number_Of_Pins_In_Reg)
        {
            // Recalculate the pin index, so it is relative to the register.
            pin_idx -= Number_Of_Pins_In_Reg;

            return static_cast<std::size_t>(reg_1);
        }

        return static_cast<std::size_t>(reg_0);
    }

    void CGPIO_Manager::Mirror_Pin_State_In_GPLEVn(std::size_t pin_idx, CPin::NState state)
    {
        const auto reg_index = Get_Register_Index(pin_idx, IGPIO_Manager::IPin::NRegister::GPLEV0, IGPIO_Manager::IPin::NRegister::GPLEV1);
        auto& GPLEVn_reg = m_regs[reg_index];

        if (state == CPin::NState::High)
        {
            // High
            GPLEVn_reg |= (0b1U << pin_idx);
        }
        else
        {
            // Low
            GPLEVn_reg &= ~(0b1U << pin_idx);
        }
    }

    void CGPIO_Manager::Set_Interrupt(std::size_t reg_idx, bool last_reg, IGPIO_Manager::IPin::NInterrupt_Type type)
    {
        // Calculate the index of the last bit as there is only 54 GPIO pins.
        const std::uint32_t last_bit_idx =
        last_reg ? (Number_of_GPIO_Pins - Number_Of_Pins_In_Reg) : Number_Of_Pins_In_Reg;

        // Iterate through the pins of the register.
        for (std::uint32_t idx = 0; idx < last_bit_idx; ++idx)
        {
            // Calculate the pin index.
            const auto pin_idx = last_reg ? (Number_Of_Pins_In_Reg + idx) : idx;

            if (utils::math::Is_Bit_Set(m_regs[reg_idx], idx))
            {
                // Enable interrupt.
                if (!m_pins[pin_idx].Is_Interrupt_Enabled(type))
                {
                    // clang-format off
                    m_logging_system.Debug(fmt::format("Interrupt {} has been enabled on pin {}",
                                                       magic_enum::enum_name(type), pin_idx).c_str());
                    // clang-format on

                    m_pins[pin_idx].Enable_Interrupt_Type(type);
                }
            }
            else
            {
                // Disable interrupt.
                if (m_pins[pin_idx].Is_Interrupt_Enabled(type))
                {
                    // clang-format off
                    m_logging_system.Debug(fmt::format("Interrupt {} has been disabled on pin {}",
                                                       magic_enum::enum_name(type), pin_idx).c_str());
                    // clang-format on

                    m_pins[pin_idx].Disable_Interrupt_Type(type);
                }
            }
        }
    }

    void CGPIO_Manager::Write(std::uint32_t addr, const char* data, std::uint32_t size)
    {
        const std::size_t reg_idx = addr / Reg_Size;
        const auto reg_type = static_cast<IGPIO_Manager::IPin::NRegister>(reg_idx);

        // Make sure we are not writing to a read-only register.
        if (IGPIO_Manager::IPin::s_read_only_registers.contains(reg_type))
        {
            // clang-format off
            m_logging_system.Warning(fmt::format("The GPIO {} register is read-only",
                                                 magic_enum::enum_name(reg_type)).c_str());
            // clang-format on

            return;
        }

        // Store a copy of the GPIO registers before they get overwritten.
        // The copy is used for example when accessing GPEDS0 and GPEDS1 (SW emulation).
        m_regs_prev = m_regs;

        // Write data to the peripheral's registers.
        std::copy_n(data, size, &std::bit_cast<char*>(m_regs.data())[addr]);

        switch (reg_type)
        {
            // Function select register
            case IGPIO_Manager::IPin::NRegister::GPFSEL0:
                [[fallthrough]];
            case IGPIO_Manager::IPin::NRegister::GPFSEL1:
            case IGPIO_Manager::IPin::NRegister::GPFSEL2:
            case IGPIO_Manager::IPin::NRegister::GPFSEL3:
            case IGPIO_Manager::IPin::NRegister::GPFSEL4:
            case IGPIO_Manager::IPin::NRegister::GPFSEL5:
                Update_Pin_Function(reg_idx, reg_type == IGPIO_Manager::IPin::NRegister::GPFSEL5);
                break;

            // Set register
            case IGPIO_Manager::IPin::NRegister::GPSET0:
                [[fallthrough]];
            case IGPIO_Manager::IPin::NRegister::GPSET1:
                Update_Pin_State(reg_idx, IPin::NState::High, reg_type == IGPIO_Manager::IPin::NRegister::GPSET1);
                break;

            // Clear register
            case IGPIO_Manager::IPin::NRegister::GPCLR0:
                [[fallthrough]];
            case IGPIO_Manager::IPin::NRegister::GPCLR1:
                Update_Pin_State(reg_idx, IPin::NState::Low, reg_type == IGPIO_Manager::IPin::NRegister::GPCLR1);
                break;

            // These registers reflect the actual state of each pin,
            // The corresponding bits are set/cleared whenever a pin changes its state,
            case IGPIO_Manager::IPin::NRegister::GPLEV0:
                [[fallthrough]];
            case IGPIO_Manager::IPin::NRegister::GPLEV1:
                break;

            // Clear pending IRQ
            // TODO write a 1 to it whenever an interrupt occurs, so we can distinguish what pin has generated it?
            case IGPIO_Manager::IPin::NRegister::GPEDS0:
                [[fallthrough]];
            case IGPIO_Manager::IPin::NRegister::GPEDS1:
                Clear_IRQ(reg_idx, reg_type == IGPIO_Manager::IPin::NRegister::GPEDS1);
                break;

            // Enable/disable rising edge (interrupt)
            case IGPIO_Manager::IPin::NRegister::GPREN0:
                [[fallthrough]];
            case IGPIO_Manager::IPin::NRegister::GPREN1:
                Set_Interrupt(reg_idx, reg_type == IGPIO_Manager::IPin::NRegister::GPREN1, IGPIO_Manager::IPin::NInterrupt_Type::Rising_Edge);
                break;

            // Enable/disable falling edge (interrupt)
            case IGPIO_Manager::IPin::NRegister::GPHEN0:
                [[fallthrough]];
            case IGPIO_Manager::IPin::NRegister::GPHEN1:
                Set_Interrupt(reg_idx, reg_type == IGPIO_Manager::IPin::NRegister::GPHEN1, IGPIO_Manager::IPin::NInterrupt_Type::High);
                break;

            // Enable/disable low state (interrupt)
            case IGPIO_Manager::IPin::NRegister::GPLEN0:
                [[fallthrough]];
            case IGPIO_Manager::IPin::NRegister::GPLEN1:
                Set_Interrupt(reg_idx, reg_type == IGPIO_Manager::IPin::NRegister::GPLEN1, IGPIO_Manager::IPin::NInterrupt_Type::Low);
                break;

            // Enable/disable high state (interrupt)
            case IGPIO_Manager::IPin::NRegister::GPFEN0:
                [[fallthrough]];
            case IGPIO_Manager::IPin::NRegister::GPFEN1:
                Set_Interrupt(reg_idx, reg_type == IGPIO_Manager::IPin::NRegister::GPFEN1, IGPIO_Manager::IPin::NInterrupt_Type::Falling_Edge);
                break;

            // Reserved unused registers
            case IGPIO_Manager::IPin::NRegister::Reserved_01:
                [[fallthrough]];
            case IGPIO_Manager::IPin::NRegister::Reserved_02:
            case IGPIO_Manager::IPin::NRegister::Reserved_03:
            case IGPIO_Manager::IPin::NRegister::Reserved_04:
            case IGPIO_Manager::IPin::NRegister::Reserved_05:
            case IGPIO_Manager::IPin::NRegister::Reserved_06:
            case IGPIO_Manager::IPin::NRegister::Reserved_07:
            case IGPIO_Manager::IPin::NRegister::Reserved_08:
            case IGPIO_Manager::IPin::NRegister::Reserved_09:
            case IGPIO_Manager::IPin::NRegister::Reserved_10:
            case IGPIO_Manager::IPin::NRegister::Reserved_11:
            case IGPIO_Manager::IPin::NRegister::Reserved_12:
                break;

            default:
                // TODO add the rest of the registers (asynchronous interrupts are not supported)
                break;
        }
    }

    void CGPIO_Manager::Add_External_Peripheral(IExternal_Peripheral* peripheral)
    {
        m_external_peripherals.emplace_back(peripheral);
    }

    void CGPIO_Manager::Read(std::uint32_t addr, char* data, std::uint32_t size)
    {
        const std::size_t reg_idx = addr / Reg_Size;
        const auto reg_type = static_cast<IGPIO_Manager::IPin::NRegister>(reg_idx);

        // Make sure we are not reading from a write-only register.
        if (IGPIO_Manager::IPin::s_write_only_registers.contains(reg_type))
        {
            // clang-format off
            m_logging_system.Warning(fmt::format("The GPIO {} register is write-only",
                                                 magic_enum::enum_name(reg_type)).c_str());
            // clang-format on

            return;
        }

        // Read data from the peripheral's registers.
        std::copy_n(&std::bit_cast<char*>(m_regs.data())[addr], size, data);
    }

    const CGPIO_Manager::CPin& CGPIO_Manager::Get_Pin(std::size_t idx) const
    {
        return m_pins.at(idx);
    }

    CGPIO_Manager::IPin::NState CGPIO_Manager::Read_GPIO_Pin(std::size_t idx) const
    {
        return m_pins.at(idx).Get_State();
    }

    IGPIO_Manager::IPin::NPin_Set_Status CGPIO_Manager::Set_Pin_State(std::size_t pin_idx, IPin::NState state)
    {
        // Make sure pin_idx is valid.
        if (pin_idx >= Number_of_GPIO_Pins)
        {
            return IGPIO_Manager::IPin::NPin_Set_Status::Invalid_Pin_Number;
        }

        // Get the pin by its index.
        auto& pin = m_pins[pin_idx];

        const IPin::NFunction pin_function = pin.Get_Function();

        // Make sure the pin function has been set to input.
        // clang-format off
        if ((pin_function != IPin::NFunction::Input) &&
            (pin_function != IPin::NFunction::Alt_5) &&
            (pin_function != IPin::NFunction::Alt_0))
        {
            return IGPIO_Manager::IPin::NPin_Set_Status::Invalid_Pin_Function;
        }
        // clang-format on

        // Check if changing the pin's state triggers an interrupt.
        // This must be checked before the state is changed.
        const bool interrupt_detected = pin.Is_Interrupt_Detected(state);

        // Change the state of the pin.
        pin.Set_State(state);
        Mirror_Pin_State_In_GPLEVn(pin_idx, state);

        // Notify external peripherals.
        Notify_External_Peripherals(static_cast<std::uint32_t>(pin_idx));

        if (interrupt_detected)
        {
            // Set a pending interrupt on the pin.
            pin.Set_Pending_IRQ(true);

            // Reflect the pending IRQ in GPEDS.
            Update_GPEDS(pin_idx);

            // Convert the pin index into an IRQ source.
            const auto irq_source = CInterrupt_Controller::Get_IRQ_Source(pin_idx);

            // Notify the interrupt controller.
            m_interrupt_controller->Signalize_IRQ(irq_source);
        }

        return IGPIO_Manager::IPin::NPin_Set_Status::OK;
    }

    void CGPIO_Manager::Enable_HW_Reset_Listening(uint32_t pin_idx)
    {
        m_reset_pin = pin_idx;
        m_logging_system.Debug(fmt::format("Interrupt {} has been enabled on pin {}",
            magic_enum::enum_name(IGPIO_Manager::IPin::NInterrupt_Type::Rising_Edge), pin_idx).c_str());
        m_pins[pin_idx].Enable_Interrupt_Type(IGPIO_Manager::IPin::NInterrupt_Type::Rising_Edge);
    }

    void CGPIO_Manager::Disable_HW_Reset_Listening(uint32_t pin_idx)
    {
        m_reset_pin = 0;
        m_logging_system.Debug(fmt::format("Interrupt {} has been disabled on pin {}",
            magic_enum::enum_name(IGPIO_Manager::IPin::NInterrupt_Type::Rising_Edge), pin_idx).c_str());
        m_pins[pin_idx].Enable_Interrupt_Type(IGPIO_Manager::IPin::NInterrupt_Type::Rising_Edge);
    }

    uint32_t CGPIO_Manager::Get_Reset_Pin()
    {
        return m_reset_pin;
    }

    void CGPIO_Manager::Update_GPEDS(std::size_t pin_idx)
    {
        // Assume the pin index comes from the first set of indexes.
        auto reg_idx = static_cast<std::uint32_t>(IGPIO_Manager::IPin::NRegister::GPEDS0);

        // Check if GPEDS1 should be used (pin_idx >= 32)
        if (pin_idx >= Number_Of_Pins_In_Reg)
        {
            reg_idx = static_cast<std::uint32_t>(IGPIO_Manager::IPin::NRegister::GPEDS1);
            pin_idx -= Number_Of_Pins_In_Reg;
        }

        // There is a pending IRQ on this pin.
        m_regs[reg_idx] |= (1U << pin_idx);
    }

    void CGPIO_Manager::Clear_IRQ(std::size_t reg_idx, bool last_reg)
    {
        // Calculate the index of the last bit as there is only 54 GPIO pins.
        const std::uint32_t last_bit_idx =
        last_reg ? (Number_of_GPIO_Pins - Number_Of_Pins_In_Reg) : Number_Of_Pins_In_Reg;

        // Iterate through the pins of the register.
        for (std::uint32_t idx = 0; idx < last_bit_idx; ++idx)
        {
            // Calculate the pin index.
            const auto pin_idx = last_reg ? (Number_Of_Pins_In_Reg + idx) : idx;
            auto& pin = m_pins[pin_idx];

            // Check if the pin is set to a 1 (pending interrupt should be cleared)
            if (utils::math::Is_Bit_Set(m_regs[reg_idx], idx) && pin.Has_Pending_IRQ())
            {
                // Clear the bit (in the prev value, so we do not affect the current state).
                m_regs_prev[reg_idx] &= ~(1U << idx);

                // Clear the interrupt
                pin.Set_Pending_IRQ(false);

                // Convert the pin index into an IRQ source.
                const auto irq_source = CInterrupt_Controller::Get_IRQ_Source(pin_idx);

                // Notify the interrupt controller.
                m_interrupt_controller->Clear_Pending_IRQ(irq_source);

                // clang-format off
                m_logging_system.Debug(fmt::format("Pending interrupt on GPIO pin {} has been cleared",
                                                   pin_idx).c_str());
                // clang-format on
            }
        }

        // RS latch - only the last write defines the state of the pin (SW emulation).
        m_regs[reg_idx] = m_regs_prev[reg_idx];
    }

} // namespace zero_mate::peripheral