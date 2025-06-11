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

// Project file imports

#include "mock_gpio.hpp"

namespace zero_mate::peripheral
{
    CGPIO_Manager_Mock::Mock_Pin::Mock_Pin()
        : m_state{ NState::Low }
        , m_function{ NFunction::Input }
        , m_enabled_interrupts{}
        , m_pending_irq{ false }
    {
    }

    IGPIO_Manager::IPin::NFunction CGPIO_Manager_Mock::Mock_Pin::Get_Function() const noexcept
    {
        return m_function;
    }

    void CGPIO_Manager_Mock::Mock_Pin::Set_Function(NFunction function) noexcept
    {
        m_function = function;
    }

    IGPIO_Manager::IPin::NState CGPIO_Manager_Mock::Mock_Pin::Get_State() const noexcept
    {
        return m_state;
    }

    void CGPIO_Manager_Mock::Mock_Pin::Set_State(NState state) noexcept
    {
        m_state = state;
    }

    bool CGPIO_Manager_Mock::Mock_Pin::Has_Pending_IRQ() const noexcept
    {
        return m_pending_irq;
    }

    void CGPIO_Manager_Mock::Mock_Pin::Set_Pending_IRQ(bool set)
    {
        m_pending_irq = set;
    }

    void CGPIO_Manager_Mock::Mock_Pin::Enable_Interrupt_Type(NInterrupt_Type type)
    {
        m_enabled_interrupts[static_cast<std::size_t>(type)] = true;
    }

    void CGPIO_Manager_Mock::Mock_Pin::Disable_Interrupt_Type(NInterrupt_Type type)
    {
        m_enabled_interrupts[static_cast<std::size_t>(type)] = false;
    }

    bool CGPIO_Manager_Mock::Mock_Pin::Is_Interrupt_Enabled(NInterrupt_Type type) const
    {
        return m_enabled_interrupts[static_cast<std::size_t>(type)];
    }

    bool CGPIO_Manager_Mock::Mock_Pin::Is_Interrupt_Detected(NState new_state) const noexcept
    {
        // Rising edge
        if (Is_Interrupt_Enabled(Mock_Pin::NInterrupt_Type::Rising_Edge) && Get_State() == Mock_Pin::NState::Low &&
            new_state == Mock_Pin::NState::High)
        {
            return true;
        }

        // Falling edge
        if (Is_Interrupt_Enabled(Mock_Pin::NInterrupt_Type::Falling_Edge) && Get_State() == Mock_Pin::NState::High &&
            new_state == Mock_Pin::NState::Low)
        {
            return true;
        }

        // Low state
        if (Is_Interrupt_Enabled(Mock_Pin::NInterrupt_Type::Low) && new_state == NState::Low)
        {
            return true;
        }

        // High state
        if (Is_Interrupt_Enabled(Mock_Pin::NInterrupt_Type::High) && new_state == NState::High)
        {
            return true;
        }

        return false;
    }

    CGPIO_Manager_Mock::CGPIO_Manager_Mock() noexcept
    {
        Reset();
    }

    CGPIO_Manager_Mock::Mock_Pin::~Mock_Pin()
    {

    }

    void CGPIO_Manager_Mock::Reset() noexcept
    {
        std::fill(m_regs.begin(), m_regs.end(), 0);
        std::fill(m_pins.begin(), m_pins.end(), Mock_Pin());
    }

    std::uint32_t CGPIO_Manager_Mock::Get_Size() const noexcept
    {
        return static_cast<std::uint32_t>(sizeof(m_regs));
    }

    void CGPIO_Manager_Mock::Update_Pin_Function(std::size_t reg_idx, bool last_reg)
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
            const auto function = static_cast<Mock_Pin::NFunction>((m_regs[reg_idx] >> idx) & 0b111U);

            // Calculate the pin index the function corresponds to.
            const auto pin_idx = (reg_idx * Number_Of_Pins_In_SEL_Reg) + idx / 3;

            // Check if the function is already assigned to the corresponds pin.
            if (m_pins[pin_idx].Get_Function() != function)
            {
                // Set the pin function.
                m_pins[pin_idx].Set_Function(function);
            }
        }
    }

    void CGPIO_Manager_Mock::Update_Pin_State(std::size_t reg_idx, Mock_Pin::NState state, bool last_reg)
    {
        // Calculate the index of the last bit as there is only 54 GPIO pins.
        const std::uint32_t last_bit_idx =
            last_reg ? (IGPIO_Manager::Number_of_GPIO_Pins - Number_Of_Pins_In_Reg) : Number_Of_Pins_In_Reg;

        // Iterate through the pins of the register.
        for (std::uint32_t idx = 0; idx < last_bit_idx; ++idx)
        {
            // Calculate the pin index.
            const auto pin_idx = last_reg ? (Number_Of_Pins_In_Reg + idx) : idx;

            // Check if the bit is set.
            if (utils::math::Is_Bit_Set(m_regs[reg_idx], idx))
            {
                // Make sure the pin function has been as to output.
                if (m_pins[pin_idx].Get_Function() == Mock_Pin::NFunction::Output)
                {
                    // Check if the state of the pin would change at all.
                    if (m_pins[pin_idx].Get_State() != state)
                    {
                        // Update the state of the pin.
                        m_pins[pin_idx].Set_State(state);
                        Mirror_Pin_State_In_GPLEVn(pin_idx, state);
                    }

                    // Notify all external peripherals subscribing to the pin
                    Notify_External_Peripherals(pin_idx);
                }
            }
        }

        // RS latch - only the last write defines the state of the pin (SW emulation).
        m_regs[reg_idx] = 0;
    }

    void CGPIO_Manager_Mock::Notify_External_Peripherals(std::uint32_t pin_idx)
    {
        std::for_each(m_external_peripherals.begin(), m_external_peripherals.end(), [pin_idx](const auto& peripheral) {
            const auto subscription = peripheral->Get_GPIO_Subscription();

            if (subscription.contains(pin_idx))
            {
                peripheral->GPIO_Subscription_Callback(pin_idx);
            }
            });
    }

    std::size_t CGPIO_Manager_Mock::Get_Register_Index(std::size_t& pin_idx, Mock_Pin::NRegister reg_0, Mock_Pin::NRegister reg_1) noexcept
    {
        if (pin_idx >= Number_Of_Pins_In_Reg)
        {
            // Recalculate the pin index, so it is relative to the register.
            pin_idx -= Number_Of_Pins_In_Reg;

            return static_cast<std::size_t>(reg_1);
        }

        return static_cast<std::size_t>(reg_0);
    }

    void CGPIO_Manager_Mock::Mirror_Pin_State_In_GPLEVn(std::size_t pin_idx, Mock_Pin::NState state)
    {
        const auto reg_index = Get_Register_Index(pin_idx, Mock_Pin::NRegister::GPLEV0, Mock_Pin::NRegister::GPLEV1);
        auto& GPLEVn_reg = m_regs[reg_index];

        if (state == Mock_Pin::NState::High)
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

    void CGPIO_Manager_Mock::Set_Interrupt(std::size_t reg_idx, bool last_reg, Mock_Pin::NInterrupt_Type type)
    {
        // Calculate the index of the last bit as there is only 54 GPIO pins.
        const std::uint32_t last_bit_idx =
            last_reg ? (IGPIO_Manager::Number_of_GPIO_Pins - Number_Of_Pins_In_Reg) : Number_Of_Pins_In_Reg;

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
                    m_pins[pin_idx].Enable_Interrupt_Type(type);
                }
            }
            else
            {
                // Disable interrupt.
                if (m_pins[pin_idx].Is_Interrupt_Enabled(type))
                {
                    m_pins[pin_idx].Disable_Interrupt_Type(type);
                }
            }
        }
    }

    void CGPIO_Manager_Mock::Write(std::uint32_t addr, const char* data, std::uint32_t size)
    {
        const std::size_t reg_idx = addr / Reg_Size;
        const auto reg_type = static_cast<Mock_Pin::NRegister>(reg_idx);

        // Make sure we are not writing to a read-only register.
        if (Mock_Pin::s_read_only_registers.contains(reg_type))
        {
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
        case Mock_Pin::NRegister::GPFSEL0:
            [[fallthrough]];
        case Mock_Pin::NRegister::GPFSEL1:
        case Mock_Pin::NRegister::GPFSEL2:
        case Mock_Pin::NRegister::GPFSEL3:
        case Mock_Pin::NRegister::GPFSEL4:
        case Mock_Pin::NRegister::GPFSEL5:
            Update_Pin_Function(reg_idx, reg_type == Mock_Pin::NRegister::GPFSEL5);
            break;

            // Set register
        case Mock_Pin::NRegister::GPSET0:
            [[fallthrough]];
        case Mock_Pin::NRegister::GPSET1:
            Update_Pin_State(reg_idx, Mock_Pin::NState::High, reg_type == Mock_Pin::NRegister::GPSET1);
            break;

            // Clear register
        case Mock_Pin::NRegister::GPCLR0:
            [[fallthrough]];
        case Mock_Pin::NRegister::GPCLR1:
            Update_Pin_State(reg_idx, Mock_Pin::NState::Low, reg_type == Mock_Pin::NRegister::GPCLR1);
            break;

            // These registers reflect the actual state of each pin,
            // The corresponding bits are set/cleared whenever a pin changes its state,
        case Mock_Pin::NRegister::GPLEV0:
            [[fallthrough]];
        case Mock_Pin::NRegister::GPLEV1:
            break;

            // Clear pending IRQ
            // TODO write a 1 to it whenever an interrupt occurs, so we can distinguish what pin has generated it?
        case Mock_Pin::NRegister::GPEDS0:
            [[fallthrough]];
        case Mock_Pin::NRegister::GPEDS1:
            Clear_IRQ(reg_idx, reg_type == Mock_Pin::NRegister::GPEDS1);
            break;

            // Enable/disable rising edge (interrupt)
        case Mock_Pin::NRegister::GPREN0:
            [[fallthrough]];
        case Mock_Pin::NRegister::GPREN1:
            Set_Interrupt(reg_idx, reg_type == Mock_Pin::NRegister::GPREN1, Mock_Pin::NInterrupt_Type::Rising_Edge);
            break;

            // Enable/disable falling edge (interrupt)
        case Mock_Pin::NRegister::GPHEN0:
            [[fallthrough]];
        case Mock_Pin::NRegister::GPHEN1:
            Set_Interrupt(reg_idx, reg_type == Mock_Pin::NRegister::GPHEN1, Mock_Pin::NInterrupt_Type::High);
            break;

            // Enable/disable low state (interrupt)
        case Mock_Pin::NRegister::GPLEN0:
            [[fallthrough]];
        case Mock_Pin::NRegister::GPLEN1:
            Set_Interrupt(reg_idx, reg_type == Mock_Pin::NRegister::GPLEN1, Mock_Pin::NInterrupt_Type::Low);
            break;

            // Enable/disable high state (interrupt)
        case Mock_Pin::NRegister::GPFEN0:
            [[fallthrough]];
        case Mock_Pin::NRegister::GPFEN1:
            Set_Interrupt(reg_idx, reg_type == Mock_Pin::NRegister::GPFEN1, Mock_Pin::NInterrupt_Type::Falling_Edge);
            break;

            // Reserved unused registers
        case Mock_Pin::NRegister::Reserved_01:
            [[fallthrough]];
        case Mock_Pin::NRegister::Reserved_02:
        case Mock_Pin::NRegister::Reserved_03:
        case Mock_Pin::NRegister::Reserved_04:
        case Mock_Pin::NRegister::Reserved_05:
        case Mock_Pin::NRegister::Reserved_06:
        case Mock_Pin::NRegister::Reserved_07:
        case Mock_Pin::NRegister::Reserved_08:
        case Mock_Pin::NRegister::Reserved_09:
        case Mock_Pin::NRegister::Reserved_10:
        case Mock_Pin::NRegister::Reserved_11:
        case Mock_Pin::NRegister::Reserved_12:
            break;

        default:
            // TODO add the rest of the registers (asynchronous interrupts are not supported)
            break;
        }
    }

    void CGPIO_Manager_Mock::Add_External_Peripheral(IExternal_Peripheral* peripheral)
    {
        m_external_peripherals.emplace_back(peripheral);
    }

    void CGPIO_Manager_Mock::Read(std::uint32_t addr, char* data, std::uint32_t size)
    {
        const std::size_t reg_idx = addr / Reg_Size;
        const auto reg_type = static_cast<Mock_Pin::NRegister>(reg_idx);

        // Make sure we are not reading from a write-only register.
        if (CGPIO_Manager_Mock::Mock_Pin::s_write_only_registers.contains(reg_type))
        {
            return;
        }

        // Read data from the peripheral's registers.
        std::copy_n(&std::bit_cast<char*>(m_regs.data())[addr], size, data);
    }

    const CGPIO_Manager_Mock::Mock_Pin& CGPIO_Manager_Mock::Get_Pin(std::size_t idx) const
    {
        return m_pins.at(idx);
    }

    IGPIO_Manager::IPin::NState CGPIO_Manager_Mock::Read_GPIO_Pin(std::size_t idx) const
    {
        return m_pins.at(idx).Get_State();
    }

    IGPIO_Manager::IPin::NPin_Set_Status CGPIO_Manager_Mock::Set_Pin_State(std::size_t pin_idx, Mock_Pin::NState state)
    {
        // Make sure pin_idx is valid.
        if (pin_idx >= IGPIO_Manager::Number_of_GPIO_Pins)
        {
            return IGPIO_Manager::IPin::NPin_Set_Status::Invalid_Pin_Number;
        }

        // Get the pin by its index.
        auto& pin = m_pins[pin_idx];

        const IGPIO_Manager::IPin::NFunction pin_function = pin.Get_Function();

        // Make sure the pin function has been set to input.
        // clang-format off
        if ((pin_function != IGPIO_Manager::IPin::NFunction::Input) &&
            (pin_function != IGPIO_Manager::IPin::NFunction::Alt_5) &&
            (pin_function != IGPIO_Manager::IPin::NFunction::Alt_0))
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
        }

        return IGPIO_Manager::IPin::NPin_Set_Status::OK;
    }

    void CGPIO_Manager_Mock::Enable_HW_Reset_Listening(uint32_t pin_idx)
    {
        m_reset_pin = pin_idx;
        m_pins[pin_idx].Enable_Interrupt_Type(Mock_Pin::NInterrupt_Type::Rising_Edge);
    }

    void CGPIO_Manager_Mock::Disable_HW_Reset_Listening(uint32_t pin_idx)
    {
        m_reset_pin = 0;
        m_pins[pin_idx].Enable_Interrupt_Type(Mock_Pin::NInterrupt_Type::Rising_Edge);
    }

    uint32_t CGPIO_Manager_Mock::Get_Reset_Pin()
    {
        return m_reset_pin;
    }

    void CGPIO_Manager_Mock::Update_GPEDS(std::size_t pin_idx)
    {
        // Assume the pin index comes from the first set of indexes.
        auto reg_idx = static_cast<std::uint32_t>(Mock_Pin::NRegister::GPEDS0);

        // Check if GPEDS1 should be used (pin_idx >= 32)
        if (pin_idx >= Number_Of_Pins_In_Reg)
        {
            reg_idx = static_cast<std::uint32_t>(Mock_Pin::NRegister::GPEDS1);
            pin_idx -= Number_Of_Pins_In_Reg;
        }

        // There is a pending IRQ on this pin.
        m_regs[reg_idx] |= (1U << pin_idx);
    }

    void CGPIO_Manager_Mock::Clear_IRQ(std::size_t reg_idx, bool last_reg)
    {
        // Calculate the index of the last bit as there is only 54 GPIO pins.
        const std::uint32_t last_bit_idx =
            last_reg ? (IGPIO_Manager::Number_of_GPIO_Pins - Number_Of_Pins_In_Reg) : Number_Of_Pins_In_Reg;

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
            }
        }

        // RS latch - only the last write defines the state of the pin (SW emulation).
        m_regs[reg_idx] = m_regs_prev[reg_idx];
    }

    bool CGPIO_Manager_Mock::Static_Read_GPIO_Pin(std::uint32_t pin) {
        if (!current_instance) return false;
        return current_instance->Read_GPIO_Pin(pin) == IGPIO_Manager::IPin::NState::High;
    }

    int CGPIO_Manager_Mock::Static_Set_GPIO_Pin(std::uint32_t pin, bool state) {
        if (!current_instance) return 0;
        current_instance->Set_Pin_State(pin, state ? IGPIO_Manager::IPin::NState::High : IGPIO_Manager::IPin::NState::Low);
        return 0;
    }

} // namespace zero_mate::peripheral