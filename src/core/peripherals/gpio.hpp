// ---------------------------------------------------------------------------------------------------------------------
/// \file gpio.hpp
/// \date 28. 05. 2023
/// \author Jakub Silhavy (jakub.silhavy.cz@gmail.com)
///
/// \brief This file defines the GPIO controller used in BCM2835.
///
/// To find more information about this peripheral, please visit
/// https://www.raspberrypi.org/app/uploads/2012/02/BCM2835-ARM-Peripherals.pdf (chapter 6)
// ---------------------------------------------------------------------------------------------------------------------

#pragma once

// STL imports (excluded from Doxygen)
/// \cond
#include <array>
#include <memory>
#include <limits>
#include <functional>
#include <unordered_set>
/// \endcond

// Project file imports

#include "peripheral.hpp"
#include "igpio.hpp"
#include "interrupt_controller.hpp"
#include "zero_mate/utils/logging_system.hpp"

#include <zero_mate/external_peripheral.hpp>

namespace zero_mate::peripheral
{
    // -----------------------------------------------------------------------------------------------------------------
    /// \class CGPIO_Manager
    /// \brief This class represents a GPIO (general-purpose I/O) manager.
    // -----------------------------------------------------------------------------------------------------------------
    class CGPIO_Manager final : public IGPIO_Manager
    {
    public:

        // -------------------------------------------------------------------------------------------------------------
        /// \class CPin
        /// \brief This class represents a single pin.
        // -------------------------------------------------------------------------------------------------------------
        class CPin final : public IPin
        {
        public:
            // ---------------------------------------------------------------------------------------------------------
            /// \brief Creates an instance of the class.
            ///
            /// The default state of the pin is set to NState::Low and its function is set to NFunction::Input.
            // ---------------------------------------------------------------------------------------------------------
            CPin();

            // ---------------------------------------------------------------------------------------------------------
            /// \brief Checks information about whether an interrupt has occurred or not.
            /// \param new_state State the pin is transitioning into
            /// \return true, if the transition triggers an interrupt. false, otherwise.
            // ---------------------------------------------------------------------------------------------------------
            [[nodiscard]] bool Is_Interrupt_Detected(IGPIO_Manager::IPin::NState new_state) const noexcept;

            // ---------------------------------------------------------------------------------------------------------
            /// \brief Returns the current state of the pin.
            /// \return State of the pin
            // ---------------------------------------------------------------------------------------------------------
            [[nodiscard]] IGPIO_Manager::IPin::NState Get_State() const noexcept;

            // ---------------------------------------------------------------------------------------------------------
            /// \brief Sets the current state of the pin.
            /// \param state New state of the pin
            // ---------------------------------------------------------------------------------------------------------
            void Set_State(IGPIO_Manager::IPin::NState state) noexcept;

            // ---------------------------------------------------------------------------------------------------------
            /// \brief Returns the current function of the pin.
            /// \return Function of the pin
            // ---------------------------------------------------------------------------------------------------------
            [[nodiscard]] IGPIO_Manager::IPin::NFunction Get_Function() const noexcept;

            // ---------------------------------------------------------------------------------------------------------
            /// \brief Sets the current function of the pin.
            /// \param function New function of the pin
            // ---------------------------------------------------------------------------------------------------------
            void Set_Function(IGPIO_Manager::IPin::NFunction function) noexcept;

            // ---------------------------------------------------------------------------------------------------------
            /// \brief Enables an interrupt on the pin.
            /// \param type Interrupt type to be enabled
            // ---------------------------------------------------------------------------------------------------------
            void Enable_Interrupt_Type(IGPIO_Manager::IPin::NInterrupt_Type type);

            // ---------------------------------------------------------------------------------------------------------
            /// \brief Disabled an interrupt on the pin.
            /// \param type Interrupt type to be disabled
            // ---------------------------------------------------------------------------------------------------------
            void Disable_Interrupt_Type(IGPIO_Manager::IPin::NInterrupt_Type type);

            // ---------------------------------------------------------------------------------------------------------
            /// \brief Checks whether or not an interrupt is enabled on the pin.
            /// \param type Interrupt type to be checked
            /// \return true, if the given interrupt type is enabled. false, otherwise.
            // ---------------------------------------------------------------------------------------------------------
            [[nodiscard]] bool Is_Interrupt_Enabled(IGPIO_Manager::IPin::NInterrupt_Type type) const;

            // ---------------------------------------------------------------------------------------------------------
            /// \brief Checks if there is a pending interrupt on the pin.
            /// \return true, if there is a pending interrupt. false, otherwise.
            // ---------------------------------------------------------------------------------------------------------
            [[nodiscard]] bool Has_Pending_IRQ() const noexcept;

            // ---------------------------------------------------------------------------------------------------------
            /// \brief Sets a pending interrupt.
            ///
            /// This function is called from CGPIO_Manager when clearing interrupts as well as when setting
            /// a new state of the pin which triggers an interrupt.
            ///
            /// \param set New value of the pending interrupt on the pin
            // ---------------------------------------------------------------------------------------------------------
            void Set_Pending_IRQ(bool set);

        private:
            IGPIO_Manager::IPin::NState m_state;                    ///< Current state of the pin
            IGPIO_Manager::IPin::NFunction m_function;              ///< Function of the pin
            IGPIO_Manager::IPin::Interrupts_t m_enabled_interrupts; ///< Array or interrupts (which are enabled/disabled)
            bool m_pending_irq;                ///< Flag if there is a pending interrupt on the pin
        };

    public:
        // -------------------------------------------------------------------------------------------------------------
        /// \brief Creates an instance of the class.
        /// \param interrupt_controller Reference to the interrupt controller, so it can notify it when an
        ///                             interrupt occurs
        // -------------------------------------------------------------------------------------------------------------
        explicit CGPIO_Manager(std::shared_ptr<CInterrupt_Controller> interrupt_controller) noexcept;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Resets/re-initializes the GPIO manager (IPeripheral interface).
        // -------------------------------------------------------------------------------------------------------------
        void Reset() noexcept override;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Returns the size of the peripheral (IPeripheral interface).
        /// \return number of register * register size
        // -------------------------------------------------------------------------------------------------------------
        [[nodiscard]] std::uint32_t Get_Size() const noexcept override;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Writes data to the peripheral (IPeripheral interface).
        /// \param addr Relative address (from the peripheral's perspective) where the data will be written
        /// \param data Pointer to the data to be written to the peripheral
        /// \param size Size of the data to be written to the peripheral [B]
        // -------------------------------------------------------------------------------------------------------------
        void Write(std::uint32_t addr, const char* data, std::uint32_t size) override;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Reads data from the peripheral (IPeripheral interface).
        /// \param addr Relative address (from the peripheral's perspective) from which the data will be read
        /// \param data Pointer to a buffer the data will be copied into
        /// \param size Size of the data to read from the peripheral [B]
        // -------------------------------------------------------------------------------------------------------------
        void Read(std::uint32_t addr, char* data, std::uint32_t size) override;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Hooks up an external peripheral to the GPIO manager.
        /// \param peripheral Instance of an external peripheral
        // -------------------------------------------------------------------------------------------------------------
        void Add_External_Peripheral(IExternal_Peripheral* peripheral) override;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Returns a const reference to a pin (visualization purposes).
        /// \param idx Index of the pin to be returned
        /// \return Const reference to a pin
        // -------------------------------------------------------------------------------------------------------------
        [[nodiscard]] const CPin& Get_Pin(std::size_t idx) const override;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Returns the current state of a given pin.
        /// \param idx Index of the pin whose state will be returned
        /// \return Current state of the given pin
        // -------------------------------------------------------------------------------------------------------------
        [[nodiscard]] IGPIO_Manager::IPin::NState Read_GPIO_Pin(std::size_t idx) const override;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Sets a new state of a given pin.
        ///
        /// This function is called from the outer world.
        ///
        /// \param pin_idx Index of the pin whose state will be changed
        /// \param state New state of the pin
        /// \return Information about whether the state has been changed successfully or not
        // -------------------------------------------------------------------------------------------------------------
        [[nodiscard]] IGPIO_Manager::IPin::NPin_Set_Status Set_Pin_State(std::size_t pin_idx, IGPIO_Manager::IPin::NState state) override;

        void Enable_HW_Reset_Listening(uint32_t pin_idx) override;

        void Disable_HW_Reset_Listening(uint32_t pin_idx) override;

        uint32_t Get_Reset_Pin() override;

    private:
        // -------------------------------------------------------------------------------------------------------------
        /// \brief Returns the index of the register (out of the two alternatives) based on the pin index.
        ///
        /// If the index of the pin is greater than 31, the second register is returned and the index gets
        /// modified, so it is relative to the register.
        ///
        /// \param pin_idx Index of the pin
        /// \param reg_0 First register (e.g. GPSET0)
        /// \param reg_1 Second register (e.g. GPSET1)
        /// \return Index of the register to be used
        // -------------------------------------------------------------------------------------------------------------
        [[nodiscard]] static std::size_t
        Get_Register_Index(std::size_t& pin_idx, IGPIO_Manager::IPin::NRegister reg_0, IGPIO_Manager::IPin::NRegister reg_1) noexcept;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Updates the function of all pins within a given register.
        /// \param reg_idx Index of an GPFSEL register
        /// \param last_reg Indication of whether it is the last register (not all bits are being used)
        // -------------------------------------------------------------------------------------------------------------
        void Update_Pin_Function(std::size_t reg_idx, bool last_reg) override;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Updates the state of all pins within a given register.
        /// \param reg_idx Index of an GPSET/GPCLR register
        /// \param state Indication of whether the state of the corresponding pins should be set to a 1 or 0
        /// \param last_reg Indication of whether it is the last register (not all bits are being used)
        // -------------------------------------------------------------------------------------------------------------
        void Update_Pin_State(std::size_t reg_idx, IGPIO_Manager::IPin::NState state, bool last_reg) override;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Mirrors the state of a given in in the GPLEV register.
        /// \param pin_idx Index of the pin
        /// \param state Current state of the pin
        // -------------------------------------------------------------------------------------------------------------
        void Mirror_Pin_State_In_GPLEVn(std::size_t pin_idx, IGPIO_Manager::IPin::NState state) override;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Enables/disables an interrupt for all pins within a given register
        ///
        /// If a bit is set to a 0, the interrupt type will be disabled. If a bit set to a 1, the interrupt type will
        /// be enabled.
        ///
        /// \param reg_idx Index of the register (GPREN, GPHEN, GPLEN, or GPFEN)
        /// \param last_reg Indication of whether it is the last register (not all bits are being used)
        /// \param type Type of the interrupt to be enabled/disabled
        // -------------------------------------------------------------------------------------------------------------
        void Set_Interrupt(std::size_t reg_idx, bool last_reg, IPin::NInterrupt_Type type) override;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Clears a pending interrupt by writing a 1 to the GPEDS register.
        /// \param reg_idx Index of the register to be used
        /// \param last_reg Indication of whether it is the last register (not all bits are being used)
        // -------------------------------------------------------------------------------------------------------------
        void Clear_IRQ(std::size_t reg_idx, bool last_reg) override;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Notifies external peripherals that subscribe to the given pin about the change of its state.
        /// \param pin_idx Index of the pin whose state has changed
        // -------------------------------------------------------------------------------------------------------------
        void Notify_External_Peripherals(std::uint32_t pin_idx) override;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Reflects the state of a pending IRQ in the GPEDS register.
        /// \param pin_idx Index of the pin where an interrupt has occurred
        // -------------------------------------------------------------------------------------------------------------
        void Update_GPEDS(std::size_t pin_idx) override;

    private:
        std::shared_ptr<CInterrupt_Controller> m_interrupt_controller; ///< Interrupt controller
        std::array<std::uint32_t, IGPIO_Manager::IPin::Number_Of_Registers> m_regs;         ///< GPIO registers
        std::array<CPin, Number_of_GPIO_Pins> m_pins;                  ///< GPIO pins
        utils::CLogging_System& m_logging_system;                      ///< Logging system
        std::vector<IExternal_Peripheral*> m_external_peripherals;     ///< Collection of external peripherals
        std::array<std::uint32_t, IGPIO_Manager::IPin::Number_Of_Registers> m_regs_prev;    ///< Previous values of the GPIO registers

        std::uint32_t m_reset_pin;
    };

} // namespace zero_mate::peripheral