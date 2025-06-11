// ---------------------------------------------------------------------------------------------------------------------
/// \file mini_uart.cpp
/// \date 24. 07. 2023
/// \author Jakub Silhavy (jakub.silhavy.cz@gmail.com)
///
/// \brief This file implements the Mini UART auxiliary peripheral used in BCM2835.
///
/// To find more information about this peripheral, please visit
/// https://www.raspberrypi.org/app/uploads/2012/02/BCM2835-ARM-Peripherals.pdf (chapter 2)
// ---------------------------------------------------------------------------------------------------------------------

// STL imports (excluded from Doxygen)
/// \cond
#include <chrono>
/// \endcond

// Project file imports

#include "mini_uart.hpp"
#include "auxiliary.hpp"

namespace zero_mate::peripheral
{
    CMini_UART::CMini_UART(CAUX& aux)
    : m_aux{ aux }
    , m_cpu_cycles{ 0 }
    , m_tx{}
    , m_rx{}
    {
        Reset();
    }

    void CMini_UART::Reset()
    {
        m_cpu_cycles = 0;

        // Reset the state machines.
        m_tx = { .state = NState_Machine::Start_Bit, .bit_idx = 0, .fifo = 0 };
        m_rx = { .state = NState_Machine::Start_Bit, .bit_idx = 0, .fifo = 0 };

        // Transmit FIFO is empty.
        m_aux.m_regs[static_cast<std::uint32_t>(CAUX::NRegister::MU_LSR)] |=
        static_cast<std::uint32_t>(NLSR_Flags::Transmitter_Empty);

        // No pending IRQ
        m_aux.m_regs[static_cast<std::uint32_t>(CAUX::NRegister::MU_IIR)] |=
        static_cast<std::uint32_t>(NIIR_Flags::Pending_IRQ);
    }

    void CMini_UART::Clear_IRQ()
    {
        // Clear transmit FIFO flag.
        m_aux.m_regs[static_cast<std::uint32_t>(CAUX::NRegister::MU_IIR)] &=
        ~static_cast<std::uint32_t>(NIIR_Flags::Clear_Transmit_FIFO);

        // Clear receive FIFO flag.
        m_aux.m_regs[static_cast<std::uint32_t>(CAUX::NRegister::MU_IIR)] &=
        ~static_cast<std::uint32_t>(NIIR_Flags::Clear_Receive_FIFO);

        Clear_Global_IRQ_Flags();
    }

    bool CMini_UART::Has_Pending_Receive_IRQ() const
    {
        return static_cast<bool>(m_aux.m_regs[static_cast<std::uint32_t>(CAUX::NRegister::MU_IIR)] &
                                 IIR_IRQ_Receive_Pending);
    }

    bool CMini_UART::Has_Pending_Transmit_IRQ() const
    {
        return static_cast<bool>(m_aux.m_regs[static_cast<std::uint32_t>(CAUX::NRegister::MU_IIR)] &
                                 IIR_IRQ_Transmit_Pending);
    }

    void CMini_UART::Clear_Pending_Receive_IRQ()
    {
        m_aux.m_regs[static_cast<std::uint32_t>(CAUX::NRegister::MU_IIR)] &= IIR_IRQ_Receive_Pending;

        // Is the there is no transit IRQ pending, clear IRQ completely.
        if (!Has_Pending_Transmit_IRQ())
        {
            Clear_Global_IRQ_Flags();
        }
    }

    void CMini_UART::Clear_Global_IRQ_Flags()
    {
        // Clear pending IRQ (inverted logic - 1 means that there is no pending IRQ).
        m_aux.m_regs[static_cast<std::uint32_t>(CAUX::NRegister::MU_IIR)] |=
        static_cast<std::uint32_t>(NIIR_Flags::Pending_IRQ);

        // Clear pending IRQ in the IRQ register of the AUX peripheral.
        m_aux.m_regs[static_cast<std::uint32_t>(CAUX::NRegister::IRQ)] &=
        ~static_cast<std::uint32_t>(CAUX::NAUX_Peripheral::Mini_UART);

        // Notify the interrupt controller.
        m_aux.m_ic->Clear_Pending_IRQ(CInterrupt_Controller::NIRQ_Source::AUX);
    }

    std::pair<std::uint8_t, bool> CMini_UART::Pop_RX_Data()
    {
        // Check if the RX queue is empty.
        if (m_RX_queue.empty())
        {
            return { 0, false };
        }

        // Pop the first element.
        const std::pair<std::uint8_t, bool> data{ m_RX_queue.front(), true };
        m_RX_queue.pop();

        return data;
    }

    void CMini_UART::Set_Transmit_Shift_Reg(std::uint8_t value)
    {
        // Store the value (byte) into the transmit FIFO.
        m_tx.fifo = value;

        // Transmit FIFO is not empty.
        m_aux.m_regs[static_cast<std::uint32_t>(CAUX::NRegister::MU_LSR)] &=
        ~static_cast<std::uint32_t>(NLSR_Flags::Transmitter_Empty);
    }

    bool CMini_UART::Is_Receive_Interrupt_Enabled() const noexcept
    {
        return static_cast<bool>(m_aux.m_regs[static_cast<std::uint32_t>(CAUX::NRegister::MU_IER)] &
                                 static_cast<std::uint32_t>(NIER_Flags::Enable_Receive_Interrupt));
    }

    bool CMini_UART::Is_Transmit_Interrupt_Enabled() const noexcept
    {
        return static_cast<bool>(m_aux.m_regs[static_cast<std::uint32_t>(CAUX::NRegister::MU_IER)] &
                                 static_cast<std::uint32_t>(NIER_Flags::Enable_Transmit_Interrupt));
    }

    CMini_UART::NChar_Length CMini_UART::Get_Char_Length() const noexcept
    {
        return static_cast<NChar_Length>(m_aux.m_regs[static_cast<std::uint32_t>(CAUX::NRegister::MU_LCR)] & 0b1U);
    }

    bool CMini_UART::Is_Transmitter_Enabled() const noexcept
    {
        return static_cast<bool>(m_aux.m_regs[static_cast<std::uint32_t>(CAUX::NRegister::MU_CNTL)] &
                                 static_cast<std::uint32_t>(NCNTL_Flags::Transmitter_Enable));
    }

    bool CMini_UART::Is_Receiver_Enabled() const noexcept
    {
        return static_cast<bool>(m_aux.m_regs[static_cast<std::uint32_t>(CAUX::NRegister::MU_CNTL)] &
                                 static_cast<std::uint32_t>(NCNTL_Flags::Receiver_Enable));
    }

    bool CMini_UART::Is_Transmission_FIFO_Empty() const noexcept
    {
        return static_cast<bool>(m_aux.m_regs[static_cast<std::uint32_t>(CAUX::NRegister::MU_LSR)] &
                                 static_cast<std::uint32_t>(NLSR_Flags::Transmitter_Empty));
    }

    std::uint32_t CMini_UART::Get_Baud_Rate_Counter() const noexcept
    {
        static constexpr std::uint32_t Baud_Rate_Mask = 0xFFFFU;

        return m_aux.m_regs[static_cast<std::uint32_t>(CAUX::NRegister::MU_BAUD)] & Baud_Rate_Mask;
    }

    std::uint32_t CMini_UART::Get_Char_Length_Value(NChar_Length char_length) noexcept
    {
        switch (char_length)
        {
            case NChar_Length::Char_7:
                return 7U;

            case NChar_Length::Char_8:
                return 8U;
        }

        return 0U;
    }

    void CMini_UART::Enable(bool enabled)
    {
        // Set both UART pins to an IDLE state.
        Set_TX_Pin(enabled);
        Set_RX_Pin(enabled); // If RX is not set to IDLE, to incorrectly starts receiving data.
    }

    void CMini_UART::Update()
    {
        // Check if UART is enabled.
        if (!m_aux.Is_Enabled(CAUX::NAUX_Peripheral::Mini_UART))
        {
            return;
        }

        Update_TX(); // Update the TX (transmit) state machine.
        Update_RX(); // Update the RX (receive) state machine.
    }

    void CMini_UART::Update_TX()
    {
        // Make sure we have some data to transfer and that the transmitter has been enabled.
        if (Is_Transmission_FIFO_Empty() || !Is_Transmitter_Enabled())
        {
            return;
        }

        // Transmission state machine
        switch (m_tx.state)
        {
            // Send a start bit.
            case NState_Machine::Start_Bit:
                UART_Send_Start_Bit();
                break;

            // Send the payload (body of the message).
            case NState_Machine::Payload:
                UART_Send_Payload();
                break;

            // Send the stop bit.
            case NState_Machine::Stop_Bit:
                UART_Send_Stop_bit();
                break;

            // Reset transmission.
            case NState_Machine::End_Of_Frame:
                UART_Reset_Transmission();
                break;
        }
    }

    void CMini_UART::Update_RX()
    {
        // Make sure the receiver is enabled.
        if (!Is_Receiver_Enabled())
        {
            return;
        }

        switch (m_rx.state)
        {
            // Receive a start bit.
            case NState_Machine::Start_Bit:
                UART_Receive_Start_Bit();
                break;

            // Receive data (payload).
            case NState_Machine::Payload:
                UART_Receive_Payload();
                break;

            // Receive a stop bit.
            case NState_Machine::Stop_Bit:
                UART_Receive_Stop_Bit();
                break;

            case NState_Machine::End_Of_Frame:
                break;
        }
    }

    void CMini_UART::UART_Receive_Start_Bit()
    {
        // Check if the start bit is detected (voltage level is low).
        if (m_aux.m_gpio->Read_GPIO_Pin(UART_0_RX_PIN_IDX) == CGPIO_Manager::CPin::NState::Low)
        {
            // Move on to receiving the payload.
            m_rx.state = NState_Machine::Payload;
        }
    }

    void CMini_UART::UART_Receive_Payload()
    {
        // Get the current char length
        const std::uint32_t char_length = Get_Char_Length_Value(Get_Char_Length());

        // Read the current value of the RX pin
        const auto bit = static_cast<std::uint8_t>(m_aux.m_gpio->Read_GPIO_Pin(UART_0_RX_PIN_IDX));

        // Add it to the FIFO.
        m_rx.fifo |= static_cast<std::uint8_t>(bit << m_rx.bit_idx);

        // Increment the number of received bits.
        ++m_rx.bit_idx;

        // Check if we have already read the expected number of bits.
        if (m_rx.bit_idx >= char_length)
        {
            // Move on to receiving the stop bit.
            m_rx.state = NState_Machine::Stop_Bit;
        }
    }

    void CMini_UART::UART_Receive_Stop_Bit()
    {
        // The stop bit must be a 1!
        if (m_aux.m_gpio->Read_GPIO_Pin(UART_0_RX_PIN_IDX) != CGPIO_Manager::CPin::NState::High)
        {
            m_aux.m_logging_system.Error("Stop bit was not received correctly");
        }

        m_RX_queue.push(m_rx.fifo);

        // Move the received data from the FIFO to the IO register.
        m_aux.m_regs[static_cast<std::uint32_t>(CAUX::NRegister::MU_IO)] = m_rx.fifo;

        // Data is ready to be read.
        m_aux.m_regs[static_cast<std::uint32_t>(CAUX::NRegister::MU_LSR)] |=
        static_cast<std::uint32_t>(NLSR_Flags::Data_Ready);

        // Reset the state machine.
        m_rx.state = NState_Machine::Start_Bit;
        m_rx.bit_idx = 0;
        m_rx.fifo = 0;

        // Check if an interrupt should be triggered.
        if (Is_Receive_Interrupt_Enabled())
        {
            Trigger_IRQ(true);
        }
    }

    void CMini_UART::Clear_Data_Ready()
    {
        m_aux.m_regs[static_cast<std::uint32_t>(CAUX::NRegister::MU_LSR)] &=
        ~static_cast<std::uint32_t>(NLSR_Flags::Data_Ready);
    }

    void CMini_UART::UART_Send_Start_Bit()
    {
        // Pull the voltage level down.
        Set_TX_Pin(false);

        // Move on to the next state (sending the body of the message).
        m_tx.state = NState_Machine::Payload;
    }

    void CMini_UART::UART_Send_Payload()
    {
        // Get the current char length
        const std::uint32_t char_length = Get_Char_Length_Value(Get_Char_Length());

        // Send another bit from the payload (FIFO).
        Set_TX_Pin(static_cast<bool>(m_tx.fifo & 0b1U));

        // Update the FIFO.
        m_tx.fifo >>= 1U;
        ++m_tx.bit_idx;

        // Check if we have sent the expected number of bits (7/8).
        if (m_tx.bit_idx >= char_length)
        {
            // Move on to the next state (send the stop bit).
            m_tx.state = NState_Machine::Stop_Bit;
        }
    }

    void CMini_UART::UART_Send_Stop_bit()
    {
        // Pull the voltage level up.
        Set_TX_Pin(true);

        // Move on to the next state - terminate the transmission of the current frame.
        m_tx.state = NState_Machine::End_Of_Frame;
    }

    void CMini_UART::Trigger_IRQ(bool receive)
    {
        // Set the type of the pending IRQ.
        Set_Pending_IRQ_Type(receive);

        // Set pending IRQ.
        m_aux.m_regs[static_cast<std::uint32_t>(CAUX::NRegister::MU_IIR)] &=
        ~static_cast<std::uint32_t>(NIIR_Flags::Pending_IRQ);

        // Set pending IRQ in the IRQ register of the AUX peripheral.
        m_aux.m_regs[static_cast<std::uint32_t>(CAUX::NRegister::IRQ)] |=
        static_cast<std::uint32_t>(CAUX::NAUX_Peripheral::Mini_UART);

        // Signalize the interrupt controller.
        m_aux.m_ic->Signalize_IRQ(CInterrupt_Controller::NIRQ_Source::AUX);
    }

    void CMini_UART::Set_Pending_IRQ_Type(bool receive)
    {
        // Clear both flags (receive + transmit).
        m_aux.m_regs[static_cast<std::uint32_t>(CAUX::NRegister::MU_IIR)] &= IIR_IRQ_Type_Mask;

        // Set the type of the pending IRQ.
        m_aux.m_regs[static_cast<std::uint32_t>(CAUX::NRegister::MU_IIR)] |=
        receive ? IIR_IRQ_Receive_Pending : IIR_IRQ_Transmit_Pending;
    }

    void CMini_UART::UART_Reset_Transmission()
    {
        // Reset the transmission state machine.
        m_tx.state = NState_Machine::Start_Bit;
        m_tx.bit_idx = 0;

        // Transmit FIFO is now empty.
        m_aux.m_regs[static_cast<std::uint32_t>(CAUX::NRegister::MU_LSR)] |=
        static_cast<std::uint32_t>(NLSR_Flags::Transmitter_Empty);

        // Check if an interrupt should be triggered.
        if (Is_Transmit_Interrupt_Enabled())
        {
            Trigger_IRQ(false);
        }
    }

    void CMini_UART::Set_TX_Pin(bool set)
    {
        Set_GPIO_pin(UART_0_TX_PIN_IDX, set);
    }

    void CMini_UART::Set_RX_Pin(bool set)
    {
        Set_GPIO_pin(UART_0_RX_PIN_IDX, set);
    }

    void CMini_UART::Set_GPIO_pin(std::uint8_t pin_idx, bool set)
    {
        // Update the state of the given pin.
        // This function is used as if it was an external peripheral ->
        // we're allowed the set the state of an INPUT pin.
        const auto status = m_aux.m_gpio->Set_Pin_State(pin_idx, static_cast<CGPIO_Manager::CPin::NState>(set));

        // Check for errors when setting the pin state.
        switch (status)
        {
            // All went well.
            case IGPIO_Manager::IPin::NPin_Set_Status::OK:
                break;

            // The GPIO pin does not have the right function (alt).
            case IGPIO_Manager::IPin::NPin_Set_Status::Invalid_Pin_Function:
                m_aux.m_logging_system.Error("Invalid function of pin");
                break;

            // Invalid pin (should not happen).
            case IGPIO_Manager::IPin::NPin_Set_Status::Invalid_Pin_Number:
                m_aux.m_logging_system.Error("Invalid pin number");
                break;
        }
    }

    void CMini_UART::Increment_Passed_Cycles(std::uint32_t count)
    {
        m_cpu_cycles += count;

        // Check if it is time to update the state machines based on the current baud rate.
        if (m_cpu_cycles >= Get_Baud_Rate_Counter())
        {
            m_cpu_cycles = 0;
            Update();
        }
    }

} // namespace zero_mate::peripheral