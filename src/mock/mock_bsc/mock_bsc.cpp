// ---------------------------------------------------------------------------------------------------------------------
/// \file bsc.cpp
/// \date 20. 08. 2023
/// \author Jakub Silhavy (jakub.silhavy.cz@gmail.com)
///
/// \brief This file implements the BSC peripheral used in BCM2835.
///
/// To find more information about this peripheral, please visit
/// https://www.raspberrypi.org/app/uploads/2012/02/BCM2835-ARM-Peripherals.pdf (chapter 3)
// ---------------------------------------------------------------------------------------------------------------------

// STL imports (excluded from Doxygen)
/// \cond
#include <bit>
#include <algorithm>
/// \endcond

// Project file imports

#include "mock_bsc.hpp"

namespace zero_mate::peripheral
{
    Mock_BSC::Mock_BSC(std::shared_ptr<IGPIO_Manager> gpio) noexcept
        : m_gpio{ gpio }
        , m_cpu_cycles{ 0 }
        , m_transaction_in_progress{ false }
        , m_transaction{}
        , m_SCL_state{ NSCL_State::SDA_Change }
    {
    }

    void Mock_BSC::Reset() noexcept
    {
        std::fill(m_regs.begin(), m_regs.end(), 0);
    }

    std::uint32_t Mock_BSC::Get_Size() const noexcept
    {
        return static_cast<std::uint32_t>(sizeof(m_regs));
    }

    void Mock_BSC::Write(std::uint32_t addr, const char* data, std::uint32_t size)
    {
        const std::size_t reg_idx = addr;
        const auto reg_type = static_cast<NRegister>(reg_idx);

        // Write data to the peripheral's registers.
        std::copy_n(data, size, &std::bit_cast<char*>(m_regs.data())[addr]);

        switch (reg_type)
        {
            // Add data to the FIFO.
        case NRegister::Data_FIFO:
            //Add_Data_To_FIFO();
            m_fifo.push(static_cast<unsigned char>(*data));
            break;

            // The control register has been changed.
        case NRegister::Control:
            Control_Reg_Callback();
            break;

        case NRegister::Status:
            m_regs[static_cast<std::uint32_t>(NRegister::Status)] = static_cast<std::uint32_t>(*data);
            break;
        case NRegister::Data_Length:
            m_regs[static_cast<std::uint32_t>(NRegister::Data_Length)] = static_cast<std::uint32_t>(*data);
            break;
        case NRegister::Slave_Address:
            m_regs[static_cast<std::uint32_t>(NRegister::Slave_Address)] = static_cast<std::uint32_t>(*data);
            break;
        case NRegister::Clock_Div:
        case NRegister::Data_Delay:
        case NRegister::Clock_Stretch_Timeout:
        case NRegister::Count:
            break;
        }
    }

    void Mock_BSC::Control_Reg_Callback()
    {
        // Check if the FIFO should be cleared.
        if (Should_FIFO_Be_Cleared())
        {
            Clear_FIFO();
        }

        // Check if a new transaction should begin.
        if (Should_Transaction_Begin())
        {
            // Initialize the transaction.
            m_transaction.state = NState_Machine::Start_Bit;
            m_transaction.address = m_regs[static_cast<std::uint32_t>(NRegister::Slave_Address)];
            m_transaction.addr_idx = Slave_Addr_Length;
            m_transaction.data_idx = Data_Length;
            m_transaction.length = m_regs[static_cast<std::uint32_t>(NRegister::Data_Length)];
            m_transaction.read = static_cast<bool>(m_regs[static_cast<std::uint32_t>(NRegister::Control)] &
                static_cast<std::uint32_t>(NControl_Flags::Read_Transfer));

            // Clear the transfer done bit.
            m_regs[static_cast<std::uint32_t>(NRegister::Status)] &=
                ~static_cast<std::uint32_t>(NStatus_Flags::Transfer_Done);

            // Pull voltage on both SDA and SCL to high.
            Set_GPIO_pin(SDA_Pin_Idx, true);
            Set_GPIO_pin(SCL_Pin_Idx, true);

            // We are ready to start a new transaction.
            m_cpu_cycles = 0;
            m_transaction_in_progress = true;
            m_SCL_state = NSCL_State::SDA_Change;
        }
    }

    void Mock_BSC::Clear_FIFO()
    {
        // Pop out all items from the FIFO.
        while (!m_fifo.empty())
        {
            m_fifo.pop();
        }
    }

    bool Mock_BSC::Should_FIFO_Be_Cleared()
    {
        // clang-format off
        return (m_regs[static_cast<std::uint32_t>(NRegister::Control)] >>
            static_cast<std::uint32_t>(NControl_Flags::FIFO_Clear) & 0b11U) != 0;
        // clang-format on
    }

    void Mock_BSC::Add_Data_To_FIFO()
    {
        m_fifo.push(static_cast<std::uint8_t>(m_regs[static_cast<std::uint32_t>(NRegister::Data_FIFO)]));
    }

    bool Mock_BSC::Should_Transaction_Begin()
    {
        // Is I2C enabled?
        const auto i2c_enabled = static_cast<bool>(m_regs[static_cast<std::uint32_t>(NRegister::Control)] &
            static_cast<std::uint32_t>(NControl_Flags::I2C_Enable));

        // Is the start transfer bit set?
        const auto start_transfer = static_cast<bool>(m_regs[static_cast<std::uint32_t>(NRegister::Control)] &
            static_cast<std::uint32_t>(NControl_Flags::Start_Transfer));

        // Both conditions must be satisfied.
        return i2c_enabled && start_transfer;
    }

    void Mock_BSC::Read(std::uint32_t addr, char* data, std::uint32_t size)
    {
        const std::size_t reg_idx = addr;
        const auto reg_type = static_cast<NRegister>(reg_idx);

        switch (reg_type)
        {
            // Move the top element from the FIFO to the Data_FIFO register.
            // This has to be done before the registers are read from.
        case NRegister::Data_FIFO:
            if (!m_fifo.empty())
            {
                m_regs[static_cast<std::uint32_t>(NRegister::Data_FIFO)] = m_fifo.front();
                m_fifo.pop();
            }
            break;

        case NRegister::Control:
            [[fallthrough]];
        case NRegister::Status:
        case NRegister::Data_Length:
        case NRegister::Slave_Address:
        case NRegister::Clock_Div:
        case NRegister::Data_Delay:
        case NRegister::Clock_Stretch_Timeout:
        case NRegister::Count:
            break;
        }

        // Read data from the peripheral's registers.
        std::copy_n(&m_regs[static_cast<std::uint32_t>(reg_idx)], size, data);
    }

    void Mock_BSC::I2C_Update()
    {
        switch (m_transaction.state)
        {
            // Send the start bit.
        case NState_Machine::Start_Bit:
            std::cout << "bsc Start_Bit\n";
            I2C_Send_Start_Bit();
            break;

            // Send the slave's address.
        case NState_Machine::Address:
            std::cout << "bsc Address\n";
            I2C_Send_Slave_Address();
            break;

            // Send the RW bit.
        case NState_Machine::RW:
            std::cout << "bsc RW\n";
            I2C_Send_RW_Bit();
            break;

            // Receive the ACK_1 bit.
        case NState_Machine::ACK_1:
            std::cout << "bsc ACK_1\n";
            I2C_Receive_ACK_1();
            break;

            // Send the data payload.
        case NState_Machine::Send:
            std::cout << "bsc Send\n";
            I2C_Send_Data();
            break;

            // Send the data payload.
        case NState_Machine::Recieve:
            std::cout << "bsc Recieve\n";
            I2C_Recieve_Data();
            break;

            // Receive the ACK_2 bit.
        case NState_Machine::ACK_2:
            std::cout << "bsc ACK_2\n";
            I2C_Receive_ACK_2();
            break;

        case NState_Machine::Send_ACK_2:
            std::cout << "bsc Send_ACK_2\n";
            I2C_Send_ACK();
            break;

            // Send the stop bit.
        case NState_Machine::Stop_Bit:
            std::cout << "bsc Stop_Bit\n";
            I2C_Send_Stop_Bit();
            break;
        }
    }

    void Mock_BSC::I2C_Send_Start_Bit()
    {
        // SDA goes low.
        Set_GPIO_pin(SDA_Pin_Idx, false);

        // Move on to sending the address of the target device.
        m_transaction.state = NState_Machine::Address;
    }

    void Mock_BSC::I2C_Send_Slave_Address()
    {
        // Get the current bit of the slave's address.
        --m_transaction.addr_idx;
        const auto curr_bit = static_cast<bool>((m_transaction.address >> m_transaction.addr_idx) & 0b1U);

        // Send the bit out to the target device.
        Set_GPIO_pin(SDA_Pin_Idx, curr_bit);

        // Have we sent all bits of the slave's address?
        if (m_transaction.addr_idx == 0)
        {
            // Move on to sending the RW bit.
            m_transaction.state = NState_Machine::RW;
        }
    }

    void Mock_BSC::I2C_Send_RW_Bit()
    {
        // Send the RW bit to the device and move on to receiving the ACK_1 bit.
        Set_GPIO_pin(SDA_Pin_Idx, m_transaction.read);
        m_transaction.state = NState_Machine::ACK_1;
    }

    void Mock_BSC::I2C_Receive_ACK_1()
    {
        // The slave device is supposed to pull the voltage low (= ACK_1).
        if (m_gpio->Read_GPIO_Pin(SDA_Pin_Idx) != IGPIO_Manager::IPin::NState::Low)
        {
            // Report any errors.
        }

        // Move on to sending the data payload itself.
        if (!m_transaction.read) {
            m_transaction.state = NState_Machine::Send;
        }
        else {
            //m_transaction.length = Data_Length; TODO vyzkouset
            m_transaction.data_idx = 8;
            m_transaction.state = NState_Machine::Recieve;
        }
    }

    void Mock_BSC::I2C_Send_Data()
    {
        bool curr_bit{ false };
        --m_transaction.data_idx;

        // Make sure there's data in the FIFO.
        if (m_fifo.empty())
        {
            
        }
        else
        {
            curr_bit = static_cast<bool>(static_cast<std::uint8_t>(m_fifo.front() >> m_transaction.data_idx) & 0b1U);
        }

        // Send out the current bit of the data payload.
        Set_GPIO_pin(SDA_Pin_Idx, curr_bit);

        // Have we sent all 8 bits already?
        if (m_transaction.data_idx == 0)
        {
            // Remove the data from the FIFO.
            if (!m_fifo.empty())
            {
                m_fifo.pop();
            }

            // Move on to receiving the ACK_2 bit.
            m_transaction.state = NState_Machine::ACK_2;
        }
    }

    void Mock_BSC::I2C_Recieve_Data()
    {
        bool curr_bit{ false };
        --m_transaction.data_idx;

        // Read the current bit of the data payload.
        curr_bit = Read_GPIO_pin(SDA_Pin_Idx);

        if (curr_bit)
        {
            m_transaction.data |= (0b1U << (m_transaction.data_idx % 8));
        }

        if (m_transaction.data_idx == 0)
        {
            m_fifo.push(m_transaction.data);
            m_transaction.data = { 0 };
            if (m_transaction.length == 0) {
                m_transaction.state = NState_Machine::Stop_Bit;
            }
            else {
                m_transaction.state = NState_Machine::Send_ACK_2;
            }
        }
    }

    void Mock_BSC::I2C_Receive_ACK_2()
    {
        // The slave device is supposed to pull the voltage low (= ACK_1).
        if (m_gpio->Read_GPIO_Pin(SDA_Pin_Idx) != IGPIO_Manager::IPin::NState::Low)
        {
            // Report any errors.
        }

        // We have finished sending out another byte of data.
        --m_transaction.length;

        if (!m_transaction.read)
        {
            if (m_transaction.length != 0)
            {
                // Move on to sending another byte of data.
                m_transaction.state = NState_Machine::Send;
                m_transaction.data_idx = Data_Length;
                //m_transaction.state = NState_Machine::Stop_Bit;
            }
            else {
                // Let us terminate the transaction.
                m_transaction.state = NState_Machine::Stop_Bit;
            }
        }
        else if (!m_transaction.request_sended) {
            m_transaction.state = NState_Machine::Start_Bit;
            m_transaction.request_sended = true;
        }
        else {
            m_transaction.state = NState_Machine::Recieve;
            m_transaction.request_sended = false;
        }
    }

    void Mock_BSC::I2C_Send_ACK() {
        // Send an ACK bit.
        Set_GPIO_pin(SDA_Pin_Idx, false);

        if (m_transaction.length != 0)
        {
            // Move on to sending another byte of data.
            --m_transaction.length;
            m_transaction.data_idx = Data_Length;
            m_transaction.state = NState_Machine::Recieve;
        }
        else {
            // Let us terminate the transaction.
            m_transaction.state = NState_Machine::Stop_Bit;
        }
    }

    void Mock_BSC::I2C_Send_Stop_Bit()
    {
        // SCL goes high before SDA.
        Set_GPIO_pin(SCL_Pin_Idx, true);

        // Terminate the transaction.
        Terminate_Transaction();
    }

    void Mock_BSC::Increment_Passed_Cycles(std::uint32_t count)
    {
        if (!m_transaction_in_progress)
        {
            return;
        }

        m_cpu_cycles += count;

        if (m_cpu_cycles < (CPU_Cycles_Per_Update / 3))
        {
            return;
        }

        m_cpu_cycles = 0;

        switch (m_SCL_state)
        {
        case NSCL_State::SDA_Change:
            I2C_Update();
            m_SCL_state = NSCL_State::SCL_High;
            break;

        case NSCL_State::SCL_High:
            Set_GPIO_pin(SCL_Pin_Idx, true);
            m_SCL_state = NSCL_State::SCL_Low;
            break;

        case NSCL_State::SCL_Low:
            Set_GPIO_pin(SCL_Pin_Idx, false);
            m_SCL_state = NSCL_State::SDA_Change;
            break;
        }
    }

    void Mock_BSC::Terminate_Transaction()
    {
        // We are done with the current transaction.
        m_transaction_in_progress = false;

        // SDA goes high after SCL.
        Set_GPIO_pin(SDA_Pin_Idx, true);

        // Set the transfer done bit.
        m_regs[static_cast<std::uint32_t>(NRegister::Status)] |=
            static_cast<std::uint32_t>(NStatus_Flags::Transfer_Done);

        // Clear the start transfer bit.
        m_regs[static_cast<std::uint32_t>(NRegister::Control)] &=
            ~static_cast<std::uint32_t>(NControl_Flags::Start_Transfer);

        // Clear the clear FIFO bit.
        m_regs[static_cast<std::uint32_t>(NRegister::Control)] &=
            ~(0b11U << static_cast<std::uint32_t>(NControl_Flags::FIFO_Clear));
    }

    void Mock_BSC::Set_GPIO_pin(std::uint8_t pin_idx, bool set)
    {
        // Set the state of the given pin.
        const auto status = m_gpio->Set_Pin_State(pin_idx, static_cast<IGPIO_Manager::IPin::NState>(set));

        // Check for any possible errors.
        switch (status)
        {
        case IGPIO_Manager::IPin::NPin_Set_Status::OK:
            break;

        case IGPIO_Manager::IPin::NPin_Set_Status::Invalid_Pin_Function:
            break;

        case IGPIO_Manager::IPin::NPin_Set_Status::Invalid_Pin_Number:
            break;
        }
    }

    bool Mock_BSC::Read_GPIO_pin(std::uint8_t pin_idx)
    {
        // Set the state of the given pin.
        const auto status = m_gpio->Read_GPIO_Pin(pin_idx);

        return status == IGPIO_Manager::IPin::NState::High;
    }

} // namespace zero_mate::peripheral
