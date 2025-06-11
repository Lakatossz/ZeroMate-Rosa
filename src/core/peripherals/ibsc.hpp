// ---------------------------------------------------------------------------------------------------------------------
/// \file bsc.hpp
/// \date 20. 08. 2023
/// \author Jakub Silhavy (jakub.silhavy.cz@gmail.com)
///
/// \brief This file defines the BSC peripheral used in BCM2835.
///
/// To find more information about this peripheral, please visit
/// https://www.raspberrypi.org/app/uploads/2012/02/BCM2835-ARM-Peripherals.pdf (chapter 3)
// ---------------------------------------------------------------------------------------------------------------------

#pragma once

// STL imports (excluded from Doxygen)
/// \cond
#include <array>
#include <queue>
/// \endcond

// Project file imports

#include "gpio.hpp"
#include "peripheral.hpp"
#include "system_clock_listener.hpp"
#include "zero_mate/utils/logging_system.hpp"
#include "zero_mate/utils/singleton.hpp"
#include "zero_mate/utils/math.hpp"

namespace zero_mate::peripheral
{
    // -----------------------------------------------------------------------------------------------------------------
    /// \class CBSC
    /// \brief This class represents the BSC peripheral used in BCM2835.
    // -----------------------------------------------------------------------------------------------------------------
    class IBSC : public IPeripheral, public ISystem_Clock_Listener
    {
    public:
        /// I2C SDA (data) pin on the Raspberry Pi Zero board
        static constexpr std::uint32_t SDA_Pin_Idx = 2;

        /// I2C SCL (clock) pin on the Raspberry Pi Zero board
        static constexpr std::uint32_t SCL_Pin_Idx = 3;

        /// Slave address length
        static constexpr std::uint8_t Slave_Addr_Length = 7;

        /// Data length
        static constexpr std::uint8_t Data_Length = 8;

        // Clock speed
        static constexpr std::uint32_t CPU_Cycles_Per_Update = 30;

        // -------------------------------------------------------------------------------------------------------------
        /// \enum NRegister
        /// \brief Enumeration of the BSC registers.
        // -------------------------------------------------------------------------------------------------------------
        enum class NRegister : std::uint32_t
        {
            Control = 0,           ///< Control register
            Status,                ///< Status register
            Data_Length,           ///< Data length (number of bytes to be sent/received)
            Slave_Address,         ///< Address of the target device (slave)
            Data_FIFO,             ///< Interaction with the data FIFO (read/write)
            Clock_Div,             ///< Clock div (not used)
            Data_Delay,            ///< Clock delay (not used)
            Clock_Stretch_Timeout, ///< Clock stretch timeout (not used)
            Count                  ///< Help record (total number of registers)
        };

        // -------------------------------------------------------------------------------------------------------------
        /// \enum NControl_Flags
        /// \brief Enumeration of different flags of the control register.
        // -------------------------------------------------------------------------------------------------------------
        enum class NControl_Flags : std::uint32_t
        {
            I2C_Enable = 0b1U << 15U,    ///< Enable the I2C peripheral
            Start_Transfer = 0b1U << 7U, ///< Begin data transfer
            FIFO_Clear = 4U,             ///< Clear the data FIFO
            Read_Transfer = 0b1U << 0U   ///< Begin read transfer
        };

        // -------------------------------------------------------------------------------------------------------------
        /// \enum NStatus_Flags
        /// \brief Enumeration of different flags of the status register.
        // -------------------------------------------------------------------------------------------------------------
        enum class NStatus_Flags : std::uint32_t
        {
            Transfer_Done = 0b1U << 1U ///< Transfer is done
        };

        /// Total number of the peripheral's registers
        static constexpr auto Number_Of_Registers = static_cast<std::size_t>(NRegister::Count);

        /// Size of a single register
        static constexpr auto Reg_Size = static_cast<std::uint32_t>(sizeof(std::uint32_t));

    public:
        // -------------------------------------------------------------------------------------------------------------
        /// \brief Creates an instance of the class (default non-parameterized constructor).
        // -------------------------------------------------------------------------------------------------------------
        IBSC() = default;

        virtual ~IBSC() = default;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Resets/re-initializes the interrupt controller (IPeripheral interface).
        // -------------------------------------------------------------------------------------------------------------
        virtual void Reset() noexcept = 0;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Returns the size of the peripheral (IPeripheral interface).
        /// \return number of register * register size
        // -------------------------------------------------------------------------------------------------------------
        [[nodiscard]] virtual std::uint32_t Get_Size() const noexcept = 0;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Writes data to the peripheral (IPeripheral interface).
        /// \param addr Relative address (from the peripheral's perspective) where the data will be written
        /// \param data Pointer to the data to be written to the peripheral
        /// \param size Size of the data to be written to the peripheral [B]
        // -------------------------------------------------------------------------------------------------------------
        virtual void Write(std::uint32_t addr, const char* data, std::uint32_t size) = 0;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Reads data from the peripheral (IPeripheral interface).
        /// \param addr Relative address (from the peripheral's perspective) from which the data will be read
        /// \param data Pointer to a buffer the data will be copied into
        /// \param size Size of the data to read from the peripheral [B]
        // -------------------------------------------------------------------------------------------------------------
        virtual void Read(std::uint32_t addr, char* data, std::uint32_t size) = 0;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Notifies the peripheral about how many CPU cycles have passed (ISystem_Clock_Listener interface).
        /// \param count Number of CPU cycles it took to execute the last instruction
        // -------------------------------------------------------------------------------------------------------------
        virtual void Increment_Passed_Cycles(std::uint32_t count) = 0;

    private:
        // -------------------------------------------------------------------------------------------------------------
        /// \enum NState_Machine
        /// \brief Enumeration of different states of the I2C state machine.
        // -------------------------------------------------------------------------------------------------------------
        enum class NState_Machine : std::uint8_t
        {
            Start_Bit, ///< Send a start bit (start of a transaction)
            Address,   ///< Send slave's address
            RW,        ///< Are we going to read from the device or write to it?
            ACK_1,     ///< The slave device is supposed to send ACK_1
            Send,      ///< Data payload
            Recieve,   ///< Data payload
            ACK_2,     ///< The slave device is supposed to send ACK_2
            Send_ACK_2,///< The slave device is supposed to send ACK_2
            Stop_Bit   ///< Stop bit (end of a transaction)
        };

        // -------------------------------------------------------------------------------------------------------------
        /// \enum TTransaction
        /// \brief Representation of a single data transaction.
        // -------------------------------------------------------------------------------------------------------------
        struct TTransaction
        {
            NState_Machine state{ NState_Machine::Start_Bit }; ///< Current state of the state machine
            std::uint32_t address{ 0x0 };                      ///< Slave address
            std::uint32_t length{ 0 };                         ///< Total number of bytes
            std::uint8_t data{ 0 };                            ///< Total number of bytes
            std::uint8_t addr_idx{ Slave_Addr_Length };        ///< Index of the current bit of the slave's address
            std::uint8_t data_idx{ Data_Length };              ///< Index of the current bit of the current data payload
            bool read{ false };                                ///< Are we going to read from the device or write to it?
            bool request_sended{ false };                      ///< Are we going to read from the device or write to it?
        };

        // -------------------------------------------------------------------------------------------------------------
        /// \enum NSCL_State
        /// \brief State of the clock signal.
        // -------------------------------------------------------------------------------------------------------------
        enum class NSCL_State
        {
            SDA_Change, ///< SDA shall be updated
            SCL_Low,    ///< SCL shall go low
            SCL_High    ///< SCL shall go high
        };

    private:
        // -------------------------------------------------------------------------------------------------------------
        /// \brief Adds data to the FIFO.
        // -------------------------------------------------------------------------------------------------------------
        virtual inline void Add_Data_To_FIFO() = 0;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Performs actions after the control register has been written to.
        // -------------------------------------------------------------------------------------------------------------
        virtual inline void Control_Reg_Callback() = 0;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Clears the FIFO.
        // -------------------------------------------------------------------------------------------------------------
        virtual inline void Clear_FIFO() = 0;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Checks whether a new data transfer (transaction) should begin.
        /// \return true, if a new transaction should begin. false otherwise.
        // -------------------------------------------------------------------------------------------------------------
        [[nodiscard]] virtual inline bool Should_Transaction_Begin() = 0;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Checks whether the FIFO should be cleared or not.
        /// \return true, if the FIFO should be cleared. false otherwise.
        // -------------------------------------------------------------------------------------------------------------
        [[nodiscard]] virtual inline bool Should_FIFO_Be_Cleared() = 0;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Sets the state of a given GPIO pin.
        /// \param pin_idx Index of the GPIO pin whose state will be set
        /// \param set Should the state of the pin be set to high or low?
        // -------------------------------------------------------------------------------------------------------------
        virtual inline void Set_GPIO_pin(std::uint8_t pin_idx, bool set) = 0;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Sets the state of a given GPIO pin.
        /// \param pin_idx Index of the GPIO pin whose state will be read
        /// \return true, if the pin is in high state. false otherwise.
        // -------------------------------------------------------------------------------------------------------------
        virtual inline bool Read_GPIO_pin(std::uint8_t pin_idx) = 0;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Terminates an ongoing transaction (stop bit).
        // -------------------------------------------------------------------------------------------------------------
        virtual inline void Terminate_Transaction() = 0;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Updates the I2C state machine.
        // -------------------------------------------------------------------------------------------------------------
        virtual void I2C_Update() = 0;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Sends a start bit (start of a frame).
        // -------------------------------------------------------------------------------------------------------------
        virtual inline void I2C_Send_Start_Bit() = 0;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Sends another bit of the slave's address.
        // -------------------------------------------------------------------------------------------------------------
        virtual inline void I2C_Send_Slave_Address() = 0;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Sends the RW bit.
        // -------------------------------------------------------------------------------------------------------------
        virtual inline void I2C_Send_RW_Bit() = 0;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Checks whether the slave device has sent ACK_1 as expected.
        // -------------------------------------------------------------------------------------------------------------
        virtual inline void I2C_Receive_ACK_1() = 0;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Sends another bit of the data payload.
        // -------------------------------------------------------------------------------------------------------------
        virtual inline void I2C_Send_Data() = 0;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Recieves another bit of the data payload.
        // -------------------------------------------------------------------------------------------------------------
        virtual inline void I2C_Recieve_Data() = 0;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Checks whether the slave device has sent ACK_2 as expected.
        // -------------------------------------------------------------------------------------------------------------
        virtual inline void I2C_Receive_ACK_2() = 0;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Send bit back to the slave device.
        // -------------------------------------------------------------------------------------------------------------
        virtual inline void I2C_Send_ACK() = 0;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Sends a stop bit (end of a frame).
        // -------------------------------------------------------------------------------------------------------------
        virtual inline void I2C_Send_Stop_Bit() = 0;
    };

} // namespace zero_mate::peripheral