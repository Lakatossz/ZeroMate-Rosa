// -------------------------------------------------------------------------------------------------------------------- -
/// \file cgm.hpp
/// \date 27. 04. 2025
/// \author Jaroslav Rosa (rosajaro1352@gmail.com)
///
/// \brief This file defines a CGM that can be connected to I2C pins at runtime as a shared lib.
///
/// You can find more information about 
// ---------------------------------------------------------------------------------------------------------------------

#pragma once

// STL imports (excluded from Doxygen)
/// \cond
#include <array>
#include <queue>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <random>
/// \endcond

#include "imgui.h"
#include "nlohmann/json.hpp"

#include "zero_mate/external_peripheral.hpp"

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#ifdef ZM_EXTERNAL_PERIPHERAL_EXPORT
#define ZM_EXTERNAL_PERIPHERAL_API __declspec(dllexport)
#else
#define ZM_EXTERNAL_PERIPHERAL_API __declspec(dllimport)
#endif

using TPatientCreateFnc = int(*)();
using TPatientTerminateFnc = void(*)(int);
using TPatientAdvanceFnc = void(*)(int);
using TPatientGetGlucoseFnc = double(*)(int);
using TPatientDoseInsulinFnc = void(*)(int, double);

namespace {
	TPatientCreateFnc Create_Patient = nullptr;
	TPatientTerminateFnc Terminate_Patient = nullptr;
	TPatientAdvanceFnc Advance = nullptr;
	TPatientGetGlucoseFnc Get_Patient_Glucose = nullptr;
	TPatientDoseInsulinFnc Dose_Insulin = nullptr;
}

bool Resolve_Functions(std::string lib_name) {
	// ifdef _WIN32, load library using Windows functions, else load using dlopen
#ifdef _WIN32
	// Load the DLL
	HMODULE hModule = LoadLibraryA(lib_name.c_str());
	if (hModule == NULL) {
		std::cerr << "Failed to load the DLL" << std::endl;
		return false;
	}
	Create_Patient = (TPatientCreateFnc)GetProcAddress(hModule, "Create_Patient");
	Terminate_Patient = (TPatientTerminateFnc)GetProcAddress(hModule, "Terminate_Patient");
	Advance = (TPatientAdvanceFnc)GetProcAddress(hModule, "Advance");
	Get_Patient_Glucose = (TPatientGetGlucoseFnc)GetProcAddress(hModule, "Get_Patient_Glucose");
	Dose_Insulin = (TPatientDoseInsulinFnc)GetProcAddress(hModule, "Dose_Insulin");
	if (!Create_Patient || !Terminate_Patient || !Advance || !Get_Patient_Glucose || !Dose_Insulin) {
		std::cerr << "Failed to resolve functions" << std::endl;
		return false;
	}
#else
	// Load the shared library
	void* handle = dlopen(lib_name.c_str(), RTLD_LAZY);
	if (!handle) {
		std::cerr << "Failed to load the shared library" << std::endl;
		return false;
	}
	Create_Patient = (TPatientCreateFnc)dlsym(handle, "Create_Patient");
	Terminate_Patient = (TPatientTerminateFnc)dlsym(handle, "Terminate_Patient");
	Advance = (TPatientAdvanceFnc)dlsym(handle, "Advance");
	Get_Patient_Glucose = (TPatientGetGlucoseFnc)dlsym(handle, "Get_Patient_Glucose");
	Dose_Insulin = (TPatientDoseInsulinFnc)dlsym(handle, "Dose_Insulin");
	if (!Create_Patient || !Terminate_Patient || !Advance || !Get_Patient_Glucose || !Dose_Insulin) {
		std::cerr << "Failed to resolve functions" << std::endl;
		return false;
	}
#endif

	return true;
}

// ---------------------------------------------------------------------------------------------------------------------
/// \class CCgm
/// \brief This class represents an CGM (Continuos Glucose Monitor).
// ---------------------------------------------------------------------------------------------------------------------
class ZM_EXTERNAL_PERIPHERAL_API CCgm final : public zero_mate::IExternal_Peripheral
{
public:
	/// Length of the slave address
	static constexpr std::uint8_t Slave_Addr_Length = 7;

	/// Length of a data payload
	static constexpr std::uint8_t Data_Length = 8;

	// -----------------------------------------------------------------------------------------------------------------
	/// \enum NCMD
	/// \brief Enumeration of different commands of the RTC.
	// -----------------------------------------------------------------------------------------------------------------
	enum class NCMD : std::uint8_t
	{
		Command_Start = 0x00,               ///< Command start
		Data_Start = 0xC0,                  ///< Data start
		Data_Continue = 0x40,               ///< Data continue
		Read_Next_Value = 0x81,				///< Read next glycemia value
		Read_Current_Value = 0xA5,					///< Read current glycemia value
	};

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Creates an instance of the class.
	/// \param name Unique name of the device
	/// \param address Address of the device
	/// \param sda_pin_idx SDA pin index
	/// \param scl_pin_idx SCL pin index
	/// \param read_pin Function the peripheral uses to read GPIO pins
	/// \param set_pin Function the peripheral uses to set GPIO pins
	/// \param logging_system Logging system
	// -----------------------------------------------------------------------------------------------------------------
	explicit CCgm(const std::string name,
		std::uint32_t address,
		std::uint32_t sda_pin_idx,
		std::uint32_t scl_pin_idx,
		zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
		zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
		zero_mate::utils::CLogging_System* logging_system);

	~CCgm();

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Notifies the peripheral that the state of one of the pins it subscribes to has changed.
	/// \param pin_idx Index of the GPIO pin whose state has been changed
	// -----------------------------------------------------------------------------------------------------------------
	void GPIO_Subscription_Callback([[maybe_unused]] std::uint32_t pin_idx) override;

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Renders the RTC (GUI).
	// -----------------------------------------------------------------------------------------------------------------
	void Render() override;

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Sets an ImGuiContext, so the DIP switch can render itself as a GUI window.
	/// \param context ImGuiContext the DIP switch uses to render itself
	// -----------------------------------------------------------------------------------------------------------------
	void Set_ImGui_Context(void* context) override;

private:

	/// Path to the external peripherals config file.
	const char* const Cgm_Config_File = "peripherals/cgm_config.json";

	[[nodiscard]] inline nlohmann::json Parse_JSON_File(const std::string& path);

	inline bool Initialize();

	// -----------------------------------------------------------------------------------------------------------------
	/// \enum NState_Machine
	/// \brief Enumeration of different states of the I2C state machine.
	// -----------------------------------------------------------------------------------------------------------------
	enum class NState_Machine : std::uint8_t
	{
		Start_Bit, ///< Receive the start bit
		Address,   ///< Receive the address of the target machine
		RW,        ///< Receive the RW bit
		ACK_1,     ///< Send the ACK_1 bit
		ACK_1_Send,///< Send the ACK_1 bit
		Recieve,   ///< Receive the data payload
		Send,      ///< Sends the data payload
		ACK_2,     ///< Send the ACK_2 bit
		Receive_ACK///< Receive the ACK_2 bit
	};

	// -----------------------------------------------------------------------------------------------------------------
	/// \enum TTransaction
	/// \brief  Representation of a single data transaction.
	// -----------------------------------------------------------------------------------------------------------------
	struct TTransaction
	{
		NState_Machine state{ NState_Machine::Start_Bit };	///< Current state of the state machine
		std::uint32_t address{ 0x0 };						///< Slave address
		std::uint32_t length{ 0 };                         ///< Total number of bytes
		std::uint8_t data{ 0 };                            ///< Total number of bytes
		std::uint8_t addr_idx{ Slave_Addr_Length };			///< Index of the current bit of the slave's address
		std::uint8_t data_idx{ Data_Length };				///< Index of the current bit of the current data payload
		bool read{ false };									///< Is the device being written into?
		bool request_sended{ false };                       ///< Is the device being written into?
	};

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Renders the information of the CGM.
	// -----------------------------------------------------------------------------------------------------------------
	inline void Render_Information() const;

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Renders the number of the GPIO pin the CGM is connected to.
	// -----------------------------------------------------------------------------------------------------------------
	inline void Render_Pins() const;

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Renders the CGM itself.
	// -----------------------------------------------------------------------------------------------------------------
	inline void Render_Cgm();

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Initializes GPIO subscription (what GPIO pins we want to be notified about).
	// -----------------------------------------------------------------------------------------------------------------
	inline void Init_GPIO_Subscription();

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Updates the I2C state machine.
	// -----------------------------------------------------------------------------------------------------------------
	inline void I2C_Update();

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Receives the start bit.
	// -----------------------------------------------------------------------------------------------------------------
	inline void I2C_Receive_Start_Bit();

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Receives the address of the target device.
	// -----------------------------------------------------------------------------------------------------------------
	inline void I2C_Receive_Address();

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Receives the RW bit.
	// -----------------------------------------------------------------------------------------------------------------
	inline void I2C_Receive_RW_Bit();

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Receives a data payload.
	// -----------------------------------------------------------------------------------------------------------------
	inline void I2C_Receive_Data();

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Send a data payload.
	// -----------------------------------------------------------------------------------------------------------------
	inline void I2C_Send_Data();

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief SCL pin change callback.
	/// \param curr_pin_state Current state of the SCL pin
	// -----------------------------------------------------------------------------------------------------------------
	inline void SCL_Pin_Change_Callback(bool curr_pin_state);

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief SDA pin change callback.
	/// \param curr_pin_state Current state of the SDA pin
	// -----------------------------------------------------------------------------------------------------------------
	inline void SDA_Pin_Change_Callback(bool curr_pin_state);

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Updates the type of processing data (Data vs Command).
	/// \param data Raw data (first byte) of the last payload
	// -----------------------------------------------------------------------------------------------------------------
	inline void Updated_Type_Of_Processing_Data(std::uint8_t data);

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Logs received data within the last transaction.
	// -----------------------------------------------------------------------------------------------------------------
	inline void Log_Received_Data();

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Callback of the end of an ongoing transactions (all data has been received).
	// -----------------------------------------------------------------------------------------------------------------
	inline void Received_Transaction_Callback();

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Initializes a new transaction.
	// -----------------------------------------------------------------------------------------------------------------
	inline void Init_Transaction();

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Sends an ACK bit back to the master device.
	// -----------------------------------------------------------------------------------------------------------------
	inline void Send_ACK();

	// -------------------------------------------------------------------------------------------------------------
	/// \brief Checks whether the master device has sent ACK_2 as expected.
	// -------------------------------------------------------------------------------------------------------------
	inline void I2C_Receive_ACK_2();

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Processes a received command.
	/// \param data Raw data (command) that has been received
	// -----------------------------------------------------------------------------------------------------------------
	inline void Process_CMD(std::uint8_t data);

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Returns next glycemia value.
	// -----------------------------------------------------------------------------------------------------------------
	inline void Read_Next_Glycemia();

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Returns current glycemia value.
	// -----------------------------------------------------------------------------------------------------------------
	inline void Read_Current_Glycemia();

	std::string m_name;											///< Unique name of the peripheral
	std::uint32_t m_address;                                    ///< Address of the device
	std::uint32_t m_sda_pin_idx;                                ///< Index of the SDA pin
	std::uint32_t m_scl_pin_idx;                                ///< Index of the SCL pin
	IExternal_Peripheral::Read_GPIO_Pin_t m_read_pin;			///< Function to read the state of a GPIO pin
	IExternal_Peripheral::Set_GPIO_Pin_t m_set_pin;				///< Function to set the state of a GPIO pin
	TTransaction m_transaction;                                 ///< Ongoing transaction
	std::vector<std::uint8_t> m_fifo;                           ///< Data FIFO
	std::vector<std::uint8_t> m_output_fifo;                    ///< Data FIFO
	std::uint32_t m_clock;                                      ///< Emulation of the current time
	std::uint32_t m_sda_rising_edge_timestamp;                  ///< Timestamp of SDA going from LOW to HIGH
	std::uint32_t m_scl_rising_edge_timestamp;                  ///< Timestamp of SCL going from LOW to HIGH
	bool m_sda_prev_state;                                      ///< Previous tate of the SDA pin
	bool m_scl_prev_state;                                      ///< Previous tate of the SCL pin
	bool m_sda_rising_edge;                                     ///< Rising edge detected on SDA?
	bool m_scl_rising_edge;                                     ///< Rising edge detected on SCL?
	bool m_processing_cmd;                                      ///< Are we currently processing a command or data?
	NCMD m_curr_cmd;                                            ///< Current command being processed
	bool m_lock_incoming_data;                                  ///< Should not the next data

	uint32_t m_burn_cycles = 0;

	zero_mate::utils::CLogging_System* m_logging_system;        ///< Logging system

	float m_higher_glycemia_value;
	float m_lower_glycemia_value;
	bool m_higher_glycemia;
	bool m_lower_glycemia;
	float m_current_glycemia;

	int m_patient_idx;

	ImGuiContext* m_context;									///< ImGUI context (rendering the GUI)
};