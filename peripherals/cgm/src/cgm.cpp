// ---------------------------------------------------------------------------------------------------------------------
/// \file cgm.cpp
/// \date 27. 04. 2025
/// \author Jaroslav Rosa (rosajaro1352@gmail.com)
///
/// \brief This file implements a CGM that can be connected to I2C pins at runtime as a shared lib.
///
/// You can find more information about 
// ---------------------------------------------------------------------------------------------------------------------

// STL imports (excluded from Doxygen)
/// \cond
#include <cassert>
/// \endcond

// Project file imports

#include "cgm.hpp"

CCgm::CCgm(const std::string name,
	std::uint32_t address,
	std::uint32_t sda_pin_idx,
	std::uint32_t scl_pin_idx,
	zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
	zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
	zero_mate::utils::CLogging_System* logging_system)
	: m_name{ std::move(name) }
	, m_address{ address }
	, m_sda_pin_idx{ sda_pin_idx }
	, m_scl_pin_idx{ scl_pin_idx }
	, m_read_pin{ read_pin }
	, m_set_pin{ set_pin }
	, m_transaction{}
	, m_clock{ 0 }
	, m_sda_rising_edge_timestamp{ 0 }
	, m_scl_rising_edge_timestamp{ 0 }
	, m_sda_prev_state{ false }
	, m_scl_prev_state{ false }
	, m_sda_rising_edge{ false }
	, m_scl_rising_edge{ false }
	, m_processing_cmd{ true }
	, m_lock_incoming_data{ false }
	, m_higher_glycemia{ false }
	, m_lower_glycemia{ false }
	, m_logging_system{ logging_system }
{
	Init_GPIO_Subscription();
	if (!Initialize())
		throw std::runtime_error("Inicialization error");
}

nlohmann::json CCgm::Parse_JSON_File(const std::string& path)
{
	// Open the config file in a read-only mode.
	std::ifstream config_file{ path, std::fstream::in };

	// Make sure the file has been opened successfully.
	if (!config_file)
	{
		m_logging_system->Error("Cannot load");

		// Return an empty JSON object.
		return {};
	}

	try
	{
		// Attempt to parse the file's contents.
		return nlohmann::json::parse(config_file);
	}
	catch ([[maybe_unused]] const std::exception& e)
	{
		// clang-format off
		m_logging_system->Error("Failed to parse the config file");
		// clang-format on

		// Return an empty JSON object.
		return {};
	}
}

bool CCgm::Initialize()
{
	nlohmann::json j = Parse_JSON_File(Cgm_Config_File);

	if (j.contains("configuration") &&
		j["configuration"].is_array() &&
		!j["configuration"].empty())
	{
		if (j["configuration"][0].contains("patient_library") &&
			j["configuration"][0]["patient_library"].is_array() &&
			!j["configuration"][0]["patient_library"].empty())
		{
			std::string patient_library = j["configuration"][0]["patient_library"][0].get<std::string>();

			if (!Resolve_Functions(patient_library)) {
				m_logging_system->Error("Failed to resolve functions");
				return false;
			}

			m_patient_idx = Create_Patient();
			if (m_patient_idx < 0) {
				m_logging_system->Error("Failed to create patient");
				return false;
			}
		}
		else
			return false;

		if (j["configuration"][0].contains("higher_value") &&
			j["configuration"][0]["higher_value"].is_array() &&
			!j["configuration"][0]["higher_value"].empty() &&
			j["configuration"][0]["higher_value"][0].is_number_unsigned())
		{
			this->m_higher_glycemia_value = j["configuration"][0]["higher_value"][0].get<double>();
		}

		if (j["configuration"][0].contains("lower_value") &&
			j["configuration"][0]["lower_value"].is_array() &&
			!j["configuration"][0]["lower_value"].empty() &&
			j["configuration"][0]["lower_value"][0].is_number_unsigned())
		{
			this->m_lower_glycemia_value = j["configuration"][0]["lower_value"][0].get<double>();
		}

		if (j["configuration"][0].contains("start_with") &&
			j["configuration"][0]["start_with"].is_array() &&
			!j["configuration"][0]["start_with"].empty() &&
			j["configuration"][0]["start_with"][0].is_number_unsigned())
		{
			std::uint32_t start_with = j["configuration"][0]["start_with"][0].get<std::uint32_t>();

			if (start_with == 0) {
				this->m_higher_glycemia = false;
				this->m_lower_glycemia = false;
			}
			else if (start_with == 1) {
				this->m_higher_glycemia = true;
				this->m_lower_glycemia = !this->m_higher_glycemia;
			}
			else if (start_with == 2) {
				this->m_lower_glycemia = true;
				this->m_higher_glycemia = !this->m_lower_glycemia;
			}
		}
	}
	else {
		m_logging_system->Error("Could not create patient cause not correct config file");
		return false;
	}

	return true;
}

CCgm::~CCgm()
{
	Terminate_Patient(m_patient_idx);
}

void CCgm::GPIO_Subscription_Callback([[maybe_unused]] std::uint32_t pin_idx)
{
	// Increment the clock and read the current state of the pin.
	++m_clock;
	const bool curr_pin_state = m_read_pin(pin_idx);
	// Call the corresponding callback function.
	if (pin_idx == m_sda_pin_idx)
	{
		// Call SDA callback.
		m_logging_system->Info("SDA_Pin_Change_Callback");
		SDA_Pin_Change_Callback(curr_pin_state);
	}
	else if (pin_idx == m_scl_pin_idx)
	{
		// Call SCL callback.
		m_logging_system->Info("SCL_Pin_Change_Callback");
		SCL_Pin_Change_Callback(curr_pin_state);
	} else 
		m_logging_system->Info("Ani jedno");

	// Check if the master has just sent a stop bit.
	// Definition of a stop bit: SDA goes high after SCL
	const bool stop_bit_detected = m_sda_rising_edge && m_scl_rising_edge &&
		(m_sda_rising_edge_timestamp - m_scl_rising_edge_timestamp) == 1 &&
		((m_transaction.state == NState_Machine::Recieve && (m_transaction.data_idx == 0 || m_transaction.data_idx == 7)) ||
			(m_transaction.state == NState_Machine::Send &&
				m_transaction.length == 0 &&
				m_transaction.data_idx == 0));

	// If a stop bit has just been detected, terminate the current transaction.
	if (stop_bit_detected)
	{
		m_logging_system->Info(("Nastavuju Start_Bit: " + std::to_string(m_transaction.data_idx)).c_str());
		m_transaction.state = NState_Machine::Start_Bit;
		Received_Transaction_Callback();
	}
}

void CCgm::Init_GPIO_Subscription()
{
	m_gpio_subscription.insert(m_sda_pin_idx);
	m_gpio_subscription.insert(m_scl_pin_idx);
}

void CCgm::I2C_Update()
{
	// I2C state machine
	switch (m_transaction.state)
	{
		// Receive the start bit.
	case NState_Machine::Start_Bit:
		m_logging_system->Info("Start_Bit");
		std::cout << "cgm Start_Bit\n";
		I2C_Receive_Start_Bit();
		break;

		// Receive the slave's address.
	case NState_Machine::Address:
		m_logging_system->Info("Address");
		std::cout << "cgm Address\n";
		I2C_Receive_Address();
		break;

		// Receive the RW bit.
	case NState_Machine::RW:
		m_logging_system->Info("RW");
		std::cout << "cgm RW\n";
		I2C_Receive_RW_Bit();
		break;

		// Send the ACK_1 bit.
	case NState_Machine::ACK_1:
		m_logging_system->Info("ACK_1");
		std::cout << "cgm ACK_1\n";
		m_transaction.state = NState_Machine::Recieve;
		break;

	case NState_Machine::ACK_1_Send:
		m_logging_system->Info("ACK_1_Send");
		std::cout << "cgm ACK_1_Send\n";
		m_transaction.state = NState_Machine::Send;
		break;

	case NState_Machine::Recieve:
		m_logging_system->Info("Recieve");
		std::cout << "cgm Recieve\n";
		I2C_Receive_Data();
		break;

		// Receive data (payload).
	case NState_Machine::Send:
		m_logging_system->Info("Send");
		std::cout << "cgm Send\n";
		I2C_Send_Data();
		break;

	case NState_Machine::Receive_ACK:
		m_logging_system->Info("Receive_ACK_2");
		std::cout << "cgm Receive_ACK_2\n";
		I2C_Receive_ACK_2();
		break;

		// Send the ACK_2 bit.
	case NState_Machine::ACK_2:
		m_logging_system->Info("ACK_2");
		std::cout << "cgm ACK_2\n";
		// Move on to receiving another byte.
		m_transaction.data = 0;
		m_transaction.data_idx = Data_Length;
		m_transaction.state = NState_Machine::Recieve;
		break;
	}
}

void CCgm::I2C_Receive_Start_Bit()
{
	if (!m_read_pin(m_sda_pin_idx))
	{
		Init_Transaction();
		m_transaction.state = NState_Machine::Address;
	}
}

void CCgm::I2C_Receive_Address()
{
	--m_transaction.addr_idx;

	// Read the state of the SDA pin and update the address.
	if (m_read_pin(m_sda_pin_idx))
	{
		m_transaction.address |= (0b1U << m_transaction.addr_idx);
	}

	// Have we read all bits of the address yet?
	if (m_transaction.addr_idx == 0)
	{
		m_logging_system->Info("Nastavuju NState_Machine::RW");
		// Move on to receiving the RW bit.
		m_transaction.state = NState_Machine::RW;
	}
}

void CCgm::I2C_Receive_RW_Bit()
{
	// Read the RW bit.
	m_transaction.read = m_read_pin(m_sda_pin_idx);

	if (m_transaction.read && m_address == (m_transaction.address & ~1U) && m_transaction.request_sended) {
		m_logging_system->Info("Nastavuju NState_Machine::Send");
		m_transaction.data = m_output_fifo[0];
		m_transaction.data_idx = Data_Length;
		m_transaction.length = sizeof(float);
		m_transaction.state = NState_Machine::Send;
		Send_ACK();
	}
	else {
		Send_ACK();
		m_logging_system->Info("Nastavuju NState_Machine::ACK_1");
		m_transaction.state = NState_Machine::ACK_1;
	}
}

void CCgm::I2C_Receive_Data()
{
	--m_transaction.data_idx;

	// Read the state of the SDA pin and update the data (receiving byte).
	if (m_read_pin(m_sda_pin_idx))
	{
		m_transaction.data |= (0b1U << m_transaction.data_idx);
	}

	m_logging_system->Info(std::to_string(m_transaction.data_idx).c_str());

	// Have we read all 8 bits of the data yet?
	if (m_transaction.data_idx == 0)
	{
		m_logging_system->Info(std::to_string((m_transaction.address & ~1U)).c_str());
		// Store the data into the FIFO only if it is meant to be for us.
		if (m_address == (m_transaction.address & ~1U))
		{
			m_fifo.push_back(m_transaction.data);
		}

		if (!m_transaction.read && m_address == (m_transaction.address & ~1U)) {
			m_transaction.request_sended = true;
		}
		else {
		}

		// Send an ACK bit to the master device.
		Send_ACK();
		m_transaction.state = NState_Machine::ACK_2;
	}
}

void CCgm::I2C_Send_Data()
{
	if (m_transaction.data_idx > 0) {
		--m_transaction.data_idx;
		// Store the data into the FIFO only if it is meant to be for us.
		if (m_address == (m_transaction.address & ~1U))
		{
			// Get bit that is gonna be sent
			bool bitToSend = (m_transaction.data >> (m_transaction.data_idx % 8)) & 0b1;

			m_logging_system->Info(("data: " + std::to_string(m_transaction.data)).c_str());
			m_logging_system->Info(("data_idx: " + std::to_string(m_transaction.data_idx)).c_str());
			m_logging_system->Info(("bitToSend: " + std::to_string((m_transaction.data >> (m_transaction.data_idx % 8)) & 0b1)).c_str());

			// Set SDA pin to the desired value
			m_set_pin(m_sda_pin_idx, bitToSend);

			// Were all the bits sent?
			if (m_transaction.data_idx == 0)
			{
				// Remove first item from the FIFO.
				if (m_transaction.length > 0)
				{
					m_transaction.state = NState_Machine::Receive_ACK;
					//m_output_fifo.erase(m_output_fifo.begin());
					//m_transaction.data = m_output_fifo[0];
					//m_transaction.data_idx = Data_Length;
				}
				else {
					m_transaction.request_sended = false;
				}
			}
		}
	}
}

void CCgm::SCL_Pin_Change_Callback(bool curr_pin_state)
{
	m_scl_rising_edge = false;

	// Check if SCL just went from HIGH to LOW and the device is awaiting a start bit.
	if (m_scl_prev_state && !curr_pin_state && m_transaction.state == NState_Machine::Start_Bit)
	{
		// Start of a new transaction.
		I2C_Update();
	}
	// Check is there is a rising edge on the SCL pin.
	else if (!m_scl_prev_state && curr_pin_state)
	{
		// Update the timestamp.
		m_scl_rising_edge = true;
		m_scl_rising_edge_timestamp = m_clock;

		// Update the state machine with every rising edge.
		I2C_Update();
	}
	// Update the previous state of the SCL pin.
	m_scl_prev_state = curr_pin_state;
}

void CCgm::SDA_Pin_Change_Callback(bool curr_pin_state)
{
	m_sda_rising_edge = false;

	// Check if there is a rising edge on the SDA pin.
	if (!m_sda_prev_state && curr_pin_state)
	{
		// Update the timestamp.
		m_sda_rising_edge = true;
		m_sda_rising_edge_timestamp = m_clock;
	}

	// Update the previous state of the SDA pin.
	m_sda_prev_state = curr_pin_state;
}

void CCgm::Updated_Type_Of_Processing_Data(std::uint8_t data)
{
	// Check the type of data that has been received.
	const auto cmd = static_cast<NCMD>(data);

	// Are we going to be processing data?
	if (cmd == NCMD::Data_Start || cmd == NCMD::Data_Continue)
	{
		m_processing_cmd = false;
	}
	// Are we going to be processing commands?
	else if (cmd == NCMD::Command_Start)
	{
		m_processing_cmd = true;
	}
}

void CCgm::Log_Received_Data()
{
	std::stringstream ss{};

	ss << "Received data: " << m_fifo.size() << " ";

	for (const auto& value : m_fifo)
	{
		ss << static_cast<std::uint32_t>(value) << " ";
	}

	m_logging_system->Debug(ss.str().c_str());
}

void CCgm::Received_Transaction_Callback()
{
	Log_Received_Data();

	// If the FIFO is empty, there is nothing to do.
	if (m_fifo.empty())
	{
		return;
	}

	// Check what kind of that we will be processing.
	Updated_Type_Of_Processing_Data(m_fifo[0]);

	// Process all data received in the FIFO.
	for (std::size_t idx = 1; idx < m_fifo.size(); ++idx)
	{
		if (m_processing_cmd)
		{
			// Process a command.
			Process_CMD(m_fifo[idx]);
		}
	}
}

void CCgm::Init_Transaction()
{
	m_fifo.clear();

	m_transaction.address = 0x0;
	m_transaction.length = 0;
	m_transaction.data = 0;
	m_transaction.data_idx = Data_Length;
	m_transaction.addr_idx = Slave_Addr_Length;
	m_transaction.read = false;
}

void CCgm::Send_ACK()
{
	// Do NOT send an ACK bit to the master devices, unless they are talking to us.
	if ((m_transaction.address & ~1U) != m_address)
	{
		return;
	}

	// Send an ACK bit.
	const int status = m_set_pin(m_sda_pin_idx, false);

	// Check for any possible errors.
	if (status != 0)
	{
		m_logging_system->Error("Failed to set the value of the SDA pin (ACK)");
	}
}

void CCgm::I2C_Receive_ACK_2()
{
	// The slave device is supposed to pull the voltage low (= ACK_1).
	if (m_read_pin(m_sda_pin_idx))
	{
		// Report any errors.
		m_logging_system->Error("Failed to receive ACK_2");
	}

	--m_transaction.length;
	m_output_fifo.erase(m_output_fifo.begin());
	m_transaction.data = m_output_fifo[0];
	m_transaction.data_idx = Data_Length;
	m_transaction.state = NState_Machine::Send;
}

void CCgm::Process_CMD(std::uint8_t data)
{
	// If the previous command locked the data, do not interpret the is as a command.
	// The command is supposed to unlock it once it has processed all data it required.
	if (!m_lock_incoming_data)
	{
		m_curr_cmd = static_cast<NCMD>(data);
	}

	// Process the current command.
	switch (m_curr_cmd)
	{
	case NCMD::Read_Next_Value: {
		Read_Next_Glycemia();
		break;
	}
	case NCMD::Read_Current_Value: {
		Read_Current_Glycemia();
		break;
	}
	case NCMD::Command_Start:
		[[fallthrough]];
	case NCMD::Data_Start:
	case NCMD::Data_Continue:
		break;
	}
}

void CCgm::Set_ImGui_Context(void* context)
{
	// Store the ImGUI Context.
	m_context = static_cast<ImGuiContext*>(context);
}

void CCgm::Render()
{
	// Make sure the ImGUIContext has been set.
	assert(m_context != nullptr);
	ImGui::SetCurrentContext(m_context);

	// Render the window.
	if (ImGui::Begin(m_name.c_str()))
	{
		Render_Information();
		Render_Pins();
		Render_Cgm();
	}

	ImGui::End();
}

void CCgm::Render_Information() const
{
	ImGui::Text("CGM sensor");
	ImGui::Text("I2C addr = 0x%X (%d dec)", m_address, m_address);
	ImGui::Text("Current glycemia: %f", m_current_glycemia);
}

void CCgm::Render_Pins() const
{
	ImGui::Text("SDA pin: %d", m_sda_pin_idx);
	ImGui::Text("SCL pin: %d", m_scl_pin_idx);
}

void CCgm::Render_Cgm()
{
	if (!m_lower_glycemia && ImGui::Checkbox("Higher glycemia", &m_higher_glycemia))
	{
		if (m_higher_glycemia) {
			m_logging_system->Error("Byla detekovana vyssi glykemie");
		}
		Read_Current_Glycemia();
	}
	else if (!m_higher_glycemia && ImGui::Checkbox("Lower glycemia", &m_lower_glycemia))
	{
		if (m_lower_glycemia) {
			m_logging_system->Error("Byla detekovana nizsi glykemie");
		}
		Read_Current_Glycemia();
	}
}

void CCgm::Read_Next_Glycemia()
{
	if (m_higher_glycemia) {
		m_current_glycemia = m_higher_glycemia_value;
	}
	else if (m_lower_glycemia) {
		m_current_glycemia = m_lower_glycemia_value;
	}
	else {
		if (m_patient_idx < 0) {
			m_logging_system->Error("Failed to create patient");
			return;
		}

		Advance(m_patient_idx);
		m_current_glycemia = Get_Patient_Glucose(m_patient_idx);
	}

	std::stringstream ss{};

	ss << static_cast<double>(m_current_glycemia);

	m_logging_system->Info(("Patient glycemia: " + ss.str()).c_str());

	byte bytes[sizeof(float)];
	memcpy(bytes, &m_current_glycemia, sizeof(float));

	m_output_fifo.clear();
	for (int i = 0; i < sizeof(float); i++) {
		m_output_fifo.push_back(bytes[i]);
	}
}

void CCgm::Read_Current_Glycemia()
{
	if (m_higher_glycemia) {
		m_current_glycemia = m_higher_glycemia_value;
	}
	else if (m_lower_glycemia) {
		m_current_glycemia = m_lower_glycemia_value;
	}
	else {
		if (m_patient_idx < 0) {
			m_logging_system->Error("Failed to create patient");
			return;
		}

		m_current_glycemia = Get_Patient_Glucose(m_patient_idx);
	}

	std::stringstream ss{};

	ss << static_cast<double>(m_current_glycemia);

	m_logging_system->Info(("Patient glycemia: " + ss.str()).c_str());

	byte bytes[sizeof(float)];
	memcpy(bytes, &m_current_glycemia, sizeof(float));

	m_output_fifo.clear();
	for (int i = 0; i < sizeof(float); i++) {
		m_output_fifo.push_back(bytes[i]);
	}
}

extern "C"
{
	zero_mate::IExternal_Peripheral::NInit_Status
		Create_Peripheral(zero_mate::IExternal_Peripheral** peripheral,
			const char* const name,
			const std::uint32_t* const connection,
			std::size_t pin_count,
			zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
			zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
			zero_mate::utils::CLogging_System* logging_system)
	{
		// SDA, SCL, and address
		if (pin_count != 3)
		{
			return zero_mate::IExternal_Peripheral::NInit_Status::GPIO_Mismatch;
		}

		// Create an instance of an Cgm sensor.
		// clang-format off
		*peripheral = new (std::nothrow) CCgm(name,
			connection[2], // Address
			connection[1], // SDA
			connection[0], // SCL
			read_pin,
			set_pin,
			logging_system);
		// clang-format on

		// Make sure the creation was successful.
		if (*peripheral == nullptr)
		{
			return zero_mate::IExternal_Peripheral::NInit_Status::Allocation_Error;
		}

		// All went well.
		return zero_mate::IExternal_Peripheral::NInit_Status::OK;
	}
}
