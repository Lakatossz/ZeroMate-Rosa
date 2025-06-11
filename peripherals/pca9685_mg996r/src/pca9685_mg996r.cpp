// ----------------------------------------------------------------------------------------------------------------------
/// \file pca9685_mg996r.hpp
/// \date 13. 04. 2025
/// \author Jaroslav Rosa (rosajaro1352@gmail.com)
///
/// \brief This file implements a combination of PCA985 PWM regulator and MG996R servo motor that 
/// can be connected to I2C pins at runtime as a shared library.
///
/// You can find more information about the PWM regulator itself over 
/// at https://www.laskakit.cz/user/related_files/pca9685.pdf
/// and more information about the servo motor itself over 
/// at https://www.electronicoscaldas.com/datasheet/MG996R_Tower-Pro.pdf
// ----------------------------------------------------------------------------------------------------------------------

// STL imports (excluded from Doxygen)
/// \cond
#include <cassert>
/// \endcond

// Project file imports

#include "pca9685_mg996r.hpp"

CPca9685_Mg996r::CPca9685_Mg996r(const std::string& name,
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
	, m_status{ 0 }
	, m_error_probability_referenced{ 0 }
	, m_motor_failure{ false }
	, m_current_angle{ 0 }
	, m_first_part_loeaded{ false }
	, m_steps_to_move{ 0 }
	, m_patient_idx{ 0 }
	, m_logging_system{ logging_system }
{
	Init_GPIO_Subscription();
	if (!Init_Motor_State())
		throw std::runtime_error("Inicialization error");
}

CPca9685_Mg996r::~CPca9685_Mg996r()
{
	if (m_patient_idx >= 0)
		Terminate_Patient(m_patient_idx);
}

void CPca9685_Mg996r::Set_ImGui_Context(void* context)
{
	// Store the ImGUI Context.
	m_context = static_cast<ImGuiContext*>(context);
}

void CPca9685_Mg996r::Init_GPIO_Subscription()
{
	m_gpio_subscription.insert(m_sda_pin_idx);
	m_gpio_subscription.insert(m_scl_pin_idx);
}

nlohmann::json CPca9685_Mg996r::Parse_JSON_File(const std::string& path)
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

bool CPca9685_Mg996r::Init_Motor_State()
{
	nlohmann::json j = Parse_JSON_File(Pca9685_Mg996r_Config_File);

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

		if (j["configuration"][0].contains("error_chance") &&
			j["configuration"][0]["error_chance"].is_array() &&
			!j["configuration"][0]["error_chance"].empty() &&
			j["configuration"][0]["error_chance"][0].is_number_unsigned())
		{
			this->m_error_probability_referenced = j["configuration"][0]["error_chance"][0].get<std::uint32_t>();
			this->m_error_probability = this->m_error_probability_referenced;
		}
	}
	else {
		m_logging_system->Error("Could not create patient cause not correct config file");
		return false;
	}

	return true;
}

void CPca9685_Mg996r::Render()
{
	// Make sure the ImGUIContext has been set.
	assert(m_context != nullptr);
	ImGui::SetCurrentContext(m_context);

	// Render the window.
	if (ImGui::Begin(m_name.c_str()))
	{
		Render_Information();
		Render_Pin_Idx();
		Render_Motor();
	}

	ImGui::End();
}

void CPca9685_Mg996r::Render_Information() const
{
	ImGui::Text("PCA9685 and MG996R");
	ImGui::Text("I2C addr = 0x%X (%d dec)", m_address, m_address);
	ImGui::Text("Angle = %d", m_current_angle);
}

void CPca9685_Mg996r::GPIO_Subscription_Callback([[maybe_unused]] std::uint32_t pin_idx)
{
	// Increment the clock and read the current state of the pin.
	++m_clock;
	const bool curr_pin_state = m_read_pin(pin_idx);

	// Call the corresponding callback function.
	if (pin_idx == m_sda_pin_idx)
	{
		// Call SDA callback.
		SDA_Pin_Change_Callback(curr_pin_state);
	}
	else if (pin_idx == m_scl_pin_idx)
	{
		// Call SCL callback.
		SCL_Pin_Change_Callback(curr_pin_state);
	}

	// Check if the master has just sent a stop bit.
	// Definition of a stop bit: SDA goes high after SCL
	const bool stop_bit_detected = m_sda_rising_edge && m_scl_rising_edge &&
									(m_sda_rising_edge_timestamp - m_scl_rising_edge_timestamp) == 1 &&
									((m_transaction.state == NState_Machine::Recieve && (m_transaction.data_idx == 0 || m_transaction.data_idx == 7)) ||
										(m_transaction.state == NState_Machine::Send && 
											m_transaction.length == 0 &&
											m_transaction.data_idx == 0) || 
										(m_transaction.state == NState_Machine::ACK_2 && m_address == (m_transaction.address & ~1U)));

	// If a stop bit has just been detected, terminate the current transaction.
	if (stop_bit_detected)
	{
		m_logging_system->Info(("Nastavuju Start_Bit: " + std::to_string(m_transaction.data_idx)).c_str());
		m_transaction.state = NState_Machine::Start_Bit;
		Received_Transaction_Callback();
	}
}

void CPca9685_Mg996r::SCL_Pin_Change_Callback(bool curr_pin_state)
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

void CPca9685_Mg996r::SDA_Pin_Change_Callback(bool curr_pin_state)
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

void CPca9685_Mg996r::I2C_Update()
{
	// I2C state machine
	switch (m_transaction.state)
	{
	// Receive the start bit.
	case NState_Machine::Start_Bit:
		m_logging_system->Info("NState_Machine::Start_Bit");
		I2C_Receive_Start_Bit();
		break;

	// Receive the slave's address.
	case NState_Machine::Address:
		m_logging_system->Info("Address");
		I2C_Receive_Address();
		break;

	// Receive the RW bit.
	case NState_Machine::RW:
		m_logging_system->Info("RW");
		I2C_Receive_RW_Bit();
		break;

	// Send the ACK_1 bit.
	case NState_Machine::ACK_1:
		m_logging_system->Info("ACK_1");
		m_transaction.state = NState_Machine::Recieve;
		break;

	case NState_Machine::ACK_1_Send:
		m_logging_system->Info("ACK_1_Send");
		m_transaction.state = NState_Machine::Send;
		break;

	case NState_Machine::Recieve:
		m_logging_system->Info("Recieve");
		I2C_Receive_Data();
		break;

		// Receive data (payload).
	case NState_Machine::Send:
		m_logging_system->Info("Send");
		I2C_Send_Data();
		break;

	case NState_Machine::Receive_ACK:
		m_logging_system->Info("Receive_ACK_2");
		I2C_Receive_ACK_2();
		break;

		// Send the ACK_2 bit.
	case NState_Machine::ACK_2:
		m_transaction.state = NState_Machine::Recieve;
		m_logging_system->Info("ACK_2");
		// Move on to receiving another byte.
		//m_transaction.request_sended = false;
		m_transaction.data = 0;
		m_transaction.data_idx = Data_Length;
		break;
	}
}

void CPca9685_Mg996r::Log_Received_Data()
{
	std::stringstream ss{};

	ss << "Received data: " << m_fifo.size() << " ";

	for (const auto& value : m_fifo)
	{
		ss << static_cast<std::uint32_t>(value) << " ";
	}

	m_logging_system->Debug(ss.str().c_str());
}

void CPca9685_Mg996r::Updated_Type_Of_Processing_Data(std::uint8_t data)
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

void CPca9685_Mg996r::Received_Transaction_Callback()
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
		else
		{
			// Process data (pixel values).
			Process_Data(m_fifo[idx]);
		}
	}
}

void CPca9685_Mg996r::Process_Data(std::uint8_t data)
{
	if (m_fifo.size() > 2) {
		uint16_t combined = (static_cast<uint16_t>(m_fifo[1]) << 8) | m_fifo[2];

		m_logging_system->Info(std::to_string(combined).c_str());
	}

	// Go through individual bits of the received byte of data.
	for (std::int8_t i = 0U; i < std::numeric_limits<std::uint8_t>::digits; ++i)
	{
		// TODO
	}
}

void CPca9685_Mg996r::Process_CMD(std::uint8_t data)
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
	case NCMD::Push_Forward:
		Push_Forward(data);
		break;
	case NCMD::Push_Backwards:
		Push_Backward(data);
		break;
	case NCMD::Reset_Position:
		Reset_Position();
		break;
	case NCMD::Read_Status:
		Read_Status();
		break;
	case NCMD::Command_Start:
		[[fallthrough]];
	case NCMD::Data_Start:
	case NCMD::Data_Continue:
		break;
	}
}

void CPca9685_Mg996r::I2C_Receive_Start_Bit()
{
	if (!m_read_pin(m_sda_pin_idx))
	{
		Init_Transaction();
		m_transaction.state = NState_Machine::Address;
	}
}

void CPca9685_Mg996r::Init_Transaction()
{
	m_fifo.clear();

	m_transaction.address = 0x0;
	m_transaction.data = 0;
	m_transaction.length = 0;
	m_transaction.data_idx = Data_Length;
	m_transaction.addr_idx = Slave_Addr_Length;
	m_transaction.read = false;
}

void CPca9685_Mg996r::I2C_Receive_Address()
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

void CPca9685_Mg996r::I2C_Receive_RW_Bit()
{
	// Read the RW bit.
	m_transaction.read = m_read_pin(m_sda_pin_idx);

	m_logging_system->Info(std::to_string((m_transaction.address & ~1U)).c_str());

	Send_ACK();

	if (m_transaction.read && m_address == (m_transaction.address & ~1U) && m_transaction.request_sended) {
		m_logging_system->Info("Nastavuju NState_Machine::Send");
		m_transaction.data = m_output_fifo[0];
		m_transaction.data_idx = Data_Length;
		m_transaction.length = sizeof(float);
		m_transaction.state = NState_Machine::Send;
	} else 
		m_transaction.state = NState_Machine::ACK_1;
}

void CPca9685_Mg996r::I2C_Receive_Data()
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

			// Send an ACK bit to the master device.
			//Send_ACK();
		}

		if (!m_transaction.read && m_address == (m_transaction.address & ~1U)) {
			//m_transaction.state = NState_Machine::ACK_2;
			m_transaction.request_sended = true;
			//m_transaction.data_idx = Data_Length;
		}
		else {
			// Nemel by tady byt, protoze vsichni museji jit do ACK, ale posle ho jen ten, ci adresa to byla
			//m_transaction.state = NState_Machine::Start_Bit;
		}

		// Send an ACK bit to the master device.
		Send_ACK();
		m_transaction.state = NState_Machine::ACK_2;
	}
}

void CPca9685_Mg996r::Send_ACK()
{
	// Do NOT send an ACK bit to the master devices, unless they are talking to us.
	if (m_transaction.address != m_address)
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

void CPca9685_Mg996r::I2C_Receive_ACK_2()
{
	// The slave device is supposed to pull the voltage low (= ACK_1).
	//if (m_gpio->Read_GPIO_Pin(SDA_Pin_Idx) != CGPIO_Manager::CPin::NState::Low)
	//{
		// Report any errors.
		//m_logging_system->Error("Failed to receive ACK_2");
	//}

	--m_transaction.length;
	m_output_fifo.erase(m_output_fifo.begin());
	m_transaction.data = m_output_fifo[0];
	m_transaction.data_idx = Data_Length;
	m_transaction.state = NState_Machine::Send;
}

void CPca9685_Mg996r::I2C_Send_Data()
{
	if (m_transaction.data_idx > 0) {
		if (m_transaction.data_idx % 8 == 0) {
			--m_transaction.data_idx;
			m_transaction.data = m_output_fifo[0];
		}
		else {
			--m_transaction.data_idx;
		}

		// Store the data into the FIFO only if it is meant to be for us.
		if (m_address == (m_transaction.address & ~1U))
		{
			// Get bit that is gonna be sent
			bool bitToSend = (m_transaction.data >> (m_transaction.data_idx % 8)) & 0b1;

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
					//m_transaction.data_idx = Data_Length;
				}
				else {
					m_transaction.request_sended = false;
				}
			}
		}
	}
}

void CPca9685_Mg996r::Render_Pin_Idx() const
{
	ImGui::Text("SDA pin: %d", m_sda_pin_idx);
	ImGui::Text("SCL pin: %d", m_scl_pin_idx);
}

void CPca9685_Mg996r::Render_Motor()
{
	if (ImGui::Checkbox("Motor failure", &m_motor_failure))
	{
		if (m_motor_failure) {
			m_logging_system->Error("Doslo k poruse motoru");
		}
		else {
			m_logging_system->Info("Porucha motoru byla opravena");
		}
	}
}

void CPca9685_Mg996r::Error_Occured()
{
	m_logging_system->Error("Byla detekovana chyba motoru.");
	m_error_probability = 100;
}

void CPca9685_Mg996r::Error_Fixed()
{
	m_logging_system->Error("Chyba motoru byla opravena.");
	m_error_probability = m_error_probability_referenced;
}

void CPca9685_Mg996r::Check_Status()
{
	if (m_error_probability >= 100) {
		m_status = static_cast<std::uint8_t>(Status::Hall_Effect_Error); //1 Oznacuje chybu, ktera se posle pres I2C
	}

	static std::random_device rd;
	static std::mt19937 gen(rd());
	std::uniform_real_distribution<float> dist(0.0f, 100.0f);

	float random_value = dist(gen);

	if (random_value < m_error_probability) {
		m_status = static_cast<std::uint8_t>(Status::Hall_Effect_Error); //1 Oznacuje chybu, ktera se posle pres I2C
	}
}

void CPca9685_Mg996r::Push_Forward(std::uint8_t steps)
{
	if (m_lock_incoming_data)
	{
		if (m_first_part_loeaded) {
			// Unlock incoming data.
			m_lock_incoming_data = false;

			m_steps_to_move = ((uint16_t)m_steps_to_move << 8) | steps;

			m_current_angle += (2 * steps);
			m_current_angle %= 180;

			if (m_patient_idx < 0) {
				m_logging_system->Error("Failed to create patient");
				return;
			}

			Dose_Insulin(m_patient_idx, static_cast<float>(steps * STEPS_TO_UNITS));

			m_first_part_loeaded = false;
			m_steps_to_move = 0;
			return;
		}
		else {
			m_first_part_loeaded = true;
			m_steps_to_move = (uint16_t)steps;
		}
		Check_Status();

		m_logging_system->Info("Posunul jsem motor");
	}

	// Lock incoming data.
	m_lock_incoming_data = true;
}

void CPca9685_Mg996r::Push_Backward(std::uint8_t steps)
{
	if (m_lock_incoming_data)
	{
		if (m_first_part_loeaded) {
			// Unlock incoming data.
			m_lock_incoming_data = false;

			m_steps_to_move = ((uint16_t)m_steps_to_move << 8) | steps;

			m_current_angle -= (2 * m_steps_to_move);
			m_current_angle %= 180;

			m_first_part_loeaded = false;
			m_steps_to_move = 0;
			return;
		}
		else {
			m_first_part_loeaded = true;
			m_steps_to_move = (uint16_t)steps;
		}
		Check_Status();
	}

	// Lock incoming data.
	m_lock_incoming_data = true;
}

void CPca9685_Mg996r::Reset_Position()
{
	m_current_angle = 0;
	return;
}

void CPca9685_Mg996r::Read_Status()
{
	m_output_fifo.clear();
	m_output_fifo.push_back(m_status);
	m_logging_system->Info(("Davam do fifo: " + std::to_string(m_status)).c_str());
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
		*peripheral = new (std::nothrow) CPca9685_Mg996r(name,
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
