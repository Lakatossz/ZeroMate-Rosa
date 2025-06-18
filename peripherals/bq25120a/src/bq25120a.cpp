#include "bq25120a.hpp"

#include <cassert>

CBq25120a::CBq25120a(const std::string& name,
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
	, m_logging_system{ logging_system }
	, m_sda_rising_edge_timestamp{ 0 }
	, m_scl_rising_edge_timestamp{ 0 }
	, m_sda_prev_state{ false }
	, m_scl_prev_state{ false }
	, m_sda_rising_edge{ false }
	, m_scl_rising_edge{ false }
	, m_processing_cmd{ true }
	, m_lock_incoming_data{ false }
	, m_full_capacity{ 0 }
	, m_current_capacity{ 0 }
	, m_capacity_downfall{ 0 }
	, m_referenced_temperature { 0 }
	, m_current_temperature{ 0 }
	, m_higher_temperature_value { 0 }
	, m_lower_temperature_value { 0 }
	, m_higher_temperature{ false }
	, m_lower_temperature{ false }
	, m_referenced_charge{ 0 }
	, m_current_charge{ 0 }
	, m_higher_charge_value{ 0 }
	, m_lower_charge_value{ 0 }
	, m_higher_charge{ false }
	, m_lower_charge{ false }
	, m_referenced_voltage{ 0 }
	, m_current_voltage{ 0 }
	, m_higher_voltage_value{ 0 }
	, m_lower_voltage_value{ 0 }
	, m_higher_voltage{ false }
	, m_lower_voltage{ false }
	, m_referenced_current{ 0 }
	, m_current_current{ 0 }
	, m_higher_current_value{ 0 }
	, m_lower_current_value{ 0 }
	, m_higher_current{ false }
	, m_lower_current{ false }
	, m_error_occured{ false }
{
	Init_GPIO_Subscription();
	Initialize();
	std::thread bq25120a_thread(&CBq25120a::Control_Loop, this);
	bq25120a_thread.detach();
}

CBq25120a::~CBq25120a()
{

}

nlohmann::json CBq25120a::Parse_JSON_File(const std::string& path)
{
	// Open the config file in a read-only mode.
	std::ifstream config_file{ path, std::fstream::in };

	// Make sure the file has been opened successfully.
	if (!config_file)
	{
		m_logging_system->Error(("Cannot load configuraiton " + path).c_str());

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

void CBq25120a::Set_Parameter(nlohmann::json j, std::string name, float& variable) {
	if (j["configuration"][0].contains(name) &&
		j["configuration"][0][name].is_array() &&
		!j["configuration"][0][name].empty())
	{
		variable = j["configuration"][0][name][0].get<float>();
	}
}

void CBq25120a::Set_Parameter_And_Referenced(nlohmann::json j, std::string name, float& referenced, float& variable) {
	if (j["configuration"][0].contains(name) &&
		j["configuration"][0][name].is_array() &&
		!j["configuration"][0][name].empty())
	{
		referenced = j["configuration"][0][name][0].get<float>();
		variable = referenced;
	}
}

void CBq25120a::Initialize()
{
	nlohmann::json j = Parse_JSON_File(Bq25120a_Config_File);

	if (!j.empty() && j.contains("configuration") &&
		j["configuration"].is_array() &&
		!j["configuration"].empty())
	{
		Set_Parameter(j, "full_capacity", this->m_full_capacity);
		Set_Parameter(j, "capacity_downfall", this->m_capacity_downfall);
		Set_Parameter_And_Referenced(j, "referenced_temperature", this->m_referenced_temperature, this->m_current_temperature);
		Set_Parameter(j, "higher_temperature", this->m_higher_temperature_value);
		Set_Parameter(j, "lower_temperature", this->m_lower_temperature_value);
		Set_Parameter_And_Referenced(j, "referenced_charge", this->m_referenced_charge, this->m_current_charge);
		Set_Parameter(j, "higher_charge", this->m_higher_charge_value);
		Set_Parameter(j, "lower_charge", this->m_lower_charge_value);
		Set_Parameter_And_Referenced(j, "referenced_voltage", this->m_referenced_voltage, this->m_current_voltage);
		Set_Parameter(j, "higher_voltage", this->m_higher_voltage_value);
		Set_Parameter(j, "lower_voltage", this->m_lower_voltage_value);
		Set_Parameter_And_Referenced(j, "referenced_current", this->m_referenced_current, this->m_current_current);
		Set_Parameter(j, "higher_current", this->m_higher_current_value);
		Set_Parameter(j, "lower_current", this->m_lower_current_value);

		if (j["configuration"][0].contains("temperature_start_with") &&
			j["configuration"][0]["temperature_start_with"].is_array() &&
			!j["configuration"][0]["temperature_start_with"].empty() &&
			j["configuration"][0]["temperature_start_with"][0].is_number_unsigned())
		{
			std::uint32_t start_with = j["configuration"][0]["temperature_start_with"][0].get<std::uint32_t>();

			if (start_with == 0) {
				this->m_higher_temperature = false;
				this->m_lower_temperature = false;
			}
			else if (start_with == 1) {
				this->m_higher_temperature = true;
				this->m_lower_temperature = !this->m_higher_temperature;
			}
			else if (start_with == 2) {
				this->m_lower_temperature = true;
				this->m_higher_temperature = !this->m_lower_temperature;
			}
		}

		if (j["configuration"][0].contains("charge_start_with") &&
			j["configuration"][0]["charge_start_with"].is_array() &&
			!j["configuration"][0]["charge_start_with"].empty() &&
			j["configuration"][0]["charge_start_with"][0].is_number_unsigned())
		{
			std::uint32_t start_with = j["configuration"][0]["charge_start_with"][0].get<std::uint32_t>();

			if (start_with == 0) {
				this->m_higher_charge = false;
				this->m_lower_charge = false;
			}
			else if (start_with == 1) {
				this->m_higher_charge = true;
				this->m_lower_charge = !this->m_higher_charge;
			}
			else if (start_with == 2) {
				this->m_lower_charge = true;
				this->m_higher_charge = !this->m_lower_charge;
			}
		}

		if (j["configuration"][0].contains("voltage_start_with") &&
			j["configuration"][0]["voltage_start_with"].is_array() &&
			!j["configuration"][0]["voltage_start_with"].empty() &&
			j["configuration"][0]["voltage_start_with"][0].is_number_unsigned())
		{
			std::uint32_t start_with = j["configuration"][0]["voltage_start_with"][0].get<std::uint32_t>();

			if (start_with == 0) {
				this->m_higher_voltage = false;
				this->m_lower_voltage = false;
			}
			else if (start_with == 1) {
				this->m_higher_voltage = true;
				this->m_lower_voltage = !this->m_higher_voltage;
			}
			else if (start_with == 2) {
				this->m_lower_voltage = true;
				this->m_higher_voltage = !this->m_lower_voltage;
			}
		}

		if (j["configuration"][0].contains("current_start_with") &&
			j["configuration"][0]["current_start_with"].is_array() &&
			!j["configuration"][0]["current_start_with"].empty() &&
			j["configuration"][0]["current_start_with"][0].is_number_unsigned())
		{
			std::uint32_t start_with = j["configuration"][0]["current_start_with"][0].get<std::uint32_t>();

			if (start_with == 0) {
				this->m_higher_current = false;
				this->m_lower_current = false;
			}
			else if (start_with == 1) {
				this->m_higher_current = true;
				this->m_lower_current = !this->m_higher_current;
			}
			else if (start_with == 2) {
				this->m_lower_current = true;
				this->m_higher_current = !this->m_lower_current;
			}
		}

		this->m_current_capacity = this->m_full_capacity * (this->m_referenced_charge / 100);
	}
}

void CBq25120a::GPIO_Subscription_Callback([[maybe_unused]] std::uint32_t pin_idx)
{
	// Increment the clock and read the current state of the pin.
	++m_clock;
	const bool curr_pin_state = m_read_pin(pin_idx);

	// Call the corresponding cI2C_Send_Dataallback function.
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
		(m_transaction.state == NState_Machine::Recieve && (m_transaction.data_idx == 0 || m_transaction.data_idx == 7) ||
			(m_transaction.state == NState_Machine::Send &&
				m_transaction.length == 0 &&
				m_transaction.data_idx == 0));

	// If a stop bit has just been detected, terminate the current transaction.
	if (stop_bit_detected)
	{
		m_transaction.state = NState_Machine::Start_Bit;
		Received_Transaction_Callback();
	}
}

void CBq25120a::Init_GPIO_Subscription()
{
	m_gpio_subscription.insert(m_sda_pin_idx);
	m_gpio_subscription.insert(m_scl_pin_idx);
}

void CBq25120a::I2C_Update()
{
	// I2C state machine
	switch (m_transaction.state)
	{
		// Receive the start bit.
	case NState_Machine::Start_Bit:
		I2C_Receive_Start_Bit();
		break;

		// Receive the slave's address.
	case NState_Machine::Address:
		I2C_Receive_Address();
		break;

		// Receive the RW bit.
	case NState_Machine::RW:
		I2C_Receive_RW_Bit();
		break;

		// Send the ACK_1 bit.
	case NState_Machine::ACK_1:
		//Send_ACK();
		m_transaction.state = NState_Machine::Recieve;
		break;

	case NState_Machine::ACK_1_Send:
		m_transaction.state = NState_Machine::Send;
		break;

	case NState_Machine::Recieve:
		I2C_Receive_Data();
		break;

		// Receive data (payload).
	case NState_Machine::Send:
		I2C_Send_Data();
		break;

	case NState_Machine::Receive_ACK:
		I2C_Receive_ACK_2();
		break;

		// Send the ACK_2 bit.
	case NState_Machine::ACK_2:
		// Move on to receiving another byte.
		m_transaction.data = 0;
		m_transaction.data_idx = Data_Length;
		m_transaction.state = NState_Machine::Recieve;

		// Send an ACK bit to the master device.
		Send_ACK();
		break;
	}
}

void CBq25120a::I2C_Receive_Start_Bit()
{
	if (!m_read_pin(m_sda_pin_idx))
	{
		Init_Transaction();
		m_transaction.state = NState_Machine::Address;
	}
}

void CBq25120a::I2C_Receive_Address()
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
		// Move on to receiving the RW bit.
		m_transaction.state = NState_Machine::RW;
	}
}

void CBq25120a::I2C_Receive_RW_Bit()
{
	// Read the RW bit.
	m_transaction.read = m_read_pin(m_sda_pin_idx);

	std::cout << "Prisla mi adresa: " << static_cast<std::uint32_t>(m_transaction.address) << std::endl;

	if (((m_transaction.address & 0x01) != 0) && m_address == (m_transaction.address & ~1U) && m_transaction.request_sended) {
		m_transaction.data = m_output_fifo[0];
		m_transaction.data_idx = Data_Length;
		m_transaction.length = sizeof(float);
		m_transaction.state = NState_Machine::Send;
		Send_ACK();
	}
	else if (((m_transaction.address & 0x01) != 0) && m_address != (m_transaction.address & ~1U)) {
		m_transaction.state = NState_Machine::Recieve;
		Send_ACK();
	}
	else {
		Send_ACK();
		m_transaction.state = NState_Machine::ACK_1;
	}
}

void CBq25120a::I2C_Receive_Data()
{
	--m_transaction.data_idx;

	// Read the state of the SDA pin and update the data (receiving byte).
	if (m_read_pin(m_sda_pin_idx))
	{
		m_transaction.data |= (0b1U << m_transaction.data_idx);
	}

	// Have we read all 8 bits of the data yet?
	if (m_transaction.data_idx == 0)
	{
		// Store the data into the FIFO only if it is meant to be for us.
		if (m_address == (m_transaction.address & ~1U))
		{
			m_fifo.push_back(m_transaction.data);
		}

		if (!m_transaction.read && m_address == (m_transaction.address & ~1U)) {
			m_transaction.request_sended = true;
		}

		m_transaction.state = NState_Machine::ACK_2;
	}
}

void CBq25120a::I2C_Send_Data()
{
	if (m_transaction.data_idx > 0) {
		--m_transaction.data_idx;
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
				}
				else {
					m_transaction.request_sended = false;
					m_clock++;
					SDA_Pin_Change_Callback(false);
				}
			}
		}
	}
}

void CBq25120a::SCL_Pin_Change_Callback(bool curr_pin_state)
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

void CBq25120a::SDA_Pin_Change_Callback(bool curr_pin_state)
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

void CBq25120a::Updated_Type_Of_Processing_Data(std::uint8_t data)
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

void CBq25120a::Log_Received_Data()
{
	std::stringstream ss{};

	ss << "Received data: " << m_fifo.size() << " ";

	for (const auto& value : m_fifo)
	{
		ss << static_cast<std::uint32_t>(value) << " ";
	}

	m_logging_system->Debug(ss.str().c_str());
}

void CBq25120a::Received_Transaction_Callback()
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

void CBq25120a::Init_Transaction()
{
	m_fifo.clear();

	m_transaction.address = 0x0;
	m_transaction.length = 0;
	m_transaction.data = 0;
	m_transaction.data_idx = Data_Length;
	m_transaction.addr_idx = Slave_Addr_Length;
	m_transaction.read = false;
}

void CBq25120a::Send_ACK()
{
	// Do NOT send an ACK bit to the master devices, unless they are talking to us.
	//if ((m_transaction.address & ~1U) != m_address)
	//{
	//	return;
	//}

	// Send an ACK bit.
	const int status = m_set_pin(m_sda_pin_idx, false);

	m_clock++;
	SDA_Pin_Change_Callback(false);

	// Check for any possible errors.
	if (status != 0)
	{
		m_logging_system->Error("Failed to set the value of the SDA pin (ACK)");
	}
}

void CBq25120a::I2C_Receive_ACK_2()
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

void CBq25120a::Process_Data(std::uint8_t data)
{
	if (m_fifo.size() > 2) {
		uint16_t combined = (static_cast<uint16_t>(m_fifo[1]) << 8) | m_fifo[2];
	}

	// Go through individual bits of the received byte of data.
	for (std::int8_t i = 0U; i < std::numeric_limits<std::uint8_t>::digits; ++i)
	{
		// TODO
	}
}

void CBq25120a::Process_CMD(std::uint8_t data)
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
	case NCMD::Read_Charge:
		m_logging_system->Info("Ctu stav nabiti baterie");
		Read_Charge();
		break;
	case NCMD::Read_Voltage:
		m_logging_system->Info("Ctu hodnotu napeti baterie");
		Read_Voltage();
		break;
	case NCMD::Read_Current:
		m_logging_system->Info("Ctu hodnotu proudu baterie");
		Read_Current();
		break;
	case NCMD::Read_Temperature:
		m_logging_system->Info("Ctu teplotu baterie");
		Read_Temperature();
		break;
	case NCMD::Read_Errors:
		Read_Errors();
		break;
	case NCMD::Read_Info:
		Read_Info();
		break;
	case NCMD::Write_Reset:
		Write_Reset();
		break;
	case NCMD::Write_Init:
		Write_Init();
		break;
	case NCMD::Write_Safe:
		Write_Safe();
		break;
	case NCMD::Command_Start:
		[[fallthrough]];
	case NCMD::Data_Start:
	case NCMD::Data_Continue:
		break;
	}
}

void CBq25120a::Set_ImGui_Context(void* context)
{
	// Store the ImGUI Context.
	m_context = static_cast<ImGuiContext*>(context);
}

void CBq25120a::Render()
{
	// Make sure the ImGUIContext has been set.
	assert(m_context != nullptr);
	ImGui::SetCurrentContext(m_context);

	// Render the window.
	if (ImGui::Begin(m_name.c_str()))
	{
		Render_Information();
		Render_Pins();
		Render_Battery();
	}

	ImGui::End();
}

void CBq25120a::Render_Information() const
{
	ImGui::Text("BQ25120a");
	ImGui::Text("I2C addr = 0x%X (%d dec)", m_address, m_address);
	ImGui::Text("Full capacity: %f mAh", m_full_capacity);
	ImGui::Text("Current charge: %f %%", m_current_charge);
	ImGui::Text("Current voltage: %f V", m_current_voltage);
	ImGui::Text("Current current: %f A", m_current_current);
	ImGui::Text("Current temperature: %f C", m_current_temperature);
}

void CBq25120a::Render_Pins() const
{
	ImGui::Text("SDA pin: %d", m_sda_pin_idx);
	ImGui::Text("SCL pin: %d", m_scl_pin_idx);
}

void CBq25120a::Render_Battery()
{
	if (ImGui::Checkbox("Battery error", &m_error_occured))
	{
		if (m_error_occured) {
			Notify_Error();
		}
	}

	if (!m_lower_temperature && ImGui::Checkbox("Higher temperature", &m_higher_temperature))
	{
		if (m_higher_temperature == true) {
			m_logging_system->Error("Byla detekovana vyssi teplota");
		}
		Read_Temperature();
	}
	else if (!m_higher_temperature && ImGui::Checkbox("Lower temperature", &m_lower_temperature))
	{
		if (m_lower_temperature == true) {
			m_logging_system->Error("Byla detekovana nizsi teplota");
		}
		Read_Temperature();
	}

	if (!m_lower_charge && ImGui::Checkbox("Higher charge", &m_higher_charge))
	{
		if (m_higher_charge == true) {
			m_logging_system->Error("Byla detekovana vyssi uroven nabiti");
		}
		Read_Charge();
	}
	else if (!m_higher_charge && ImGui::Checkbox("Lower charge", &m_lower_charge))
	{
		if (m_lower_charge == true) {
			m_logging_system->Error("Byla detekovana nizsi uroven nabiti");
		}
		Read_Charge();
	}

	if (!m_lower_voltage && ImGui::Checkbox("Higher voltage", &m_higher_voltage))
	{
		if (m_higher_voltage == true) {
			m_logging_system->Error("Bylo detekovano vyssi napeti");
		}
		Read_Voltage();
	}
	else if (!m_higher_voltage && ImGui::Checkbox("Lower voltage", &m_lower_voltage))
	{
		if (m_lower_voltage == true) {
			m_logging_system->Error("Bylo detekovano nizsi napeti");
		}
		Read_Voltage();
	}

	if (!m_lower_current && ImGui::Checkbox("Higher current", &m_higher_current))
	{
		if (m_higher_current == true) {
			m_logging_system->Error("Byl detekovan vyssi proud");
		}
		Read_Current();
	}
	else if (!m_higher_current && ImGui::Checkbox("Lower current", &m_lower_current))
	{
		if (m_lower_current == true) {
			m_logging_system->Error("Byl detekovan nizsi tlak");
		}
		Read_Current();
	}
}

void CBq25120a::Notify_Error()
{
	m_logging_system->Error("Byla detekovana chyba baterie");
}

void CBq25120a::Read_Temperature()
{
	if (m_higher_temperature) {
		m_current_temperature = this->m_higher_temperature_value;
	}
	else if (m_lower_temperature) {
		m_current_temperature = this->m_lower_temperature_value;
	}
	else {
		m_current_temperature = this->m_referenced_temperature;
	}

	byte bytes[sizeof(float)];
	memcpy(bytes, &m_current_temperature, sizeof(float));

	m_output_fifo.clear();
	for (int i = 0; i < sizeof(float); i++) {
		m_output_fifo.push_back(bytes[i]);
	}
}

void CBq25120a::Read_Charge()
{
	if (m_higher_charge) {
		m_current_charge = this->m_higher_charge_value;
	}
	else if (m_lower_charge) {
		m_current_charge = this->m_lower_charge_value;
	}
	else {
		m_current_charge = this->m_referenced_charge;
	}

	byte bytes[sizeof(float)];
	memcpy(bytes, &m_current_charge, sizeof(float));

	m_output_fifo.clear();
	for (int i = 0; i < sizeof(float); i++) {
		m_output_fifo.push_back(bytes[i]);
	}
}

void CBq25120a::Read_Voltage()
{
	if (m_higher_voltage) {
		m_current_voltage = this->m_higher_voltage_value;
	}
	else if (m_lower_voltage) {
		m_current_voltage = this->m_lower_voltage_value;
	}
	else {
		m_current_voltage = this->m_referenced_voltage;
	}

	byte bytes[sizeof(float)];
	memcpy(bytes, &m_current_voltage, sizeof(float));

	m_output_fifo.clear();
	for (int i = 0; i < sizeof(float); i++) {
		m_output_fifo.push_back(bytes[i]);
	}
}

void CBq25120a::Read_Current()
{
	if (m_higher_current) {
		m_current_current = this->m_higher_current_value;
	}
	else if (m_lower_current) {
		m_current_current = this->m_lower_current_value;
	}
	else {
		m_current_current = this->m_referenced_current;
	}

	byte bytes[sizeof(float)];
	memcpy(bytes, &m_current_current, sizeof(float));

	m_output_fifo.clear();
	for (int i = 0; i < sizeof(float); i++) {
		m_output_fifo.push_back(bytes[i]);
	}
}

void CBq25120a::Read_Errors()
{

}

void CBq25120a::Read_Info()
{

}

void CBq25120a::Write_Reset()
{

}

void CBq25120a::Write_Init()
{

}

void CBq25120a::Write_Safe()
{

}

void CBq25120a::Control_Loop()
{
	while (true) {
		std::this_thread::sleep_for(std::chrono::seconds(charge_cycle_time));

		if ((m_current_capacity - m_capacity_downfall) > 0) {
			m_current_capacity -= m_capacity_downfall;
			m_current_charge = 100 * m_current_capacity / m_full_capacity;
		}
		else {
			m_error_occured = true;
		}
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

		// Create an instance of an battery system.
		// clang-format off
		*peripheral = new (std::nothrow) CBq25120a(name,
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
