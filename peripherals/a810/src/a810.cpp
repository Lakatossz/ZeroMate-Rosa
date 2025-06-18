#include "a810.hpp"

#include <cassert>

CA810::CA810(const std::string& name,
	std::uint32_t pin_idx,
	zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
	zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
	zero_mate::utils::CLogging_System* logging_system)
	: m_name{ name }
	, m_pin_idx{ pin_idx }
	, m_read_pin{ read_pin }
	, m_set_pin{ set_pin }
	, m_time_interval{ 6 }
	, m_last_time{ std::chrono::steady_clock::now() }
	, m_logging_system{ logging_system }
	, m_bubble_found { false }
	, m_output{ true }
	, m_should_stop{ false }
{
	Init_GPIO_Subscription();
	Initialize();
	m_thread = std::thread(&CA810::Control_Loop, this);
}

CA810::~CA810()
{
	if (m_thread.joinable()) {
		m_should_stop = true;
		m_thread.join();
	}
}

nlohmann::json CA810::Parse_JSON_File(const std::string& path)
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

void CA810::Initialize()
{
	nlohmann::json j = Parse_JSON_File(CA810_Config_File);

	if (!j.empty() && j.contains("configuration") &&
		j["configuration"].is_array() &&
		!j["configuration"].empty())
	{
		if (j["configuration"][0].contains("probability") &&
			j["configuration"][0]["probability"].is_array() &&
			!j["configuration"][0]["probability"].empty() &&
			j["configuration"][0]["probability"][0].is_number_unsigned())
		{
			this->m_probability = j["configuration"][0]["probability"][0].get<uint32_t>();
		}
	}
}

void CA810::GPIO_Subscription_Callback(std::uint32_t pin_idx)
{
}

void CA810::Init_GPIO_Subscription()
{
}

void CA810::Set_ImGui_Context(void* context)
{
	// Store the ImGUI Context.
	m_context = static_cast<ImGuiContext*>(context);
}

void CA810::Render()
{
	// Make sure the ImGUIContext has been set.
	assert(m_context != nullptr);
	ImGui::SetCurrentContext(m_context);

	// Render the window.
	if (ImGui::Begin(m_name.c_str()))
	{
		Render_Pins_Idx();
		Render_Sensor();
	}

	ImGui::End();
}

void CA810::Render_Pins_Idx() const
{
	ImGui::Text("GPIO pin: %d", m_pin_idx);
}

void CA810::Render_Sensor()
{
	{
		// The button needs to be pressed down for the output to stay HIGH.
		if (ImGui::Button("Bubble found"))
		{
			if (!m_output && !ImGui::IsItemActive())
			{
				m_output = true;
				m_set_pin(m_pin_idx, !m_output);
			}
		}
		else if (m_output && ImGui::IsItemActive())
		{
			m_logging_system->Error("Bubble found");
			m_output = false;
			m_set_pin(m_pin_idx, !m_output);
		}
	}
}

void CA810::Control_Loop()
{
	while (!m_should_stop.load(std::memory_order_relaxed)) {
		auto now = std::chrono::steady_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - m_last_time).count();

		if (elapsed > m_time_interval) {

			if (m_probability >= 100) {
				m_logging_system->Error("Bubble found");
				m_output = false;
				m_set_pin(m_pin_idx, !m_output);
			}

			static std::random_device rd;
			static std::mt19937 gen(rd());
			std::uniform_real_distribution<float> dist(0.0f, 100.0f);

			float random_value = dist(gen);

			if (random_value < m_probability) {
				m_logging_system->Error("Bubble found");
				m_output = false;
				m_set_pin(m_pin_idx, !m_output);
			}

			m_last_time = std::chrono::steady_clock::now();
		}

		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
}

extern "C"
{
	zero_mate::IExternal_Peripheral::NInit_Status
		Create_Peripheral(zero_mate::IExternal_Peripheral** peripheral,
			const char* const name,
			const std::uint32_t* const connection,
			std::size_t pin_count,
			[[maybe_unused]] zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
			[[maybe_unused]] zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
			[[maybe_unused]] zero_mate::utils::CLogging_System* logging_system)
	{
		// Only one pin shall be passed to the peripheral.
		if (pin_count != 1)
		{
			return zero_mate::IExternal_Peripheral::NInit_Status::GPIO_Mismatch;
		}

		// Create an instance of a Watchdog timer.
		*peripheral = new (std::nothrow) CA810(name,
			connection[0],
			read_pin,
			set_pin,
			logging_system);

		// Make sure the creation was successful.
		if (*peripheral == nullptr)
		{
			return zero_mate::IExternal_Peripheral::NInit_Status::Allocation_Error;
		}

		// All went well.
		return zero_mate::IExternal_Peripheral::NInit_Status::OK;
	}
}
