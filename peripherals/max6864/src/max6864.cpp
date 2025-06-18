#include "max6864.hpp"

#include <cassert>

CMax6864::CMax6864(const std::string& name,
	std::uint32_t pin_reset_idx,
	std::uint32_t pin_wdi_idx,
	std::uint32_t pin_mr_idx,
	zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
	zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
	zero_mate::utils::CLogging_System* logging_system)
	: m_name{ name }
	, m_pin_reset_idx{ pin_reset_idx }
	, m_pin_wdi_idx{ pin_wdi_idx }
	, m_pin_mr_idx{ pin_mr_idx }
	, m_read_pin{ read_pin }
	, m_set_pin{ set_pin }
	, m_time_interval{ 60 }
	, m_logging_system{ logging_system }
	, m_bark{ false }
	, m_output{ true }
	, m_should_stop{ false }
	, m_wdt_thread{ }
{
	Init_GPIO_Subscription();
	// Initialize logic of the WDT
	Initialize();
	// Start control loop of WDT in separated thread
	m_wdt_thread = std::thread(&CMax6864::Control_Loop, this);
}

CMax6864::~CMax6864()
{
	// Stop separeted therad
	if (m_wdt_thread.joinable()) {
		m_should_stop = true;
		m_wdt_thread.join();
	}
}

nlohmann::json CMax6864::Parse_JSON_File(const std::string& path)
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

void CMax6864::Initialize()
{
	nlohmann::json j = Parse_JSON_File(Max6864_Config_File);

	if (!j.empty() && j.contains("configuration") &&
		j["configuration"].is_array() &&
		!j["configuration"].empty())
	{
		if (j["configuration"][0].contains("time_interval") &&
			j["configuration"][0]["time_interval"].is_array() &&
			!j["configuration"][0]["time_interval"].empty() &&
			j["configuration"][0]["time_interval"][0].is_number_unsigned())
		{
			this->m_time_interval = j["configuration"][0]["time_interval"][0].get<uint32_t>();
		}
	}

	last_kick.store(std::chrono::steady_clock::now(), std::memory_order_release);
}

void CMax6864::GPIO_Subscription_Callback(std::uint32_t pin_idx)
{
	// Initialize listening for kicks
	if (pin_idx == m_pin_wdi_idx) {
		m_pin_high = m_read_pin(pin_idx);
		// If kicked, update time of last_kick timestamp
		last_kick.store(std::chrono::steady_clock::now(), std::memory_order_release);
	}
}

void CMax6864::Init_GPIO_Subscription()
{
	m_gpio_subscription.insert(m_pin_wdi_idx);
	m_gpio_subscription.insert(m_pin_mr_idx);
}

void CMax6864::Set_ImGui_Context(void* context)
{
	// Store the ImGUI Context.
	m_context = static_cast<ImGuiContext*>(context);
}

void CMax6864::Render()
{
	// Make sure the ImGUIContext has been set.
	assert(m_context != nullptr);
	ImGui::SetCurrentContext(m_context);

	// Render the window.
	if (ImGui::Begin(m_name.c_str()))
	{
		Render_Pins_Idx();
		Render_WDT();
	}

	ImGui::End();
}

void CMax6864::Render_Pins_Idx() const
{
	ImGui::Text("GPIO reset-pin: %d", m_pin_reset_idx);
	ImGui::Text("GPIO wdi-pin: %d", m_pin_wdi_idx);
	ImGui::Text("GPIO mr-pin: %d", m_pin_mr_idx);
}

void CMax6864::Render_WDT()
{
	{
		// The button needs to be pressed down for the output to stay HIGH.
		if (ImGui::Button("Manual reset"))
		{
			if (!m_output && !ImGui::IsItemActive())
			{
				m_output = true;
				m_set_pin(m_pin_reset_idx, !m_output);
			}
		}
		else if (m_output && ImGui::IsItemActive())
		{
			Bark();
		}
	}
}

void CMax6864::Bark()
{
	// Log message
	m_logging_system->Error(TIME_OUT_MESSAGE);
	
	// Set pulse on reset pin
	m_set_pin(m_pin_reset_idx, m_output);
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	m_set_pin(m_pin_reset_idx, !m_output);

	// Update time of last_kick timestamp
	last_kick.store(std::chrono::steady_clock::now(), std::memory_order_release);
}

void CMax6864::Control_Loop()
{
	// In loop check whether the WDT the interval is comming to an end
	while (!m_should_stop.load(std::memory_order_acquire)) {
		auto now = std::chrono::steady_clock::now();
		auto last_kick_time = last_kick.load(std::memory_order_acquire);
		auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_kick_time);

		// If the interval passed, bark
		if (elapsed.count() > m_time_interval) {
			Bark();
			break;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
		if (pin_count != 3)
		{
			return zero_mate::IExternal_Peripheral::NInit_Status::GPIO_Mismatch;
		}

		// Create an instance of a Watchdog timer.
		*peripheral = new (std::nothrow) CMax6864(name,
												connection[0], 
												connection[1], 
												connection[2], 
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
