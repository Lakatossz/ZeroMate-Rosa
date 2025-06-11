#include "hall_effect_sensor.hpp"

CHall_Effect_Sensor::CHall_Effect_Sensor(std::string name,
	std::uint32_t pin_idx,
	zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin)
	: m_name{ std::move(name) }
	, m_pin_idx{ pin_idx }
	, m_set_pin{ set_pin }
	, m_output{ false }
{

}

void CHall_Effect_Sensor::Magnetic_Detected()
{
	m_output = false;
	//m_logging_system->Info("Magentic power detected");
	m_set_pin(m_pin_idx, m_output);
}

extern "C"
{
	zero_mate::IExternal_Peripheral::NInit_Status
		Create_Peripheral(zero_mate::IExternal_Peripheral** peripheral,
			const char* const name,
			const std::uint32_t* const connection,
			std::size_t pin_count,
			zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
			[[maybe_unused]] zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
			[[maybe_unused]] zero_mate::utils::CLogging_System* logging_system)
	{
		// Only one pin shall be passed to the peripheral.
		if (pin_count != 1)
		{
			return zero_mate::IExternal_Peripheral::NInit_Status::GPIO_Mismatch;
		}

		// Create an instance of a DIP switch.
		*peripheral = new (std::nothrow) CHall_Effect_Sensor(name, connection[0], set_pin);

		// Make sure the creation was successful.
		if (*peripheral == nullptr)
		{
			return zero_mate::IExternal_Peripheral::NInit_Status::Allocation_Error;
		}

		// All went well.
		return zero_mate::IExternal_Peripheral::NInit_Status::OK;
	}
}
