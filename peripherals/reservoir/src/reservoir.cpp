#include "reservoir.hpp"

#include <cassert>

CReservoir::CReservoir(
	const std::string& name, 
	std::uint32_t pin_idx, 
	zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
	zero_mate::utils::CLogging_System* logging_system)
	: m_name{ name }
	, m_pin_idx{ pin_idx }
	, m_set_pin{ set_pin }
	, volume{ 300 }
	, current_volume{ 300 }
	, referenced_pressure{ 5.5f }
	, current_pressure{ 5.5f }
	, referenced_temperature{ 20.1f }
	, current_temperature{ 20.1f }
	, m_logging_system{ logging_system }
{
	
}

void CReservoir::Set_ImGui_Context(void* context)
{
	// Store the ImGUI Context.
	m_context = static_cast<ImGuiContext*>(context);
}

void CReservoir::Render()
{
	// Make sure the ImGUIContext has been set.
	assert(m_context != nullptr);
	ImGui::SetCurrentContext(m_context);

	// Render the window.
	if (ImGui::Begin(m_name.c_str()))
	{
		Render_Pin_Idx();
		Render_DIP_Switch();
	}

	ImGui::End();
}

void CReservoir::Render_Pin_Idx() const
{
	ImGui::Text("GPIO pin: %d", m_pin_idx);
}

void CReservoir::Render_DIP_Switch()
{
	if (ImGui::Checkbox("Empty reservoir", &m_is_reservoir_empty))
	{
		if (m_is_reservoir_empty) {
			Notify_Empty();
		}
		else {
			m_logging_system->Info("Nadrzka inzulinu byla doplnena.");
		}
	}
}

void CReservoir::Push_Insulin_Out()
{
	if (Has_Air_Bubble()) {
		std::cerr << "Warning: Air bubble detected! Insulin delivery interrupted." << std::endl;
		return;
	}

	if (Is_Reservoir_Empty()) {
		std::cerr << "Error: Reservoir is empty!" << std::endl;
		return;
	}

	if (Is_Occluded()) {
		std::cerr << "Error: Occlusion detected! Insulin flow blocked." << std::endl;
		return;
	}

	current_volume -= PUSH_OUT_DOSE;

	current_volume -= PUSH_OUT_DOSE;
	current_pressure += Change_Temperature();
	current_temperature += Change_Pressure();
}

float CReservoir::Change_Temperature()
{
	return current_pressure;
}

float CReservoir::Change_Pressure()
{
	return current_temperature;
}

bool CReservoir::Is_Reservoir_Empty() const {
	return current_volume < PUSH_OUT_DOSE;
}

bool CReservoir::Is_Occluded() const {
	return current_pressure > referenced_pressure;
}

bool CReservoir::Has_Air_Bubble() {
	float probability = static_cast<float>(rand()) / RAND_MAX;
	return probability < 0.05f; // 5% chance of air bubble
}

void CReservoir::Notify_Empty() {
	m_logging_system->Error("Je prazdna nadrzka na inzulin.");
	m_output = true;
	m_set_pin(m_pin_idx, m_output);
}

extern "C"
{
	zero_mate::IExternal_Peripheral::NInit_Status
		Create_Peripheral(zero_mate::IExternal_Peripheral** peripheral,
			const char* const name,
			const std::uint32_t* const connection,
			std::size_t pin_count,
			[[maybe_unused]] zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
			zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
			[[maybe_unused]] zero_mate::utils::CLogging_System* logging_system)
	{
		// Only one pin shall be passed to the peripheral.
		if (pin_count != 1)
		{
			return zero_mate::IExternal_Peripheral::NInit_Status::GPIO_Mismatch;
		}

		// Create an instance of a Reservoir .
		*peripheral = new (std::nothrow) CReservoir(name, connection[0], set_pin, logging_system);

		// Make sure the creation was successful.
		if (*peripheral == nullptr)
		{
			return zero_mate::IExternal_Peripheral::NInit_Status::Allocation_Error;
		}

		// All went well.
		return zero_mate::IExternal_Peripheral::NInit_Status::OK;
	}
}
