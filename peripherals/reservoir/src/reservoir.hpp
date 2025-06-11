// ---------------------------------------------------------------------------------------------------------------------
/// \file reservoir.hpp
/// \date 19. 03. 2023
/// \author Jaroslav Rosa (rosajaro1352@gmail.com)
///
/// \brief 
// ---------------------------------------------------------------------------------------------------------------------

#pragma once

#include <iostream>

#include "imgui.h"

#include "zero_mate/external_peripheral.hpp"

#define PUSH_OUT_DOSE 0.5f	///< Number of units in one push

// ---------------------------------------------------------------------------------------------------------------------
/// \class CReservoir
/// \brief This class presents an Reservoir (external peripheral).
// ---------------------------------------------------------------------------------------------------------------------
class CReservoir final : public zero_mate::IExternal_Peripheral
{
public:

	// -----------------------------------------------------------------------------------------------------------------
   /// \brief Creates an instance of the class.
   /// \param name Unique name of the peripheral (e.g. My_LED_1)
   /// \param pin_idx GPIO pin the LED is connected to
   /// \param read_pin Function provided by the emulator that is used to read the state of a GPIO pin
   // -----------------------------------------------------------------------------------------------------------------
	explicit CReservoir(const std::string& name,
						std::uint32_t pin_idx,
		zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
		zero_mate::utils::CLogging_System* logging_system);

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Renders the DIP switch (GUI).
	// -----------------------------------------------------------------------------------------------------------------
	void Render() override;

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Sets an ImGuiContext, so the DIP switch can render itself as a GUI window.
	/// \param context ImGuiContext the DIP switch uses to render itself
	// -----------------------------------------------------------------------------------------------------------------
	void Set_ImGui_Context(void* context) override;

private:

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Renders the DIP switch itself.
	// -----------------------------------------------------------------------------------------------------------------
	inline void Render_DIP_Switch();

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Renders the number of the GPIO pin the DIP switch is connected to.
	// -----------------------------------------------------------------------------------------------------------------
	inline void Render_Pin_Idx() const;

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief 
	// -----------------------------------------------------------------------------------------------------------------
	void Push_Insulin_Out();

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief 
	// -----------------------------------------------------------------------------------------------------------------
	float Change_Temperature();

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief 
	// -----------------------------------------------------------------------------------------------------------------
	float Change_Pressure();

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief 
	// -----------------------------------------------------------------------------------------------------------------
	bool Is_Reservoir_Empty() const;

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief 
	// -----------------------------------------------------------------------------------------------------------------
	bool Is_Occluded() const;

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief 
	// -----------------------------------------------------------------------------------------------------------------
	bool Has_Air_Bubble();

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief 
	// -----------------------------------------------------------------------------------------------------------------
	void Notify_Empty();

	std::string m_name;											///< Unique name of the peripheral
	std::uint32_t m_pin_idx;                                    ///< GPIO pin the LED is hooked up to
	zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t m_set_pin;	///< Function used to read the
	const float volume;											///< 
	float current_volume;										///< 
	const float referenced_pressure;							///< 
	float current_pressure;										///< 
	const float referenced_temperature;							///< 
	float current_temperature;									///< 

	zero_mate::utils::CLogging_System* m_logging_system;        ///< Logging system

	ImGuiContext* m_context;									///< ImGUI context (rendering the GUI)
	bool m_output;												///< Current output from the DIP switch

	bool m_is_reservoir_empty;									///< Empty reservoir variable
};
