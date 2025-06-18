// ----------------------------------------------------------------------------------------------------------------------
/// \file a810.hpp
/// \date 11. 04. 2025
/// \author Jaroslav Rosa (rosajaro1352@gmail.com)
///
/// \brief This file defines an A810 bubble sensor that can be connected to GPIO pin at runtime as a shared library.
///
/// You can find more information about the WDT itself over 
/// at https://www.smdsensors.com/wp-content/uploads/2021/02/A810-bubble-detector-half-inch-tubing-air-in-line-sensor.pdf
// ----------------------------------------------------------------------------------------------------------------------

#pragma once

// STL imports (excluded from Doxygen)
/// \cond
#include <ctime>
#include <array>
#include <queue>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <random>
#include <chrono>
#include <thread>
#include <chrono>
#include <iomanip>
/// \endcond

#include "imgui.h"
#include "nlohmann/json.hpp"

#include "zero_mate/external_peripheral.hpp"

#ifdef ZM_EXTERNAL_PERIPHERAL_EXPORT
#define ZM_EXTERNAL_PERIPHERAL_API __declspec(dllexport)
#else
#define ZM_EXTERNAL_PERIPHERAL_API __declspec(dllimport)
#endif

#define TIME_OUT 10

// ---------------------------------------------------------------------------------------------------------------------
/// \class CA810
/// \brief This class represents an A810.
// ---------------------------------------------------------------------------------------------------------------------
class ZM_EXTERNAL_PERIPHERAL_API CA810 final : public zero_mate::IExternal_Peripheral
{
public:

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Creates an instance of the class.
	/// \param name Unique name of the device
	/// \param pin_idx Pin index
	/// \param read_pin Function the peripheral uses to read GPIO pin
	/// \param set_pin Function the peripheral uses to set GPIO pin
	/// \param logging_system Logging system
	// -----------------------------------------------------------------------------------------------------------------
	explicit CA810(const std::string& name,
		std::uint32_t pin_idx,
		zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
		zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
		zero_mate::utils::CLogging_System* logging_system);

	~CA810();

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Callback function that notifies the peripheral about a change of one of the pins it subscribes to.
	/// \param pin_idx Index of the GPIO pin whose state has been changed
	// -----------------------------------------------------------------------------------------------------------------
	void GPIO_Subscription_Callback(std::uint32_t pin_idx) override;

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Renders the A810 sensor (GUI).
	// -----------------------------------------------------------------------------------------------------------------
	void Render() override;

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Sets an ImGuiContext, so the sensor can render itself as a GUI window.
	/// \param context ImGuiContext the sensor uses to render itself
	// -----------------------------------------------------------------------------------------------------------------
	void Set_ImGui_Context(void* context) override;

private:

	/// Path to the external peripherals config file.
	const char* const CA810_Config_File = "peripherals/a810_config.json";

	[[nodiscard]] inline nlohmann::json Parse_JSON_File(const std::string& path);

	inline void Initialize();

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Initializes the subscription (list of GPIO pins the peripheral wants to listen to).
	// -----------------------------------------------------------------------------------------------------------------
	inline void Init_GPIO_Subscription();

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Controls time interval of sensor.
	// -----------------------------------------------------------------------------------------------------------------
	void Control_Loop();

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Renders the number of GPIO pins the sensor is connected to.
	// -----------------------------------------------------------------------------------------------------------------
	inline void Render_Pins_Idx() const;

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Renders the sensor itself.
	// -----------------------------------------------------------------------------------------------------------------
	inline void Render_Sensor();

	std::string m_name;											///< Unique name of the peripheral
	std::uint32_t m_pin_idx;									///< GPIO pin
	zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t m_read_pin;///< Function used to read the
	zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t m_set_pin;	///< Function used to read the
	bool m_pin_high;                                            ///< Current state of the pin sensor
	std::chrono::steady_clock::time_point m_last_time;			///< Last time the sensor looked for bubble
	std::uint32_t m_probability;								///< Probability that bubble is found
	std::uint32_t m_time_interval;								///< Time after which sensor checks bubble

	zero_mate::utils::CLogging_System* m_logging_system;        ///< Logging system

	std::thread m_thread;
	std::atomic<bool> m_should_stop;

	ImGuiContext* m_context;									///< ImGUI context (rendering the GUI)
	bool m_output;												///< Current output from the WDT

	bool m_bubble_found;												///< State when WDT barks at MCU
};
