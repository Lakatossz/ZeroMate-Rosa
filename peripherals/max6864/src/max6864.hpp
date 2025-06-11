// ----------------------------------------------------------------------------------------------------------------------
/// \file max6864.hpp
/// \date 11. 04. 2025
/// \author Jaroslav Rosa (rosajaro1352@gmail.com)
///
/// \brief This file defines a MAX6864 WDT that can be connected to GPIO pin at runtime as a shared library.
///
/// You can find more information about the WDT itself over 
/// at https://www.analog.com/media/en/technical-documentation/data-sheets/max6854-max6869.pdf
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

#define TIME_OUT_MESSAGE "MCU neodpovedel vcas, restratuji ho"
#define TIME_OUT 10

// ---------------------------------------------------------------------------------------------------------------------
/// \class CMax6864
/// \brief This class represents a MAX6864 WDT.
// ---------------------------------------------------------------------------------------------------------------------
class ZM_EXTERNAL_PERIPHERAL_API CMax6864 final : public zero_mate::IExternal_Peripheral
{
public:

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Creates an instance of the class.
	/// \param name Unique name of the device
	/// \param pin_reset_idx Reset pin index
	/// \param pin_wdi_idx WDT input pin index
	/// \param pin_mr_idx Manual pin index
	/// \param read_pin Function the peripheral uses to read GPIO pin
	/// \param set_pin Function the peripheral uses to set GPIO pin
	/// \param logging_system Logging system
	// -----------------------------------------------------------------------------------------------------------------
	explicit CMax6864(const std::string& name,
		std::uint32_t pin_reset_idx,
		std::uint32_t pin_wdi_idx,
		std::uint32_t pin_mr_idx,
		zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
		zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
		zero_mate::utils::CLogging_System* logging_system);

	~CMax6864();

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Callback function that notifies the peripheral about a change of one of the pins it subscribes to.
	/// \param pin_idx Index of the GPIO pin whose state has been changed
	// -----------------------------------------------------------------------------------------------------------------
	void GPIO_Subscription_Callback(std::uint32_t pin_idx) override;

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Renders the WDT (GUI).
	// -----------------------------------------------------------------------------------------------------------------
	void Render() override;

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Sets an ImGuiContext, so the WDT can render itself as a GUI window.
	/// \param context ImGuiContext the WDT uses to render itself
	// -----------------------------------------------------------------------------------------------------------------
	void Set_ImGui_Context(void* context) override;

private:

	/// Path to the external peripherals config file.
	const char* const CMax6864_Config_File = "peripherals/max6864_config.json";

	[[nodiscard]] inline nlohmann::json Parse_JSON_File(const std::string& path);

	inline void Initialize();

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Initializes the subscription (list of GPIO pins the peripheral wants to listen to).
	// -----------------------------------------------------------------------------------------------------------------
	inline void Init_GPIO_Subscription();

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Controls time interval of WDT.
	// -----------------------------------------------------------------------------------------------------------------
	void Control_Loop();

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Sets an ImGuiContext, so the WDT can render itself as a GUI window.
	// -----------------------------------------------------------------------------------------------------------------
	void Bark();
	
	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Renders the number of GPIO pins the WDT is connected to.
	// -----------------------------------------------------------------------------------------------------------------
	inline void Render_Pins_Idx() const;

	// -----------------------------------------------------------------------------------------------------------------
	/// \brief Renders the WDT itself.
	// -----------------------------------------------------------------------------------------------------------------
	inline void Render_WDT();

	std::string m_name;											///< Unique name of the peripheral
	std::uint32_t m_pin_reset_idx;                              ///< GPIO pin the Watchdog
	std::uint32_t m_pin_wdi_idx;                                ///< GPIO pin the LED is hooked up to
	std::uint32_t m_pin_mr_idx;                                 ///< GPIO pin the LED is hooked up to
	zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t m_read_pin;///< Function used to read the
	zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t m_set_pin;	///< Function used to read the
	bool m_pin_high;                                            ///< Current state of the LED
	std::chrono::steady_clock::time_point last_kick_time;		///< Last time the WDT was kicked
	std::atomic<int64_t> last_kick_ticks;
	std::mutex last_kick_time_mutex;
	std::uint32_t m_time_interval;								///< Time after which WDT sets reset pin

	zero_mate::utils::CLogging_System* m_logging_system;        ///< Logging system

	std::thread m_wdt_thread;
	std::atomic<bool> m_should_stop;

	ImGuiContext* m_context;									///< ImGUI context (rendering the GUI)
	bool m_output;												///< Current output from the WDT

	bool m_bark;												///< State when WDT barks at MCU
};
