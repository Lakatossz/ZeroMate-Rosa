#pragma once

#include <array>
#include <queue>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include "zero_mate/external_peripheral.hpp"

class CHall_Effect_Sensor final : public zero_mate::IExternal_Peripheral
{
public:

	explicit CHall_Effect_Sensor(std::string name,
		std::uint32_t pin_idx,
		zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin);

private:

	inline void Magnetic_Detected();

	std::string m_name;											///< Unique name of the peripheral
	std::uint32_t m_pin_idx;									///< GPIO pin the Watchdog
	zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t m_set_pin;	///< Function used to read the
	bool m_output;												///< Current output from the WDT
	bool m_pin_high;                                            ///< Current state of the LED
};
