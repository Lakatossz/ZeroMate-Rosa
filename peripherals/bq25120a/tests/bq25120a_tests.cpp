#include "gtest/gtest.h"
#include <filesystem>
#include <array>
#include <memory>
#include <chrono>
#include <thread>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include "zero_mate/external_peripheral.hpp"
#include "zero_mate/utils/singleton.hpp"
#include "../src/bq25120a.hpp"
#include "../../../src/mock/mock_gpio/mock_gpio.hpp"
#include "../../../src/mock/mock_bsc/mock_bsc.hpp"

using json = nlohmann::json;
using namespace zero_mate;
namespace fs = std::filesystem;

class Test_Bq25120a : public ::testing::Test {
private:
    static inline Test_Bq25120a* current_instance{ nullptr };

public:
    Test_Bq25120a() {
        current_instance = this; // Správné nastavení instance pro pøístup k èlenským funkcím
    }

    float Read(std::uint32_t address) {
        char buffer[32] = {};

        char s;

        std::uint32_t read_address = address | 1;

        // Nastaveni I2C adresy senzoru v BSC
        m_bsc->Write(static_cast<uint32_t>(peripheral::IBSC::NRegister::Slave_Address),
            reinterpret_cast<const char*>(&read_address), sizeof(uint32_t));

        // Nastaveni delky dat, ktere budu cist v BSC
        std::uint32_t size = 4;
        m_bsc->Write(static_cast<uint32_t>(peripheral::IBSC::NRegister::Data_Length),
            reinterpret_cast<const char*>(&size), sizeof(uint32_t));

        uint32_t control_reg_value = ((1 << 9) | (1 << 8) | (1 << 1));
        char* data_to_write = reinterpret_cast<char*>(&control_reg_value);

        // Nastaveni Status registru pro cteni
        m_bsc->Write(static_cast<uint32_t>(peripheral::IBSC::NRegister::Status),
            reinterpret_cast<const char*>(data_to_write), sizeof(uint32_t));

        control_reg_value = ((1 << 15) | (1 << 7) | (1 << 4) | (1 << 0));
        data_to_write = reinterpret_cast<char*>(&control_reg_value);

        // Nastaveni Control registru pro cteni
        m_bsc->Write(static_cast<uint32_t>(peripheral::IBSC::NRegister::Control),
            reinterpret_cast<const char*>(data_to_write), sizeof(uint32_t));

        for (int i = 0; i < 10000; ++i) {
            m_bsc->Increment_Passed_Cycles(1);
        }

        // Cteni dat z FIFO
        for (int i = 0; i < sizeof(float); ++i) {
            m_bsc->Read(static_cast<uint32_t>(peripheral::IBSC::NRegister::Data_FIFO),
                &buffer[i], sizeof(uint8_t));
        }

        control_reg_value = (1 << 4);
        data_to_write = reinterpret_cast<char*>(&control_reg_value);

        // Vyprazdneni FIFO
        m_bsc->Write(static_cast<uint32_t>(peripheral::IBSC::NRegister::Control),
            reinterpret_cast<const char*>(data_to_write), sizeof(uint32_t));

        float value;
        std::memcpy(&value, buffer, sizeof(float));
        return value;
    }

    void Write(std::uint32_t command, std::uint32_t address) {
        char s;
        uint32_t control_reg_value = (1 << 4);
        char* data_to_write = reinterpret_cast<char*>(&control_reg_value);

        // Vyprazdneni FIFO
        m_bsc->Write(static_cast<uint32_t>(peripheral::IBSC::NRegister::Control),
            data_to_write, sizeof(uint32_t));

        // Nastavení I2C adresy senzoru v BSC
        m_bsc->Write(static_cast<uint32_t>(peripheral::IBSC::NRegister::Slave_Address),
            reinterpret_cast<const char*>(&address), sizeof(uint32_t));

        // Nastaveni delky dat, ktere budu zapisovat
        std::uint32_t size = 2;
        m_bsc->Write(static_cast<uint32_t>(peripheral::IBSC::NRegister::Data_Length),
            reinterpret_cast<const char*>(&size), sizeof(uint32_t));

        control_reg_value = (1 << 9) | (1 << 8) | (1 << 1);
        data_to_write = reinterpret_cast<char*>(&control_reg_value);

        // Nastaveni Status registru pro zapis
        m_bsc->Write(static_cast<uint32_t>(peripheral::IBSC::NRegister::Status),
            reinterpret_cast<const char*>(data_to_write), sizeof(uint32_t));

        control_reg_value = (1 << 15) | (1 << 7);
        data_to_write = reinterpret_cast<char*>(&control_reg_value);

        // Nastaveni Control registru pro zapis
        m_bsc->Write(static_cast<uint32_t>(peripheral::IBSC::NRegister::Control),
            reinterpret_cast<const char*>(data_to_write), sizeof(uint32_t));

        control_reg_value = 0x00;
        data_to_write = reinterpret_cast<char*>(&control_reg_value);

        // Zapis zacatek prikazu do FIFO
        m_bsc->Write(static_cast<uint32_t>(peripheral::IBSC::NRegister::Data_FIFO),
            reinterpret_cast<const char*>(data_to_write), sizeof(uint32_t));

        // Zapis prikaz do FIFO
        m_bsc->Write(static_cast<uint32_t>(peripheral::IBSC::NRegister::Data_FIFO),
            reinterpret_cast<const char*>(&command), sizeof(uint32_t));

        for (int i = 0; i < 10000; ++i) {
            m_bsc->Increment_Passed_Cycles(1);
        }

        control_reg_value = 1 << 4;
        data_to_write = reinterpret_cast<char*>(&control_reg_value);
        // Vyprazdneni FIFO
        m_bsc->Write(static_cast<uint32_t>(peripheral::IBSC::NRegister::Control),
            reinterpret_cast<const char*>(data_to_write), sizeof(uint32_t));
    }

    float GetTarget(std::string config_string, std::string label) {
        float target = 0.0f;

        try {
            // Parsování JSON
            json j = json::parse(config_string);

            if (j["configuration"][0].contains(label.c_str()) &&
                j["configuration"][0][label.c_str()].is_array() &&
                !j["configuration"][0][label.c_str()].empty() &&
                j["configuration"][0][label.c_str()][0].is_number_unsigned())
            {
                target = j["configuration"][0][label.c_str()][0].get<float>();
            }
        }
        catch (const std::exception& e) {
            std::cerr << "Chyba parsovani JSON: " << e.what() << std::endl;
        }

        return target;
    }

protected:
    void SetUp() override {
        // Inicializace mockovaného GPIO
        m_gpio = std::make_shared<zero_mate::peripheral::CGPIO_Manager_Mock>();
        zero_mate::peripheral::CGPIO_Manager_Mock::current_instance = m_gpio.get();

        // Vytvoøení BSC øadièe (I2C master)
        m_bsc = std::make_unique<peripheral::Mock_BSC>(m_gpio);

        m_logging_system = utils::CSingleton<utils::CLogging_System>::Get_Instance();

        // Vytvoøení konfiguraèního souboru
        fs::create_directories("peripherals");
    }

    void TearDown() override {
        m_battery.release();
        m_bsc.release();
        fs::remove_all("peripherals");
    }

    std::shared_ptr<zero_mate::peripheral::CGPIO_Manager_Mock> m_gpio;
    std::unique_ptr<peripheral::Mock_BSC> m_bsc;
    std::unique_ptr<CBq25120a> m_battery;
    std::unique_ptr<CBq25120a> m_battery_1;
    std::unique_ptr<CBq25120a> m_battery_2;
    std::unique_ptr<CBq25120a> m_battery_3;
    utils::CLogging_System* m_logging_system;
    std::uint32_t m_address = 0x48;
    std::uint32_t m_address_1 = 0x4A;
    std::uint32_t m_address_2 = 0x4C;
    std::uint32_t m_address_3 = 0x4E;
    std::uint32_t m_sda_pin = 2;
    std::uint32_t m_scl_pin = 3;
};

//-----------------------------------------------------------------------------
// Testovací pøípady
//-----------------------------------------------------------------------------
TEST_F(Test_Bq25120a, Basic_Initialization) {
    bool correct = true;

    std::string config_string = R"({
      "configuration": [
        {
          "full_capacity": [ 4000 ],
          "capacity_downfall": [ 10 ],
          "referenced_temperature": [ 25 ],
          "higher_temperature": [ 50 ],
          "lower_temperature": [ 5 ],
          "temperature_start_with": [ 0 ],
          "referenced_charge": [ 100 ],
          "higher_charge": [ 100 ],
          "lower_charge": [ 10 ],
          "charge_start_with": [ 0 ],
          "referenced_voltage": [ 3700 ],
          "higher_voltage": [ 4200 ],
          "lower_voltage": [ 3000 ],
          "voltage_start_with": [ 0 ],
          "referenced_current": [ 100 ],
          "higher_current": [ 1000 ],
          "lower_current": [ 50 ],
          "current_start_with": [ 0 ]
        }
      ]
    })";

    std::ofstream("peripherals/bq25120a_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_battery = std::make_unique<CBq25120a>(
            "TestBattery",
            m_address,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );
    }
    catch (const std::exception& e) {
        correct = false;
    }

    m_gpio->Add_External_Peripheral(m_battery.get());

    EXPECT_TRUE(correct);
}

TEST_F(Test_Bq25120a, Read_Charge_Once) {
    bool correct = true;

    std::string config_string = R"({
      "configuration": [
        {
          "full_capacity": [ 4000 ],
          "capacity_downfall": [ 10 ],
          "referenced_temperature": [ 25 ],
          "higher_temperature": [ 50 ],
          "lower_temperature": [ 5 ],
          "temperature_start_with": [ 0 ],
          "referenced_charge": [ 100 ],
          "higher_charge": [ 100 ],
          "lower_charge": [ 10 ],
          "charge_start_with": [ 0 ],
          "referenced_voltage": [ 3700 ],
          "higher_voltage": [ 4200 ],
          "lower_voltage": [ 3000 ],
          "voltage_start_with": [ 0 ],
          "referenced_current": [ 100 ],
          "higher_current": [ 1000 ],
          "lower_current": [ 50 ],
          "current_start_with": [ 0 ]
        }
      ]
    })";

    std::ofstream("peripherals/bq25120a_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_battery = std::make_unique<CBq25120a>(
            "TestBattery",
            m_address,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );
    }
    catch (const std::exception& e) {
        correct = false;
    }

    if (m_battery != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_battery.get());

        Write(0x81, m_address);

        const float target = GetTarget(config_string, "referenced_charge");
        const float epsilon = 0.01;
        correct = std::abs(Read(m_address) - target) < epsilon;
    }
    else {
        correct = false;
    }

    EXPECT_TRUE(correct);
}

TEST_F(Test_Bq25120a, Read_Charge_Many_Times) {
    bool correct = true;

    std::string config_string = R"({
      "configuration": [
        {
          "full_capacity": [ 4000 ],
          "capacity_downfall": [ 10 ],
          "referenced_temperature": [ 25 ],
          "higher_temperature": [ 50 ],
          "lower_temperature": [ 5 ],
          "temperature_start_with": [ 0 ],
          "referenced_charge": [ 100 ],
          "higher_charge": [ 100 ],
          "lower_charge": [ 10 ],
          "charge_start_with": [ 0 ],
          "referenced_voltage": [ 3700 ],
          "higher_voltage": [ 4200 ],
          "lower_voltage": [ 3000 ],
          "voltage_start_with": [ 0 ],
          "referenced_current": [ 100 ],
          "higher_current": [ 1000 ],
          "lower_current": [ 50 ],
          "current_start_with": [ 0 ]
        }
      ]
    })";

    std::ofstream("peripherals/bq25120a_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_battery = std::make_unique<CBq25120a>(
            "TestBattery",
            m_address,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );
    }
    catch (const std::exception& e) {
        correct = false;
    }

    if (m_battery != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_battery.get());

        const float target = GetTarget(config_string, "referenced_charge");
        const float epsilon = 1.0f;

        for (int i = 0; i < 10; ++i) {
            Write(0x81, m_address);
            correct = std::abs(Read(m_address) - target) < epsilon;
        }
    }
    else {
        correct = false;
    }

    EXPECT_TRUE(correct);
}

TEST_F(Test_Bq25120a, Read_Higher_Charge_Once) {
    bool correct = true;

    std::string config_string = R"({
      "configuration": [
        {
          "full_capacity": [ 4000 ],
          "capacity_downfall": [ 10 ],
          "referenced_temperature": [ 25 ],
          "higher_temperature": [ 50 ],
          "lower_temperature": [ 5 ],
          "temperature_start_with": [ 0 ],
          "referenced_charge": [ 100 ],
          "higher_charge": [ 100 ],
          "lower_charge": [ 10 ],
          "charge_start_with": [ 1 ],
          "referenced_voltage": [ 3700 ],
          "higher_voltage": [ 4200 ],
          "lower_voltage": [ 3000 ],
          "voltage_start_with": [ 0 ],
          "referenced_current": [ 100 ],
          "higher_current": [ 1000 ],
          "lower_current": [ 50 ],
          "current_start_with": [ 0 ]
        }
      ]
    })";

    std::ofstream("peripherals/bq25120a_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_battery = std::make_unique<CBq25120a>(
            "TestBattery",
            m_address,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );
    }
    catch (const std::exception& e) {
        correct = false;
    }

    if (m_battery != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_battery.get());

        Write(0x81, m_address);

        const float target = GetTarget(config_string, "higher_charge");
        const float epsilon = 0.01;
        correct = std::abs(Read(m_address) - target) < epsilon;
    }
    else {
        correct = false;
    }

    EXPECT_TRUE(correct);
}

TEST_F(Test_Bq25120a, Read_Lower_Charge_Once) {
    bool correct = true;

    std::string config_string = R"({
      "configuration": [
        {
          "full_capacity": [ 4000 ],
          "capacity_downfall": [ 10 ],
          "referenced_temperature": [ 25 ],
          "higher_temperature": [ 50 ],
          "lower_temperature": [ 5 ],
          "temperature_start_with": [ 0 ],
          "referenced_charge": [ 100 ],
          "higher_charge": [ 100 ],
          "lower_charge": [ 10 ],
          "charge_start_with": [ 2 ],
          "referenced_voltage": [ 3700 ],
          "higher_voltage": [ 4200 ],
          "lower_voltage": [ 3000 ],
          "voltage_start_with": [ 0 ],
          "referenced_current": [ 100 ],
          "higher_current": [ 1000 ],
          "lower_current": [ 50 ],
          "current_start_with": [ 0 ]
        }
      ]
    })";

    std::ofstream("peripherals/bq25120a_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_battery = std::make_unique<CBq25120a>(
            "TestBattery",
            m_address,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );
    }
    catch (const std::exception& e) {
        correct = false;
    }

    if (m_battery != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_battery.get());

        Write(0x81, m_address);

        const float target = GetTarget(config_string, "lower_charge");
        const float epsilon = 0.01;
        correct = std::abs(Read(m_address) - target) < epsilon;
    }
    else {
        correct = false;
    }

    EXPECT_TRUE(correct);
}

TEST_F(Test_Bq25120a, Read_Charge_Once_With_Other_I2C_Devices) {
    bool correct = true;

    std::string config_string = R"({
      "configuration": [
        {
          "full_capacity": [ 4000 ],
          "capacity_downfall": [ 10 ],
          "referenced_temperature": [ 25 ],
          "higher_temperature": [ 50 ],
          "lower_temperature": [ 5 ],
          "temperature_start_with": [ 0 ],
          "referenced_charge": [ 100 ],
          "higher_charge": [ 100 ],
          "lower_charge": [ 10 ],
          "charge_start_with": [ 0 ],
          "referenced_voltage": [ 3700 ],
          "higher_voltage": [ 4200 ],
          "lower_voltage": [ 3000 ],
          "voltage_start_with": [ 0 ],
          "referenced_current": [ 100 ],
          "higher_current": [ 1000 ],
          "lower_current": [ 50 ],
          "current_start_with": [ 0 ]
        }
      ]
    })";

    std::ofstream("peripherals/bq25120a_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_battery = std::make_unique<CBq25120a>(
            "TestBattery",
            m_address,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );

        m_battery_1 = std::make_unique<CBq25120a>(
            "TestBattery",
            m_address_1,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );

        m_battery_2 = std::make_unique<CBq25120a>(
            "TestBattery2",
            m_address_2,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );

        m_battery_3 = std::make_unique<CBq25120a>(
            "TestBattery3",
            m_address_3,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );
    }
    catch (const std::exception& e) {
        correct = false;
    }

    if (m_battery != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_battery.get());
        m_gpio->Add_External_Peripheral(m_battery_1.get());
        m_gpio->Add_External_Peripheral(m_battery_2.get());
        m_gpio->Add_External_Peripheral(m_battery_3.get());

        const float target = GetTarget(config_string, "referenced_charge");
        const float epsilon = 0.01;

        Write(0x81, m_address);
        correct = std::abs(Read(m_address) - target) < epsilon;

        Write(0x81, m_address_1);
        correct = std::abs(Read(m_address_1) - target) < epsilon;

        Write(0x81, m_address_2);
        correct = std::abs(Read(m_address_2) - target) < epsilon;

        Write(0x81, m_address_3);
        correct = std::abs(Read(m_address_3) - target) < epsilon;
    }
    else {
        correct = false;
    }

    EXPECT_TRUE(correct);
}

TEST_F(Test_Bq25120a, Read_Voltage_Once) {
    bool correct = true;

    std::string config_string = R"({
      "configuration": [
        {
          "full_capacity": [ 4000 ],
          "capacity_downfall": [ 10 ],
          "referenced_temperature": [ 25 ],
          "higher_temperature": [ 50 ],
          "lower_temperature": [ 5 ],
          "temperature_start_with": [ 0 ],
          "referenced_charge": [ 100 ],
          "higher_charge": [ 100 ],
          "lower_charge": [ 10 ],
          "charge_start_with": [ 0 ],
          "referenced_voltage": [ 3700 ],
          "higher_voltage": [ 4200 ],
          "lower_voltage": [ 3000 ],
          "voltage_start_with": [ 0 ],
          "referenced_current": [ 100 ],
          "higher_current": [ 1000 ],
          "lower_current": [ 50 ],
          "current_start_with": [ 0 ]
        }
      ]
    })";

    std::ofstream("peripherals/bq25120a_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_battery = std::make_unique<CBq25120a>(
            "TestBattery",
            m_address,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );
    }
    catch (const std::exception& e) {
        correct = false;
    }

    if (m_battery != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_battery.get());

        Write(0xA5, m_address);

        const float target = GetTarget(config_string, "referenced_voltage");
        const float epsilon = 0.01;
        correct = std::abs(Read(m_address) - target) < epsilon;
    }
    else {
        correct = false;
    }

    EXPECT_TRUE(correct);
}

TEST_F(Test_Bq25120a, Read_Voltage_Many_Times) {
    bool correct = true;

    std::string config_string = R"({
      "configuration": [
        {
          "full_capacity": [ 4000 ],
          "capacity_downfall": [ 10 ],
          "referenced_temperature": [ 25 ],
          "higher_temperature": [ 50 ],
          "lower_temperature": [ 5 ],
          "temperature_start_with": [ 0 ],
          "referenced_charge": [ 100 ],
          "higher_charge": [ 100 ],
          "lower_charge": [ 10 ],
          "charge_start_with": [ 0 ],
          "referenced_voltage": [ 3700 ],
          "higher_voltage": [ 4200 ],
          "lower_voltage": [ 3000 ],
          "voltage_start_with": [ 0 ],
          "referenced_current": [ 100 ],
          "higher_current": [ 1000 ],
          "lower_current": [ 50 ],
          "current_start_with": [ 0 ]
        }
      ]
    })";

    std::ofstream("peripherals/bq25120a_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_battery = std::make_unique<CBq25120a>(
            "TestBattery",
            m_address,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );
    }
    catch (const std::exception& e) {
        correct = false;
    }

    if (m_battery != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_battery.get());

        const float target = GetTarget(config_string, "referenced_voltage");
        const float epsilon = 1.0f;

        for (int i = 0; i < 10; ++i) {
            Write(0xA5, m_address);
            correct = std::abs(Read(m_address) - target) < epsilon;
        }
    }
    else {
        correct = false;
    }

    EXPECT_TRUE(correct);
}

TEST_F(Test_Bq25120a, Read_Higher_Voltage_Once) {
    bool correct = true;

    std::string config_string = R"({
      "configuration": [
        {
          "full_capacity": [ 4000 ],
          "capacity_downfall": [ 10 ],
          "referenced_temperature": [ 25 ],
          "higher_temperature": [ 50 ],
          "lower_temperature": [ 5 ],
          "temperature_start_with": [ 0 ],
          "referenced_charge": [ 100 ],
          "higher_charge": [ 100 ],
          "lower_charge": [ 10 ],
          "charge_start_with": [ 0 ],
          "referenced_voltage": [ 3700 ],
          "higher_voltage": [ 4200 ],
          "lower_voltage": [ 3000 ],
          "voltage_start_with": [ 1 ],
          "referenced_current": [ 100 ],
          "higher_current": [ 1000 ],
          "lower_current": [ 50 ],
          "current_start_with": [ 0 ]
        }
      ]
    })";

    std::ofstream("peripherals/bq25120a_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_battery = std::make_unique<CBq25120a>(
            "TestBattery",
            m_address,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );
    }
    catch (const std::exception& e) {
        correct = false;
    }

    if (m_battery != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_battery.get());

        Write(0xA5, m_address);

        const float target = GetTarget(config_string, "higher_voltage");
        const float epsilon = 0.01;
        correct = std::abs(Read(m_address) - target) < epsilon;
    }
    else {
        correct = false;
    }

    EXPECT_TRUE(correct);
}

TEST_F(Test_Bq25120a, Read_Lower_Voltage_Once) {
    bool correct = true;

    std::string config_string = R"({
      "configuration": [
        {
          "full_capacity": [ 4000 ],
          "capacity_downfall": [ 10 ],
          "referenced_temperature": [ 25 ],
          "higher_temperature": [ 50 ],
          "lower_temperature": [ 5 ],
          "temperature_start_with": [ 0 ],
          "referenced_charge": [ 100 ],
          "higher_charge": [ 100 ],
          "lower_charge": [ 10 ],
          "charge_start_with": [ 0 ],
          "referenced_voltage": [ 3700 ],
          "higher_voltage": [ 4200 ],
          "lower_voltage": [ 3000 ],
          "voltage_start_with": [ 2 ],
          "referenced_current": [ 100 ],
          "higher_current": [ 1000 ],
          "lower_current": [ 50 ],
          "current_start_with": [ 0 ]
        }
      ]
    })";

    std::ofstream("peripherals/bq25120a_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_battery = std::make_unique<CBq25120a>(
            "TestBattery",
            m_address,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );
    }
    catch (const std::exception& e) {
        correct = false;
    }

    if (m_battery != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_battery.get());

        Write(0xA5, m_address);

        const float target = GetTarget(config_string, "lower_voltage");
        const float epsilon = 0.01;
        correct = std::abs(Read(m_address) - target) < epsilon;
    }
    else {
        correct = false;
    }

    EXPECT_TRUE(correct);
}

TEST_F(Test_Bq25120a, Read_Voltage_Once_With_Other_I2C_Devices) {
    bool correct = true;

    std::string config_string = R"({
      "configuration": [
        {
          "full_capacity": [ 4000 ],
          "capacity_downfall": [ 10 ],
          "referenced_temperature": [ 25 ],
          "higher_temperature": [ 50 ],
          "lower_temperature": [ 5 ],
          "temperature_start_with": [ 0 ],
          "referenced_charge": [ 100 ],
          "higher_charge": [ 100 ],
          "lower_charge": [ 10 ],
          "charge_start_with": [ 0 ],
          "referenced_voltage": [ 3700 ],
          "higher_voltage": [ 4200 ],
          "lower_voltage": [ 3000 ],
          "voltage_start_with": [ 0 ],
          "referenced_current": [ 100 ],
          "higher_current": [ 1000 ],
          "lower_current": [ 50 ],
          "current_start_with": [ 0 ]
        }
      ]
    })";

    std::ofstream("peripherals/bq25120a_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_battery = std::make_unique<CBq25120a>(
            "TestBattery",
            m_address,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );

        m_battery_1 = std::make_unique<CBq25120a>(
            "TestBattery",
            m_address_1,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );

        m_battery_2 = std::make_unique<CBq25120a>(
            "TestBattery2",
            m_address_2,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );

        m_battery_3 = std::make_unique<CBq25120a>(
            "TestBattery3",
            m_address_3,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );
    }
    catch (const std::exception& e) {
        correct = false;
    }

    if (m_battery != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_battery.get());
        m_gpio->Add_External_Peripheral(m_battery_1.get());
        m_gpio->Add_External_Peripheral(m_battery_2.get());
        m_gpio->Add_External_Peripheral(m_battery_3.get());

        const float target = GetTarget(config_string, "referenced_voltage");
        const float epsilon = 0.01;

        Write(0xA5, m_address);
        correct = std::abs(Read(m_address) - target) < epsilon;

        Write(0xA5, m_address_1);
        correct = std::abs(Read(m_address_1) - target) < epsilon;

        Write(0xA5, m_address_2);
        correct = std::abs(Read(m_address_2) - target) < epsilon;

        Write(0xA5, m_address_3);
        correct = std::abs(Read(m_address_3) - target) < epsilon;
    }
    else {
        correct = false;
    }

    EXPECT_TRUE(correct);
}
TEST_F(Test_Bq25120a, Read_Current_Once) {
    bool correct = true;

    std::string config_string = R"({
      "configuration": [
        {
          "full_capacity": [ 4000 ],
          "capacity_downfall": [ 10 ],
          "referenced_temperature": [ 25 ],
          "higher_temperature": [ 50 ],
          "lower_temperature": [ 5 ],
          "temperature_start_with": [ 0 ],
          "referenced_charge": [ 100 ],
          "higher_charge": [ 100 ],
          "lower_charge": [ 10 ],
          "charge_start_with": [ 0 ],
          "referenced_voltage": [ 3700 ],
          "higher_voltage": [ 4200 ],
          "lower_voltage": [ 3000 ],
          "voltage_start_with": [ 0 ],
          "referenced_current": [ 100 ],
          "higher_current": [ 1000 ],
          "lower_current": [ 50 ],
          "current_start_with": [ 0 ]
        }
      ]
    })";

    std::ofstream("peripherals/bq25120a_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_battery = std::make_unique<CBq25120a>(
            "TestBattery",
            m_address,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );
    }
    catch (const std::exception& e) {
        correct = false;
    }

    if (m_battery != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_battery.get());

        Write(0xA6, m_address);

        const float target = GetTarget(config_string, "referenced_current");
        const float epsilon = 0.01;
        correct = std::abs(Read(m_address) - target) < epsilon;
    }
    else {
        correct = false;
    }

    EXPECT_TRUE(correct);
}

TEST_F(Test_Bq25120a, Read_Current_Many_Times) {
    bool correct = true;

    std::string config_string = R"({
      "configuration": [
        {
          "full_capacity": [ 4000 ],
          "capacity_downfall": [ 10 ],
          "referenced_temperature": [ 25 ],
          "higher_temperature": [ 50 ],
          "lower_temperature": [ 5 ],
          "temperature_start_with": [ 0 ],
          "referenced_charge": [ 100 ],
          "higher_charge": [ 100 ],
          "lower_charge": [ 10 ],
          "charge_start_with": [ 0 ],
          "referenced_voltage": [ 3700 ],
          "higher_voltage": [ 4200 ],
          "lower_voltage": [ 3000 ],
          "voltage_start_with": [ 0 ],
          "referenced_current": [ 100 ],
          "higher_current": [ 1000 ],
          "lower_current": [ 50 ],
          "current_start_with": [ 0 ]
        }
      ]
    })";

    std::ofstream("peripherals/bq25120a_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_battery = std::make_unique<CBq25120a>(
            "TestBattery",
            m_address,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );
    }
    catch (const std::exception& e) {
        correct = false;
    }

    if (m_battery != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_battery.get());

        const float target = GetTarget(config_string, "referenced_current");
        const float epsilon = 1.0f;

        for (int i = 0; i < 10; ++i) {
            Write(0xA6, m_address);
            correct = std::abs(Read(m_address) - target) < epsilon;
        }
    }
    else {
        correct = false;
    }

    EXPECT_TRUE(correct);
}

TEST_F(Test_Bq25120a, Read_Higher_Current_Once) {
    bool correct = true;

    std::string config_string = R"({
      "configuration": [
        {
          "full_capacity": [ 4000 ],
          "capacity_downfall": [ 10 ],
          "referenced_temperature": [ 25 ],
          "higher_temperature": [ 50 ],
          "lower_temperature": [ 5 ],
          "temperature_start_with": [ 0 ],
          "referenced_charge": [ 100 ],
          "higher_charge": [ 100 ],
          "lower_charge": [ 10 ],
          "charge_start_with": [ 0 ],
          "referenced_voltage": [ 3700 ],
          "higher_voltage": [ 4200 ],
          "lower_voltage": [ 3000 ],
          "voltage_start_with": [ 0 ],
          "referenced_current": [ 100 ],
          "higher_current": [ 1000 ],
          "lower_current": [ 50 ],
          "current_start_with": [ 1 ]
        }
      ]
    })";

    std::ofstream("peripherals/bq25120a_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_battery = std::make_unique<CBq25120a>(
            "TestBattery",
            m_address,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );
    }
    catch (const std::exception& e) {
        correct = false;
    }

    if (m_battery != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_battery.get());

        Write(0xA6, m_address);

        const float target = GetTarget(config_string, "higher_current");
        const float epsilon = 0.01;
        correct = std::abs(Read(m_address) - target) < epsilon;
    }
    else {
        correct = false;
    }

    EXPECT_TRUE(correct);
}

TEST_F(Test_Bq25120a, Read_Lower_Current_Once) {
    bool correct = true;

    std::string config_string = R"({
      "configuration": [
        {
          "full_capacity": [ 4000 ],
          "capacity_downfall": [ 10 ],
          "referenced_temperature": [ 25 ],
          "higher_temperature": [ 50 ],
          "lower_temperature": [ 5 ],
          "temperature_start_with": [ 0 ],
          "referenced_charge": [ 100 ],
          "higher_charge": [ 100 ],
          "lower_charge": [ 10 ],
          "charge_start_with": [ 0 ],
          "referenced_voltage": [ 3700 ],
          "higher_voltage": [ 4200 ],
          "lower_voltage": [ 3000 ],
          "voltage_start_with": [ 0 ],
          "referenced_current": [ 100 ],
          "higher_current": [ 1000 ],
          "lower_current": [ 50 ],
          "current_start_with": [ 2 ]
        }
      ]
    })";

    std::ofstream("peripherals/bq25120a_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_battery = std::make_unique<CBq25120a>(
            "TestBattery",
            m_address,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );
    }
    catch (const std::exception& e) {
        correct = false;
    }

    if (m_battery != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_battery.get());

        Write(0xA6, m_address);

        const float target = GetTarget(config_string, "lower_current");
        const float epsilon = 0.01;
        correct = std::abs(Read(m_address) - target) < epsilon;
    }
    else {
        correct = false;
    }

    EXPECT_TRUE(correct);
}

TEST_F(Test_Bq25120a, Read_Current_Once_With_Other_I2C_Devices) {
    bool correct = true;

    std::string config_string = R"({
      "configuration": [
        {
          "full_capacity": [ 4000 ],
          "capacity_downfall": [ 10 ],
          "referenced_temperature": [ 25 ],
          "higher_temperature": [ 50 ],
          "lower_temperature": [ 5 ],
          "temperature_start_with": [ 0 ],
          "referenced_charge": [ 100 ],
          "higher_charge": [ 100 ],
          "lower_charge": [ 10 ],
          "charge_start_with": [ 0 ],
          "referenced_voltage": [ 3700 ],
          "higher_voltage": [ 4200 ],
          "lower_voltage": [ 3000 ],
          "voltage_start_with": [ 0 ],
          "referenced_current": [ 100 ],
          "higher_current": [ 1000 ],
          "lower_current": [ 50 ],
          "current_start_with": [ 0 ]
        }
      ]
    })";

    std::ofstream("peripherals/bq25120a_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_battery = std::make_unique<CBq25120a>(
            "TestBattery",
            m_address,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );

        m_battery_1 = std::make_unique<CBq25120a>(
            "TestBattery",
            m_address_1,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );

        m_battery_2 = std::make_unique<CBq25120a>(
            "TestBattery2",
            m_address_2,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );

        m_battery_3 = std::make_unique<CBq25120a>(
            "TestBattery3",
            m_address_3,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );
    }
    catch (const std::exception& e) {
        correct = false;
    }

    if (m_battery != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_battery.get());
        m_gpio->Add_External_Peripheral(m_battery_1.get());
        m_gpio->Add_External_Peripheral(m_battery_2.get());
        m_gpio->Add_External_Peripheral(m_battery_3.get());

        const float target = GetTarget(config_string, "referenced_current");
        const float epsilon = 0.01;

        Write(0xA6, m_address);
        correct = std::abs(Read(m_address) - target) < epsilon;

        Write(0xA6, m_address_1);
        correct = std::abs(Read(m_address_1) - target) < epsilon;

        Write(0xA6, m_address_2);
        correct = std::abs(Read(m_address_2) - target) < epsilon;

        Write(0xA6, m_address_3);
        correct = std::abs(Read(m_address_3) - target) < epsilon;
    }
    else {
        correct = false;
    }

    EXPECT_TRUE(correct);
}
TEST_F(Test_Bq25120a, Read_Temperature_Once) {
    bool correct = true;

    std::string config_string = R"({
      "configuration": [
        {
          "full_capacity": [ 4000 ],
          "capacity_downfall": [ 10 ],
          "referenced_temperature": [ 25 ],
          "higher_temperature": [ 50 ],
          "lower_temperature": [ 5 ],
          "temperature_start_with": [ 0 ],
          "referenced_charge": [ 100 ],
          "higher_charge": [ 100 ],
          "lower_charge": [ 10 ],
          "charge_start_with": [ 0 ],
          "referenced_voltage": [ 3700 ],
          "higher_voltage": [ 4200 ],
          "lower_voltage": [ 3000 ],
          "voltage_start_with": [ 0 ],
          "referenced_current": [ 100 ],
          "higher_current": [ 1000 ],
          "lower_current": [ 50 ],
          "current_start_with": [ 0 ]
        }
      ]
    })";

    std::ofstream("peripherals/bq25120a_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_battery = std::make_unique<CBq25120a>(
            "TestBattery",
            m_address,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );
    }
    catch (const std::exception& e) {
        correct = false;
    }

    if (m_battery != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_battery.get());

        Write(0xA7, m_address);

        const float target = GetTarget(config_string, "referenced_temperature");
        const float epsilon = 0.01;
        correct = std::abs(Read(m_address) - target) < epsilon;
    }
    else {
        correct = false;
    }

    EXPECT_TRUE(correct);
}

TEST_F(Test_Bq25120a, Read_Temperature_Many_Times) {
    bool correct = true;

    std::string config_string = R"({
      "configuration": [
        {
          "full_capacity": [ 4000 ],
          "capacity_downfall": [ 10 ],
          "referenced_temperature": [ 25 ],
          "higher_temperature": [ 50 ],
          "lower_temperature": [ 5 ],
          "temperature_start_with": [ 0 ],
          "referenced_charge": [ 100 ],
          "higher_charge": [ 100 ],
          "lower_charge": [ 10 ],
          "charge_start_with": [ 0 ],
          "referenced_voltage": [ 3700 ],
          "higher_voltage": [ 4200 ],
          "lower_voltage": [ 3000 ],
          "voltage_start_with": [ 0 ],
          "referenced_current": [ 100 ],
          "higher_current": [ 1000 ],
          "lower_current": [ 50 ],
          "current_start_with": [ 0 ]
        }
      ]
    })";

    std::ofstream("peripherals/bq25120a_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_battery = std::make_unique<CBq25120a>(
            "TestBattery",
            m_address,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );
    }
    catch (const std::exception& e) {
        correct = false;
    }

    if (m_battery != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_battery.get());

        const float target = GetTarget(config_string, "referenced_temperature");
        const float epsilon = 1.0f;

        for (int i = 0; i < 10; ++i) {
            Write(0xA7, m_address);
            correct = std::abs(Read(m_address) - target) < epsilon;
        }
    }
    else {
        correct = false;
    }

    EXPECT_TRUE(correct);
}

TEST_F(Test_Bq25120a, Read_Higher_Temperature_Once) {
    bool correct = true;

    std::string config_string = R"({
      "configuration": [
        {
          "full_capacity": [ 4000 ],
          "capacity_downfall": [ 10 ],
          "referenced_temperature": [ 25 ],
          "higher_temperature": [ 50 ],
          "lower_temperature": [ 5 ],
          "temperature_start_with": [ 1 ],
          "referenced_charge": [ 100 ],
          "higher_charge": [ 100 ],
          "lower_charge": [ 10 ],
          "charge_start_with": [ 0 ],
          "referenced_voltage": [ 3700 ],
          "higher_voltage": [ 4200 ],
          "lower_voltage": [ 3000 ],
          "voltage_start_with": [ 0 ],
          "referenced_current": [ 100 ],
          "higher_current": [ 1000 ],
          "lower_current": [ 50 ],
          "current_start_with": [ 0 ]
        }
      ]
    })";

    std::ofstream("peripherals/bq25120a_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_battery = std::make_unique<CBq25120a>(
            "TestBattery",
            m_address,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );
    }
    catch (const std::exception& e) {
        correct = false;
    }

    if (m_battery != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_battery.get());

        Write(0xA7, m_address);

        const float target = GetTarget(config_string, "higher_temperature");
        const float epsilon = 0.01;
        correct = std::abs(Read(m_address) - target) < epsilon;
    }
    else {
        correct = false;
    }

    EXPECT_TRUE(correct);
}

TEST_F(Test_Bq25120a, Read_Lower_Temperature_Once) {
    bool correct = true;

    std::string config_string = R"({
      "configuration": [
        {
          "full_capacity": [ 4000 ],
          "capacity_downfall": [ 10 ],
          "referenced_temperature": [ 25 ],
          "higher_temperature": [ 50 ],
          "lower_temperature": [ 5 ],
          "temperature_start_with": [ 2 ],
          "referenced_charge": [ 100 ],
          "higher_charge": [ 100 ],
          "lower_charge": [ 10 ],
          "charge_start_with": [ 0 ],
          "referenced_voltage": [ 3700 ],
          "higher_voltage": [ 4200 ],
          "lower_voltage": [ 3000 ],
          "voltage_start_with": [ 0 ],
          "referenced_current": [ 100 ],
          "higher_current": [ 1000 ],
          "lower_current": [ 50 ],
          "current_start_with": [ 0 ]
        }
      ]
    })";

    std::ofstream("peripherals/bq25120a_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_battery = std::make_unique<CBq25120a>(
            "TestBattery",
            m_address,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );
    }
    catch (const std::exception& e) {
        correct = false;
    }

    if (m_battery != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_battery.get());

        Write(0xA7, m_address);

        const float target = GetTarget(config_string, "lower_temperature");
        const float epsilon = 0.01;
        correct = std::abs(Read(m_address) - target) < epsilon;
    }
    else {
        correct = false;
    }

    EXPECT_TRUE(correct);
}

TEST_F(Test_Bq25120a, Read_Temperature_Once_With_Other_I2C_Devices) {
    bool correct = true;

    std::string config_string = R"({
      "configuration": [
        {
          "full_capacity": [ 4000 ],
          "capacity_downfall": [ 10 ],
          "referenced_temperature": [ 25 ],
          "higher_temperature": [ 50 ],
          "lower_temperature": [ 5 ],
          "temperature_start_with": [ 0 ],
          "referenced_charge": [ 100 ],
          "higher_charge": [ 100 ],
          "lower_charge": [ 10 ],
          "charge_start_with": [ 0 ],
          "referenced_voltage": [ 3700 ],
          "higher_voltage": [ 4200 ],
          "lower_voltage": [ 3000 ],
          "voltage_start_with": [ 0 ],
          "referenced_current": [ 100 ],
          "higher_current": [ 1000 ],
          "lower_current": [ 50 ],
          "current_start_with": [ 0 ]
        }
      ]
    })";

    std::ofstream("peripherals/bq25120a_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_battery = std::make_unique<CBq25120a>(
            "TestBattery",
            m_address,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );

        m_battery_1 = std::make_unique<CBq25120a>(
            "TestBattery",
            m_address_1,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );

        m_battery_2 = std::make_unique<CBq25120a>(
            "TestBattery2",
            m_address_2,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );

        m_battery_3 = std::make_unique<CBq25120a>(
            "TestBattery3",
            m_address_3,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );
    }
    catch (const std::exception& e) {
        correct = false;
    }

    if (m_battery != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_battery.get());
        m_gpio->Add_External_Peripheral(m_battery_1.get());
        m_gpio->Add_External_Peripheral(m_battery_2.get());
        m_gpio->Add_External_Peripheral(m_battery_3.get());

        const float target = GetTarget(config_string, "referenced_temperature");
        const float epsilon = 0.01;

        Write(0xA7, m_address);
        correct = std::abs(Read(m_address) - target) < epsilon;

        Write(0xA7, m_address_1);
        correct = std::abs(Read(m_address_1) - target) < epsilon;

        Write(0xA7, m_address_2);
        correct = std::abs(Read(m_address_2) - target) < epsilon;

        Write(0xA7, m_address_3);
        correct = std::abs(Read(m_address_3) - target) < epsilon;
    }
    else {
        correct = false;
    }

    EXPECT_TRUE(correct);
}
