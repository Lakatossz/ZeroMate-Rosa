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
#include "../src/pca9685_mg996r.hpp"
#include "../../../src/mock/mock_gpio/mock_gpio.hpp"
#include "../../../src/mock/mock_bsc/mock_bsc.hpp"

using json = nlohmann::json;
using namespace zero_mate;
namespace fs = std::filesystem;

class Test_Pca9685_Mg996r : public ::testing::Test {
private:
    static inline Test_Pca9685_Mg996r* current_instance{ nullptr };

public:
    Test_Pca9685_Mg996r() {
        current_instance = this; // Správné nastavení instance pro pøístup k èlenským funkcím
    }

    std::uint8_t Read() {
        char buffer[8] = {};

        char s;

        std::uint32_t read_address = m_address | 1;

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
        m_bsc->Read(static_cast<uint32_t>(peripheral::IBSC::NRegister::Data_FIFO),
            &buffer[0], sizeof(uint8_t));

        control_reg_value = (1 << 4);
        data_to_write = reinterpret_cast<char*>(&control_reg_value);

        // Vyprazdneni FIFO
        m_bsc->Write(static_cast<uint32_t>(peripheral::IBSC::NRegister::Control),
            reinterpret_cast<const char*>(data_to_write), sizeof(uint32_t));
        return static_cast<std::uint8_t>(buffer[0]);
    }

    void Write(std::uint32_t command, char* data, std::uint32_t data_size) {
        char s;
        uint32_t control_reg_value = (1 << 4);
        char* data_to_write = reinterpret_cast<char*>(&control_reg_value);

        // Vyprazdneni FIFO
        m_bsc->Write(static_cast<uint32_t>(peripheral::IBSC::NRegister::Control),
            data_to_write, sizeof(uint32_t));

        // Nastavení I2C adresy senzoru v BSC
        m_bsc->Write(static_cast<uint32_t>(peripheral::IBSC::NRegister::Slave_Address),
            reinterpret_cast<const char*>(&m_address), sizeof(uint32_t));

        // Nastaveni delky dat, ktere budu zapisovat
        std::uint32_t size = 2 + data_size;
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

        for (int i = 0; i < data_size; ++i) {
            m_bsc->Write(static_cast<uint32_t>(peripheral::IBSC::NRegister::Data_FIFO),
                reinterpret_cast<const char*>(&data[i]), sizeof(uint32_t));
        }

        for (int i = 0; i < 10000; ++i) {
            m_bsc->Increment_Passed_Cycles(1);
        }

        control_reg_value = 1 << 4;
        data_to_write = reinterpret_cast<char*>(&control_reg_value);
        // Vyprazdneni FIFO
        m_bsc->Write(static_cast<uint32_t>(peripheral::IBSC::NRegister::Control),
            reinterpret_cast<const char*>(data_to_write), sizeof(uint32_t));
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
        m_motor.release();
        m_bsc.release();
        fs::remove_all("peripherals");
    }

    std::shared_ptr<zero_mate::peripheral::CGPIO_Manager_Mock> m_gpio;
    std::unique_ptr<peripheral::Mock_BSC> m_bsc;
    std::unique_ptr<CPca9685_Mg996r> m_motor;
    utils::CLogging_System* m_logging_system;
    std::uint32_t m_address = 0x48;
    std::uint32_t m_sda_pin = 2;
    std::uint32_t m_scl_pin = 3;
};

//-----------------------------------------------------------------------------
// Testovací pøípady
//-----------------------------------------------------------------------------
TEST_F(Test_Pca9685_Mg996r, Invalid_Path_Initialization) {
    bool correct = true;

    std::string config_string = R"({
          "configuration": [
            {
              "patient_library": [ "Invalid_Path" ],
              "error_chance": [ 0 ]
            }
          ]
        })";

    std::ofstream("peripherals/pca9685_mg996r_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_motor = std::make_unique<CPca9685_Mg996r>(
            "TestSensor",
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

    EXPECT_FALSE(correct);
}

TEST_F(Test_Pca9685_Mg996r, Invalid_Parameter_Initialization) {
    bool correct = true;

    std::string config_string = R"({
          "configuration": [
            {
              "invalid_parameter": [ "C:\\Users\\rosaj\\Source\\Repos\\ZeroMate\\peripherals\\cgm\\src\\patient\\simple-patient.dll" ],
              "error_chance": [ 0 ]
            }
          ]
        })";

    std::ofstream("peripherals/cgm_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_motor = std::make_unique<CPca9685_Mg996r>(
            "TestSensor",
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

    EXPECT_FALSE(correct);
}

TEST_F(Test_Pca9685_Mg996r, Basic_Initialization) {
    bool correct = true;

    std::string config_string = R"({
          "configuration": [
            {
              "patient_library": [ "C:\\Users\\rosaj\\Source\\Repos\\ZeroMate\\peripherals\\cgm\\src\\patient\\simple-patient.dll" ],
              "error_chance": [ 0 ]
            }
          ]
        })";

    std::ofstream("peripherals/pca9685_mg996r_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_motor = std::make_unique<CPca9685_Mg996r>(
            "TestSensor",
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

    m_gpio->Add_External_Peripheral(m_motor.get());

    EXPECT_TRUE(correct);
}

TEST_F(Test_Pca9685_Mg996r, Push_Forward_Once) {
    bool correct = true;

    std::string config_string = R"({
          "configuration": [
            {
              "patient_library": [ "C:\\Users\\rosaj\\Source\\Repos\\ZeroMate\\peripherals\\cgm\\src\\patient\\simple-patient.dll" ],
              "error_chance": [ 0 ]
            }
          ]
        })";

    std::ofstream("peripherals/pca9685_mg996r_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_motor = std::make_unique<CPca9685_Mg996r>(
            "TestSensor",
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

    if (m_motor != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_motor.get());

        std::uint16_t steps = 10;

        std::uint8_t high = (steps >> 8) & 0xFF;
        std::uint8_t low = steps & 0xFF;

        char buffer[2];
        buffer[0] = static_cast<char>(high);
        buffer[1] = static_cast<char>(low);

        Write(0x81, buffer, 2);
    }
    else {
        correct = false;
    }
    Write(0xA6, "", 0);
    correct = Read() == 0;

    EXPECT_TRUE(correct);
}

TEST_F(Test_Pca9685_Mg996r, Push_Forward_Many_Times) {
    bool correct = true;

    std::string config_string = R"({
          "configuration": [
            {
              "patient_library": [ "C:\\Users\\rosaj\\Source\\Repos\\ZeroMate\\peripherals\\cgm\\src\\patient\\simple-patient.dll" ],
              "error_chance": [ 0 ]
            }
          ]
        })";

    std::ofstream("peripherals/pca9685_mg996r_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_motor = std::make_unique<CPca9685_Mg996r>(
            "TestSensor",
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

    if (m_motor != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_motor.get());

        std::uint16_t steps = 10;

        std::uint8_t high = (steps >> 8) & 0xFF;
        std::uint8_t low = steps & 0xFF;

        char buffer[2];
        buffer[0] = static_cast<char>(high);
        buffer[1] = static_cast<char>(low);

        for (int i = 0; i < 10; ++i) {
            Write(0x81, buffer, 2);
            Write(0xA6, "", 0);
            correct = Read() == 0;
        }
    }
    else {
        correct = false;
    }

    EXPECT_TRUE(correct);
}

TEST_F(Test_Pca9685_Mg996r, Push_Forward_Once_With_Error) {
    bool correct = true;

    std::string config_string = R"({
          "configuration": [
            {
              "patient_library": [ "C:\\Users\\rosaj\\Source\\Repos\\ZeroMate\\peripherals\\cgm\\src\\patient\\simple-patient.dll" ],
              "error_chance": [ 100 ]
            }
          ]
        })";

    std::ofstream("peripherals/pca9685_mg996r_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_motor = std::make_unique<CPca9685_Mg996r>(
            "TestSensor",
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

    if (m_motor != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_motor.get());

        std::uint16_t steps = 10;

        std::uint8_t high = (steps >> 8) & 0xFF;
        std::uint8_t low = steps & 0xFF;

        char buffer[2];
        buffer[0] = static_cast<char>(high);
        buffer[1] = static_cast<char>(low);

        Write(0x81, buffer, 2);
    }
    else {
        correct = false;
    }

    Write(0xA6, "", 0);
    correct = Read() == 1;

    EXPECT_TRUE(correct);
}

TEST_F(Test_Pca9685_Mg996r, Push_Backward_Once) {
    bool correct = true;

    std::string config_string = R"({
          "configuration": [
            {
              "patient_library": [ "C:\\Users\\rosaj\\Source\\Repos\\ZeroMate\\peripherals\\cgm\\src\\patient\\simple-patient.dll" ],
              "error_chance": [ 0 ]
            }
          ]
        })";

    std::ofstream("peripherals/pca9685_mg996r_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_motor = std::make_unique<CPca9685_Mg996r>(
            "TestSensor",
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

    if (m_motor != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_motor.get());

        std::uint16_t steps = 10;

        std::uint8_t high = (steps >> 8) & 0xFF;
        std::uint8_t low = steps & 0xFF;

        char buffer[2];
        buffer[0] = static_cast<char>(high);
        buffer[1] = static_cast<char>(low);

        Write(0xA4, buffer, 2);
    }
    else {
        correct = false;
    }
    Write(0xA6, "", 0);
    correct = Read() == 0;

    EXPECT_TRUE(correct);
}

TEST_F(Test_Pca9685_Mg996r, Push_Backward_Many_Times) {
    bool correct = true;

    std::string config_string = R"({
          "configuration": [
            {
              "patient_library": [ "C:\\Users\\rosaj\\Source\\Repos\\ZeroMate\\peripherals\\cgm\\src\\patient\\simple-patient.dll" ],
              "error_chance": [ 0 ]
            }
          ]
        })";

    std::ofstream("peripherals/pca9685_mg996r_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_motor = std::make_unique<CPca9685_Mg996r>(
            "TestSensor",
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

    if (m_motor != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_motor.get());

        std::uint16_t steps = 10;

        std::uint8_t high = (steps >> 8) & 0xFF;
        std::uint8_t low = steps & 0xFF;

        char buffer[2];
        buffer[0] = static_cast<char>(high);
        buffer[1] = static_cast<char>(low);

        for (int i = 0; i < 10; ++i) {
            Write(0xA4, buffer, 2);
            Write(0xA6, "", 0);
            correct = Read() == 0;
        }
    }
    else {
        correct = false;
    }

    EXPECT_TRUE(correct);
}

TEST_F(Test_Pca9685_Mg996r, Push_Backward_Once_With_Error) {
    bool correct = true;

    std::string config_string = R"({
          "configuration": [
            {
              "patient_library": [ "C:\\Users\\rosaj\\Source\\Repos\\ZeroMate\\peripherals\\cgm\\src\\patient\\simple-patient.dll" ],
              "error_chance": [ 100 ]
            }
          ]
        })";

    std::ofstream("peripherals/pca9685_mg996r_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_motor = std::make_unique<CPca9685_Mg996r>(
            "TestSensor",
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

    if (m_motor != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_motor.get());

        std::uint16_t steps = 10;

        std::uint8_t high = (steps >> 8) & 0xFF;
        std::uint8_t low = steps & 0xFF;

        char buffer[2];
        buffer[0] = static_cast<char>(high);
        buffer[1] = static_cast<char>(low);

        Write(0xA4, buffer, 2);
    }
    else {
        correct = false;
    }

    Write(0xA6, "", 0);
    correct = Read() == 1;

    EXPECT_TRUE(correct);
}

TEST_F(Test_Pca9685_Mg996r, Push_Forward_And_Backward_Once) {
    bool correct = true;

    std::string config_string = R"({
          "configuration": [
            {
              "patient_library": [ "C:\\Users\\rosaj\\Source\\Repos\\ZeroMate\\peripherals\\cgm\\src\\patient\\simple-patient.dll" ],
              "error_chance": [ 0 ]
            }
          ]
        })";

    std::ofstream("peripherals/pca9685_mg996r_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_motor = std::make_unique<CPca9685_Mg996r>(
            "TestSensor",
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

    if (m_motor != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_motor.get());

        std::uint16_t steps = 10;

        std::uint8_t high = (steps >> 8) & 0xFF;
        std::uint8_t low = steps & 0xFF;

        char buffer[2];
        buffer[0] = static_cast<char>(high);
        buffer[1] = static_cast<char>(low);

        Write(0x81, buffer, 2);
        Write(0xA4, buffer, 2);
    }
    else {
        correct = false;
    }
    Write(0xA6, "", 0);
    correct = Read() == 0;

    EXPECT_TRUE(correct);
}

TEST_F(Test_Pca9685_Mg996r, Push_Forward_And_Backward_Many_Times) {
    bool correct = true;

    std::string config_string = R"({
          "configuration": [
            {
              "patient_library": [ "C:\\Users\\rosaj\\Source\\Repos\\ZeroMate\\peripherals\\cgm\\src\\patient\\simple-patient.dll" ],
              "error_chance": [ 0 ]
            }
          ]
        })";

    std::ofstream("peripherals/pca9685_mg996r_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_motor = std::make_unique<CPca9685_Mg996r>(
            "TestSensor",
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

    if (m_motor != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_motor.get());

        std::uint16_t steps = 10;

        std::uint8_t high = (steps >> 8) & 0xFF;
        std::uint8_t low = steps & 0xFF;

        char buffer[2];
        buffer[0] = static_cast<char>(high);
        buffer[1] = static_cast<char>(low);

        for (int i = 0; i < 10; ++i) {
            Write(0x81, buffer, 2);
            Write(0xA4, buffer, 2);
            Write(0xA6, "", 0);
            correct = Read() == 0;
        }
    }
    else {
        correct = false;
    }

    EXPECT_TRUE(correct);
}
