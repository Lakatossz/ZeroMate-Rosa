#include "gtest/gtest.h"
#include <filesystem>
#include <array>
#include <memory>
#include <chrono>
#include <thread>
#include <fstream>
#include <iostream>
#include <ctime>
#include <unordered_map>
#include "zero_mate/external_peripheral.hpp"
#include "zero_mate/utils/singleton.hpp"
#include "../src/ds3231.hpp"
#include "../../../src/core/peripherals/igpio.hpp"
#include "../../../src/mock/mock_gpio/mock_gpio.hpp"
#include "../../../src/mock/mock_bsc/mock_bsc.hpp"
#include "../../../src/core/peripherals/peripheral.hpp"
#include "../../../src/core/peripherals/ibsc.hpp"

using json = nlohmann::json;
using namespace zero_mate;
namespace fs = std::filesystem;

class Test_DS3231 : public ::testing::Test {
private:
    static inline Test_DS3231* current_instance{ nullptr };

public:
    Test_DS3231() {
        current_instance = this; // Správné nastavení instance pro pøístup k èlenským funkcím
    }

    std::uint32_t Read(char *buffer, std::uint32_t address) {
        char s;

        std::uint32_t read_address = address | 1;

        // Nastaveni I2C adresy senzoru v BSC
        m_bsc->Write(static_cast<uint32_t>(peripheral::IBSC::NRegister::Slave_Address),
            reinterpret_cast<const char*>(&read_address), sizeof(uint32_t));

        // Nastaveni delky dat, ktere budu cist v BSC
        std::uint32_t size = 7;
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
        for (int i = 0; i < size; ++i) {
            m_bsc->Read(static_cast<uint32_t>(peripheral::IBSC::NRegister::Data_FIFO),
                &buffer[i], sizeof(uint8_t));
        }

        control_reg_value = (1 << 4);
        data_to_write = reinterpret_cast<char*>(&control_reg_value);

        // Vyprazdneni FIFO
        m_bsc->Write(static_cast<uint32_t>(peripheral::IBSC::NRegister::Control),
            reinterpret_cast<const char*>(data_to_write), sizeof(uint32_t));

        return size;
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
        m_rtc.reset();
        m_bsc.release();
        fs::remove_all("peripherals");
    }

    std::shared_ptr<zero_mate::peripheral::CGPIO_Manager_Mock> m_gpio;
    std::unique_ptr<peripheral::Mock_BSC> m_bsc;
    std::unique_ptr<CDS3231> m_rtc;
    std::unique_ptr<CDS3231> m_rtc_1;
    std::unique_ptr<CDS3231> m_rtc_2;
    std::unique_ptr<CDS3231> m_rtc_3;
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
TEST_F(Test_DS3231, Basic_Initialization) {
    bool correct = true;

    std::string config_string = R"({
      "configuration": [
        {
          "probability": [ 0 ]
        }
      ]
    })";

    std::ofstream("peripherals/ds3231_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_rtc = std::make_unique<CDS3231>(
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

    EXPECT_TRUE(correct);
}

TEST_F(Test_DS3231, Read_Time_Once) {
    bool correct = true;

    std::string config_string = R"({
      "configuration": [
        {
          "probability": [ 0 ]
        }
      ]
    })";

    std::ofstream("peripherals/ds3231_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_rtc = std::make_unique<CDS3231>(
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

    if (m_rtc != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_rtc.get());

        char buffer[7] = {};
        Write(0x81, m_address);
        std::uint32_t size = Read(buffer, m_address);

        if (size == 7) {
            std::time_t now = std::time(nullptr);
            std::tm local_tm{};

            if (localtime_s(&local_tm, &now) != 0) {
            }

            correct = static_cast<std::uint8_t>(local_tm.tm_sec) == static_cast<std::uint8_t>(buffer[0]) &&
                static_cast<std::uint8_t>(local_tm.tm_min) == static_cast<std::uint8_t>(buffer[1]) &&
                static_cast<std::uint8_t>(local_tm.tm_hour) == static_cast<std::uint8_t>(buffer[2]) &&
                static_cast<std::uint8_t>(local_tm.tm_mday) == static_cast<std::uint8_t>(buffer[3]) &&
                static_cast<std::uint8_t>(local_tm.tm_wday) == static_cast<std::uint8_t>(buffer[4]) &&
                static_cast<std::uint8_t>(local_tm.tm_mon) == static_cast<std::uint8_t>(buffer[5]) &&
                static_cast<std::uint8_t>(local_tm.tm_year) == static_cast<std::uint8_t>(buffer[6]);
        }
        else {
            correct = false;
        }
    }
    else {
        correct = false;
    }

    EXPECT_TRUE(correct);
}

TEST_F(Test_DS3231, Read_Time_Many_Times) {
    bool correct = true;

    std::string config_string = R"({
      "configuration": [
        {
          "probability": [ 0 ]
        }
      ]
    })";

    std::ofstream("peripherals/ds3231_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_rtc = std::make_unique<CDS3231>(
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

    if (m_rtc != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_rtc.get());

        char buffer[7] = {};
        std::uint32_t size;

        for (int i = 0; i < 10; ++i) {
            Write(0x81, m_address);
            size = Read(buffer, m_address);

            if (size == 7) {
                std::time_t now = std::time(nullptr);
                std::tm local_tm{};

                if (localtime_s(&local_tm, &now) != 0) {
                }

                correct = static_cast<std::uint8_t>(local_tm.tm_sec) == static_cast<std::uint8_t>(buffer[0]) &&
                    static_cast<std::uint8_t>(local_tm.tm_min) == static_cast<std::uint8_t>(buffer[1]) &&
                    static_cast<std::uint8_t>(local_tm.tm_hour) == static_cast<std::uint8_t>(buffer[2]) &&
                    static_cast<std::uint8_t>(local_tm.tm_mday) == static_cast<std::uint8_t>(buffer[3]) &&
                    static_cast<std::uint8_t>(local_tm.tm_wday) == static_cast<std::uint8_t>(buffer[4]) &&
                    static_cast<std::uint8_t>(local_tm.tm_mon) == static_cast<std::uint8_t>(buffer[5]) &&
                    static_cast<std::uint8_t>(local_tm.tm_year) == static_cast<std::uint8_t>(buffer[6]);
            }
            else {
                correct = false;
            }
        }
    }
    else {
        correct = false;
    }

    EXPECT_TRUE(correct);
}

TEST_F(Test_DS3231, Read_Time_Once_With_Error) {
    bool correct = true;

    std::string config_string = R"({
      "configuration": [
        {
          "probability": [ 100 ]
        }
      ]
    })";

    std::ofstream("peripherals/ds3231_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_rtc = std::make_unique<CDS3231>(
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

    if (m_rtc != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_rtc.get());

        char buffer[7] = {};
        Write(0x81, m_address);
        std::uint32_t size = Read(buffer, m_address);

        if (size == 7) {
            std::time_t now = std::time(nullptr);
            std::tm local_tm{};

            if (localtime_s(&local_tm, &now) != 0) {
            }

            correct = static_cast<std::uint8_t>(local_tm.tm_sec) == static_cast<std::uint8_t>(buffer[0]) &&
                static_cast<std::uint8_t>(local_tm.tm_min) == static_cast<std::uint8_t>(buffer[1]) &&
                static_cast<std::uint8_t>(local_tm.tm_hour) == static_cast<std::uint8_t>(buffer[2]) &&
                static_cast<std::uint8_t>(local_tm.tm_mday) == static_cast<std::uint8_t>(buffer[3]) &&
                static_cast<std::uint8_t>(local_tm.tm_wday) == static_cast<std::uint8_t>(buffer[4]) &&
                static_cast<std::uint8_t>(local_tm.tm_mon) == static_cast<std::uint8_t>(buffer[5]) &&
                static_cast<std::uint8_t>(local_tm.tm_year) == static_cast<std::uint8_t>(buffer[6]);
        }
        else {
            correct = false;
        }
    }
    else {
        correct = false;
    }

    EXPECT_FALSE(correct);
}

TEST_F(Test_DS3231, Read_Time_Once_With_Other_I2C_Devices) {
    bool correct = true;

    std::string config_string = R"({
      "configuration": [
        {
          "probability": [ 0 ]
        }
      ]
    })";

    std::ofstream("peripherals/ds3231_config.json") << config_string;

    try {
        // Vytvoøení instance senzoru (I2C slave)
        m_rtc = std::make_unique<CDS3231>(
            "TestSensor",
            m_address,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );

        m_rtc_1 = std::make_unique<CDS3231>(
            "TestSensor1",
            m_address_1,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );

        m_rtc_2 = std::make_unique<CDS3231>(
            "TestSensor2",
            m_address_2,
            m_sda_pin,
            m_scl_pin,
            &m_gpio->Static_Read_GPIO_Pin,
            &m_gpio->Static_Set_GPIO_Pin,
            m_logging_system
        );

        m_rtc_3 = std::make_unique<CDS3231>(
            "TestSensor3",
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

    if (m_rtc != nullptr && m_gpio != nullptr && m_bsc != nullptr) {
        m_gpio->Add_External_Peripheral(m_rtc.get());
        m_gpio->Add_External_Peripheral(m_rtc_1.get());
        m_gpio->Add_External_Peripheral(m_rtc_2.get());
        m_gpio->Add_External_Peripheral(m_rtc_3.get());

        char buffer[7] = {};
        Write(0x81, m_address);
        std::uint32_t size = Read(buffer, m_address);

        if (size == 7) {
            std::time_t now = std::time(nullptr);
            std::tm local_tm{};

            if (localtime_s(&local_tm, &now) != 0) {
            }

            correct = static_cast<std::uint8_t>(local_tm.tm_sec) == static_cast<std::uint8_t>(buffer[0]) &&
                static_cast<std::uint8_t>(local_tm.tm_min) == static_cast<std::uint8_t>(buffer[1]) &&
                static_cast<std::uint8_t>(local_tm.tm_hour) == static_cast<std::uint8_t>(buffer[2]) &&
                static_cast<std::uint8_t>(local_tm.tm_mday) == static_cast<std::uint8_t>(buffer[3]) &&
                static_cast<std::uint8_t>(local_tm.tm_wday) == static_cast<std::uint8_t>(buffer[4]) &&
                static_cast<std::uint8_t>(local_tm.tm_mon) == static_cast<std::uint8_t>(buffer[5]) &&
                static_cast<std::uint8_t>(local_tm.tm_year) == static_cast<std::uint8_t>(buffer[6]);
        }
        else {
            correct = false;
        }

        Write(0x81, m_address_1);
        size = Read(buffer, m_address_1);

        if (size == 7) {
            std::time_t now = std::time(nullptr);
            std::tm local_tm{};

            if (localtime_s(&local_tm, &now) != 0) {
            }

            correct = static_cast<std::uint8_t>(local_tm.tm_sec) == static_cast<std::uint8_t>(buffer[0]) &&
                static_cast<std::uint8_t>(local_tm.tm_min) == static_cast<std::uint8_t>(buffer[1]) &&
                static_cast<std::uint8_t>(local_tm.tm_hour) == static_cast<std::uint8_t>(buffer[2]) &&
                static_cast<std::uint8_t>(local_tm.tm_mday) == static_cast<std::uint8_t>(buffer[3]) &&
                static_cast<std::uint8_t>(local_tm.tm_wday) == static_cast<std::uint8_t>(buffer[4]) &&
                static_cast<std::uint8_t>(local_tm.tm_mon) == static_cast<std::uint8_t>(buffer[5]) &&
                static_cast<std::uint8_t>(local_tm.tm_year) == static_cast<std::uint8_t>(buffer[6]);
        }
        else {
            correct = false;
        }

        Write(0x81, m_address_2);
        size = Read(buffer, m_address_2);

        if (size == 7) {
            std::time_t now = std::time(nullptr);
            std::tm local_tm{};

            if (localtime_s(&local_tm, &now) != 0) {
            }

            correct = static_cast<std::uint8_t>(local_tm.tm_sec) == static_cast<std::uint8_t>(buffer[0]) &&
                static_cast<std::uint8_t>(local_tm.tm_min) == static_cast<std::uint8_t>(buffer[1]) &&
                static_cast<std::uint8_t>(local_tm.tm_hour) == static_cast<std::uint8_t>(buffer[2]) &&
                static_cast<std::uint8_t>(local_tm.tm_mday) == static_cast<std::uint8_t>(buffer[3]) &&
                static_cast<std::uint8_t>(local_tm.tm_wday) == static_cast<std::uint8_t>(buffer[4]) &&
                static_cast<std::uint8_t>(local_tm.tm_mon) == static_cast<std::uint8_t>(buffer[5]) &&
                static_cast<std::uint8_t>(local_tm.tm_year) == static_cast<std::uint8_t>(buffer[6]);
        }
        else {
            correct = false;
        }

        Write(0x81, m_address_3);
        size = Read(buffer, m_address_3);

        if (size == 7) {
            std::time_t now = std::time(nullptr);
            std::tm local_tm{};

            if (localtime_s(&local_tm, &now) != 0) {
            }

            correct = static_cast<std::uint8_t>(local_tm.tm_sec) == static_cast<std::uint8_t>(buffer[0]) &&
                static_cast<std::uint8_t>(local_tm.tm_min) == static_cast<std::uint8_t>(buffer[1]) &&
                static_cast<std::uint8_t>(local_tm.tm_hour) == static_cast<std::uint8_t>(buffer[2]) &&
                static_cast<std::uint8_t>(local_tm.tm_mday) == static_cast<std::uint8_t>(buffer[3]) &&
                static_cast<std::uint8_t>(local_tm.tm_wday) == static_cast<std::uint8_t>(buffer[4]) &&
                static_cast<std::uint8_t>(local_tm.tm_mon) == static_cast<std::uint8_t>(buffer[5]) &&
                static_cast<std::uint8_t>(local_tm.tm_year) == static_cast<std::uint8_t>(buffer[6]);
        }
        else {
            correct = false;
        }
    }
    else {
        correct = false;
    }

    EXPECT_TRUE(correct);
}
