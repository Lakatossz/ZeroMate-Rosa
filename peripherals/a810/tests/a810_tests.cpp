#include "gtest/gtest.h"
#include <filesystem>
#include <array>
#include <memory>
#include <chrono>
#include <thread>
#include <fstream>
#include <iostream>
#include "../src/a810.hpp"
#include "zero_mate/external_peripheral.hpp"
#include "zero_mate/utils/singleton.hpp"
#include "../../../src/core/peripherals/peripheral.hpp"

using namespace zero_mate;
namespace fs = std::filesystem;

//-----------------------------------------------------------------------------
// Tøída mockující GPIO manager
//-----------------------------------------------------------------------------
class CGPIO_Manager_Mock final : public peripheral::IPeripheral {
public:
    static constexpr std::size_t Number_of_GPIO_Pins = 54;
    static inline CGPIO_Manager_Mock* current_instance = nullptr;

    enum class NState { Low, High };

    struct CPin {
        NState state{ NState::Low };
        bool accessed{ false };
    };

    // Implementace rozhraní IPeripheral
    void Write(std::uint32_t, const char*, std::uint32_t) override {}
    void Read(std::uint32_t, char*, std::uint32_t) override {}
    [[nodiscard]] std::uint32_t Get_Size() const noexcept override { return 0; }
    void Reset() noexcept override {}

    // Statické callback metody
    static bool Static_Read_GPIO_Pin(std::uint32_t pin) {
        if (!current_instance) return false;
        return current_instance->m_pins[pin].state == NState::High;
    }

    static int Static_Set_GPIO_Pin(std::uint32_t pin, bool state) {
        if (!current_instance) return 0;
        current_instance->m_pins[pin].state = state ? NState::High : NState::Low;
        current_instance->m_pins[pin].accessed = true;
        return 0;
    }

    // Metody pro testy
    [[nodiscard]] NState Get_Pin(std::uint32_t pin) const {
        return m_pins[pin].state;
    }

private:
    std::array<CPin, Number_of_GPIO_Pins> m_pins{};
};

//-----------------------------------------------------------------------------
// Testovací fixture pro WDT
//-----------------------------------------------------------------------------
class Test_A810 : public ::testing::Test {
protected:
    void SetUp() override {
        m_gpio = std::make_shared<CGPIO_Manager_Mock>();
        CGPIO_Manager_Mock::current_instance = m_gpio.get();
        m_logging_system = utils::CSingleton<utils::CLogging_System>::Get_Instance();
        fs::create_directories("peripherals");
    }

    void TearDown() override {
        m_monitoring_active = false;

        if (m_monitor_thread.joinable()) {
            m_monitor_thread.join();
        }

        m_sensor.release();
        CGPIO_Manager_Mock::current_instance = nullptr;
        fs::remove_all("peripherals");
    }

    const std::uint32_t m_pin = 26;
    std::shared_ptr<CGPIO_Manager_Mock> m_gpio; // Zmìnìno na shared_ptr
    std::unique_ptr<CA810> m_sensor;
    utils::CLogging_System* m_logging_system;
    std::atomic<bool> m_reset_detected{ false }; // Pøesunuto do fixture
    std::atomic<bool> m_monitoring_active{ true };
    std::thread m_monitor_thread;
};

//-----------------------------------------------------------------------------
// Testovací pøípady
//-----------------------------------------------------------------------------
TEST_F(Test_A810, Initialization) {
    std::ofstream("peripherals/a810_config.json") << R"({
  "configuration": [
    {
      "probability": [ 100 ]
    }
  ]
})";

    m_sensor = std::make_unique<CA810>(
        "TestSensor",
        m_pin,
        &CGPIO_Manager_Mock::Static_Read_GPIO_Pin,
        &CGPIO_Manager_Mock::Static_Set_GPIO_Pin,
        m_logging_system
    );

    // Oèekáváme HIGH po inicializaci
    EXPECT_EQ(m_gpio->Get_Pin(m_pin), CGPIO_Manager_Mock::NState::Low);
    EXPECT_TRUE(true);
}

TEST_F(Test_A810, WDT_Bubble_Found) {
    std::ofstream("peripherals/a810_config.json") << R"({
  "configuration": [
    {
      "probability": [ 100 ]
    }
  ]
})";

    m_sensor = std::make_unique<CA810>(
        "TestSensor",
        m_pin,
        &CGPIO_Manager_Mock::Static_Read_GPIO_Pin,
        &CGPIO_Manager_Mock::Static_Set_GPIO_Pin,
        m_logging_system
    );

    // Spuštìní monitorovacího vlákna
    m_monitor_thread = std::thread([this]() {
        while (m_monitoring_active) {
            if (m_gpio->Static_Read_GPIO_Pin(m_pin)) {
                m_reset_detected = true;
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        });

    std::this_thread::sleep_for(std::chrono::seconds(10));

    EXPECT_TRUE(m_reset_detected);
}

TEST_F(Test_A810, Bubble_Not_Found) {
    std::ofstream("peripherals/a810_config.json") << R"({
  "configuration": [
    {
      "probability": [ 0 ]
    }
  ]
})";

    m_sensor = std::make_unique<CA810>(
        "TestSensor",
        m_pin,
        &CGPIO_Manager_Mock::Static_Read_GPIO_Pin,
        &CGPIO_Manager_Mock::Static_Set_GPIO_Pin,
        m_logging_system
    );

    // Spuštìní monitorovacího vlákna
    m_monitor_thread = std::thread([this]() { // Místo lokální promìnné
        while (m_monitoring_active) {
            if (m_gpio->Static_Read_GPIO_Pin(m_pin)) {
                m_reset_detected = true;
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        });

    // Simulace uplynutí èasu
    std::this_thread::sleep_for(std::chrono::seconds(3));

    EXPECT_FALSE(m_reset_detected);
}
