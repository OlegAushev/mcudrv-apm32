#pragma once


#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include <mcudrv/apm32/apm32_base.h>
#include <mcudrv/apm32/f4/system/system.h>
#include <mcudrv/apm32/f4/gpio/gpio.h>
#include <apm32f4xx_spi.h>
#include <initializer_list>
#include <optional>
#include <vector>


namespace mcu {
namespace spi {


constexpr size_t peripheral_count = 3;
enum class Peripheral : unsigned int {
    spi1,
    spi2,
    spi3
};


enum class Direction { rx, tx };


struct MosiPinConfig { GPIO_T* port; uint16_t pin; GPIO_AF_T altfunc; };
struct MisoPinConfig { GPIO_T* port; uint16_t pin; GPIO_AF_T altfunc; };
struct ClkPinConfig { GPIO_T* port; uint16_t pin; GPIO_AF_T altfunc; };
struct HwCsPinConfig { GPIO_T* port; uint16_t pin; GPIO_AF_T altfunc; };
struct SwCsPinConfig { GPIO_T* port; uint16_t pin; };


struct Config {
    SPI_Config_T hal_config;
};


namespace impl {


inline const std::array<SPI_T*, peripheral_count> instances = {SPI1, SPI2, SPI3};


inline Peripheral to_peripheral(const SPI_T* instance) {
    return static_cast<Peripheral>(
        std::distance(instances.begin(), std::find(instances.begin(), instances.end(), instance)));
}


inline std::array<void(*)(void), peripheral_count> clk_enable_funcs = {
    [](){ RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_SPI1); },
    [](){ RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_SPI2); },
    [](){ RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_SPI3); },
};


} // namespace impl


class Module : public emb::interrupt_invoker_array<Module, peripheral_count>, private emb::noncopyable {
private:
    const Peripheral _peripheral;
    SPI_T* const _reg;
    mcu::gpio::AlternatePin _mosi_pin;
    mcu::gpio::AlternatePin _miso_pin;
    mcu::gpio::AlternatePin _clk_pin;
    mcu::gpio::AlternatePin _cs_pin;
    std::vector<mcu::gpio::OutputPin> _cs_pins;

    static inline std::array<bool, peripheral_count> _clk_enabled{};
public:
    Module(Peripheral peripheral,
           const MosiPinConfig& mosi_pin_config, const MisoPinConfig& miso_pin_config,
           const ClkPinConfig& clk_pin_config, const HwCsPinConfig& cs_pin_config,
           const Config& config);

    Module(Peripheral peripheral,
           const MosiPinConfig& mosi_pin_config, const MisoPinConfig& miso_pin_config,
           const ClkPinConfig& clk_pin_config, std::initializer_list<SwCsPinConfig> cs_pin_configs,
           const Config& config);

    Peripheral peripheral() const { return _peripheral; }
    SPI_T* reg() { return _reg; }

    static Module* instance(Peripheral peripheral) {
        return emb::interrupt_invoker_array<Module, peripheral_count>::instance(std::to_underlying(peripheral));
    }

    void enable() { _reg->CTRL1_B.SPIEN = 1; }
    void disable() { _reg->CTRL1_B.SPIEN = 0; }

    bool busy() const { return _reg->STS_B.BSYFLG == 1; }
    bool rx_empty() const { return _reg->STS_B.RXBNEFLG == 0; }
    bool tx_empty() const { return _reg->STS_B.TXBEFLG == 1; }

    DrvStatus put_data(uint16_t data) {
        if (_reg->STS_B.TXBEFLG == 0) {
            return DrvStatus::busy;
        }
        _reg->DATA_B.DATA = data;
        return DrvStatus::ok;
    }

    std::optional<uint16_t> get_data() {
        if (_reg->STS_B.RXBNEFLG == 0) {
            return {};
        }
        uint16_t data = _reg->DATA_B.DATA;
        return {data};
    }

    void set_bidirectional_mode(Direction dir) {
        switch (dir) {
            case Direction::rx:
                _reg->CTRL1_B.BMOEN = 0;
                break;
            case Direction::tx:
                _reg->CTRL1_B.BMOEN = 1;
                break;
        }
    }

    void set_cs(size_t cs_idx = 0) {
        if (cs_idx >= _cs_pins.size()) {
            return;
        }
        _cs_pins[cs_idx].set();
    }

    void reset_cs(size_t cs_idx = 0) {
        if (cs_idx >= _cs_pins.size()) {
            return;
        }
        _cs_pins[cs_idx].reset();
    }
protected:
    static void _enable_clk(Peripheral peripheral);
    void _init_mosi_miso_clk(const MosiPinConfig& mosi_pin_config,
                             const MisoPinConfig& miso_pin_config,
                             const ClkPinConfig& clk_pin_config);
};


} // namespace spi
} // namespace mcu


#endif
#endif
