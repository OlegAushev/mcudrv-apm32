#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include "tests.h"


void mcu::tests::gpio_test()
{
#ifdef STM32F446_NUCLEO
    // connect PF3 to PE8 and PF15 to PF10
    mcu::gpio::Config out1cfg = {.port = GPIOF,
                                 .pin = {.Pin = GPIO_PIN_3, .Mode = GPIO_MODE_OUTPUT_PP, .Pull = GPIO_NOPULL, .Speed = GPIO_SPEED_FREQ_LOW, .Alternate = 0},
                                 .actstate = emb::gpio::active_state::high};
    mcu::gpio::Config out2cfg = {.port = GPIOF,
                                 .pin = {.Pin = GPIO_PIN_15, .Mode = GPIO_MODE_OUTPUT_PP, .Pull = GPIO_NOPULL, .Speed = GPIO_SPEED_FREQ_LOW, .Alternate = 0},
                                 .actstate = emb::gpio::active_state::low};

    mcu::gpio::Config in1cfg = {.port = GPIOE,
                                .pin = {.Pin = GPIO_PIN_8, .Mode = MODE_INPUT, .Pull = GPIO_NOPULL, .Speed = GPIO_SPEED_FREQ_LOW, .Alternate = 0},
                                .actstate = emb::gpio::active_state::low};
    mcu::gpio::Config in2cfg = {.port = GPIOF,
                                .pin = {.Pin = GPIO_PIN_10, .Mode = MODE_INPUT, .Pull = GPIO_NOPULL, .Speed = GPIO_SPEED_FREQ_LOW, .Alternate = 0},
                                .actstate = emb::gpio::active_state::high};

    mcu::gpio::Output out1(out1cfg);
    mcu::gpio::Output out2(out2cfg);
    mcu::gpio::Input in1(in1cfg);
    mcu::gpio::Input in2(in2cfg);
    
    out1.reset();
    out2.reset();

    EMB_ASSERT_EQUAL(out1.read(), emb::gpio::state::inactive);
    EMB_ASSERT_EQUAL(in1.read(), emb::gpio::state::active);
    EMB_ASSERT_EQUAL(out2.read(), emb::gpio::state::inactive);
    EMB_ASSERT_EQUAL(in2.read(), emb::gpio::state::active);

    EMB_ASSERT_EQUAL(out1.read_level(), 0);
    EMB_ASSERT_EQUAL(in1.read_level(), 0);
    EMB_ASSERT_EQUAL(out2.read_level(), 1);
    EMB_ASSERT_EQUAL(in2.read_level(), 1);

    out1.set();
    out2.set();

    EMB_ASSERT_EQUAL(out1.read(), emb::gpio::state::active);
    EMB_ASSERT_EQUAL(in1.read(), emb::gpio::state::inactive);
    EMB_ASSERT_EQUAL(out2.read(), emb::gpio::state::active);
    EMB_ASSERT_EQUAL(in2.read(), emb::gpio::state::inactive);

    EMB_ASSERT_EQUAL(out1.read_level(), 1);
    EMB_ASSERT_EQUAL(in1.read_level(), 1);
    EMB_ASSERT_EQUAL(out2.read_level(), 0);
    EMB_ASSERT_EQUAL(in2.read_level(), 0);

    out1.toggle();
    out2.toggle();

    EMB_ASSERT_EQUAL(out1.read(), emb::gpio::state::inactive);
    EMB_ASSERT_EQUAL(in1.read(), emb::gpio::state::active);
    EMB_ASSERT_EQUAL(out2.read(), emb::gpio::state::inactive);
    EMB_ASSERT_EQUAL(in2.read(), emb::gpio::state::active);

    EMB_ASSERT_EQUAL(out1.read_level(), 0);
    EMB_ASSERT_EQUAL(in1.read_level(), 0);
    EMB_ASSERT_EQUAL(out2.read_level(), 1);
    EMB_ASSERT_EQUAL(in2.read_level(), 1);  

    out1.deinit();
    out2.deinit();
    in1.deinit();
    in2.deinit();
#endif
}


#endif
#endif
