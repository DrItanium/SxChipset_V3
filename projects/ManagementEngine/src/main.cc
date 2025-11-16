#include <Arduino.h>
#include <Event.h>
#include <Logic.h>
#include <Wire.h>
#include <EEPROM.h>
#include "ManagementEngineProtocol.h"



constexpr auto CLK1OUT = PIN_PA0;
constexpr auto DTR = PIN_PA1;
constexpr auto DTR_HI16 = PIN_PA2;
constexpr auto READY_OUT = PIN_PA3;
// PIN_PA4
// PIN_PA5
constexpr auto DTR_LO16 = PIN_PA6;
constexpr auto CLK2OUT = PIN_PA7;

// PIN_PB0
// PIN_PB1
// PIN_PB2
// PIN_PB3
// PIN_PB4
// PIN_PB5
// PIN_PB6
// PIN_PB7

constexpr auto READY_IN = PIN_PC0;
// PIN_PC1
// PIN_PC2 -- Wire
// PIN_PC3 -- Wire
// PIN_PC4
// PIN_PC5
// PIN_PC6
// PIN_PC7

constexpr auto ByteEnable0 = PIN_PD1;
constexpr auto ByteEnable1 = PIN_PD2;
constexpr auto BLAST960 = PIN_PD0;
constexpr auto FAIL960 = PIN_PD3;
constexpr auto RESET960 = PIN_PD4;
constexpr auto HOLD960 = PIN_PD5;
constexpr auto LOCK960 = PIN_PD6;
constexpr auto HLDA960 = PIN_PD7;

// PIN_PE0
// PIN_PE1
// PIN_PE2
// PIN_PE3
// PIN_PE4
// PIN_PE5
// PIN_PE6
// PIN_PE7

// PIN_PF0
// PIN_PF1
// PIN_PF2
// PIN_PF3
// PIN_PF4
// PIN_PF5
// PIN_PF6 (input only)

constexpr auto i960_A1 = PIN_PG0;
constexpr auto DEN = PIN_PG1;
constexpr auto DEN_HI16 = PIN_PG2;
constexpr auto DEN_LO16 = PIN_PG3;
// PIN_PG4
// PIN_PG5
// PIN_PG6
// PIN_PG7

// will be updated on startup
uint32_t CLKSpeeds [] {
    0, 0,
};
// make it really easy to transmit the serial number cache as fast as possible
uint8_t serialNumberCache[16] = { 0 };
uint8_t devidCache[3] = { 0 };
long rSeed = 0;
bool chipIsUp = false;

void
configurePins() noexcept {
    pinMode(HLDA960, INPUT);
    pinMode(HOLD960, OUTPUT);
    digitalWrite(HOLD960, LOW);
    pinMode(LOCK960, INPUT); // this is open drain
    pinMode(RESET960, OUTPUT);
    digitalWrite(RESET960, LOW);
    pinMode(CLK1OUT, OUTPUT);
    pinMode(CLK2OUT, OUTPUT);
    pinMode(READY_IN, INPUT);
    pinMode(READY_OUT, OUTPUT);
    pinMode(ByteEnable0, INPUT_PULLUP);
    pinMode(ByteEnable1, INPUT_PULLUP);
    pinMode(BLAST960, INPUT_PULLUP);
    pinMode(FAIL960, OUTPUT);
    pinMode(i960_A1, INPUT_PULLUP);
    pinMode(DEN, INPUT_PULLUP);
    pinMode(DTR, INPUT_PULLUP);
    pinMode(DEN_HI16, OUTPUT);
    pinMode(DEN_LO16, OUTPUT);
    pinMode(DTR_LO16, OUTPUT);
    pinMode(DTR_HI16, OUTPUT);
}
uint8_t isBusHeld() noexcept { return digitalRead(HLDA960) == HIGH ? 0xFF : 0x00; }
uint8_t isBusLocked() noexcept { return digitalRead(LOCK960) == LOW ? 0xFF : 0x00; }
void
setupSystemClocks() noexcept {
    // Here is what we are doing:
    // MCLKCTRLA:
    // - Enable the CLKOUT function on PA7
    // - Main clock will use the internal high-frequency oscillator
    //
    // OSCHFCTRLA:
    // - Make sure that the high frequency oscillator is always running
    // - Set the frequency of the internal high-frequency oscillator to 24MHz
    // -- NOTE: This can be changed to 20MHz if needed
    //
    // When it comes to using the internal oscillator, there is no question
    // about its performance. Just like with the older ATMEGA 0 series (4809,
    // 4808, etc) the internal frequency oscillator is very accurate and
    // useful. I have confirmed this from reading other accounts (DxCore) and
    // my own observations using one for the core clock of my ATMEGA2560/4808
    // based i960 chipset. The 4808 emits a 20MHz clock from its internal
    // oscillator that drives the clocks of the i960, 2560, and any support
    // hardware. 
  
    // configure the CLKOUT function and which oscillator is used for Main
    // Clock
    CCP = 0xD8; 
    CLKCTRL.MCLKCTRLA = CLKCTRL_CLKOUT_bm;
    asm volatile("nop"); // then wait one cycle
#if (F_CPU == 24000000L) 
    // configure the high frequency oscillator
    CCP = 0xD8; 
    CLKCTRL.OSCHFCTRLA = CLKCTRL_RUNSTDBY_bm | CLKCTRL_FREQSEL_24M_gc;
    asm volatile("nop");
#endif

    // configure the 32khz oscillator to run in standby
    CCP = 0xD8;
    CLKCTRL.OSC32KCTRLA = CLKCTRL_RUNSTDBY_bm;
    asm volatile("nop");
}
bool genericBooleanFunction(bool in0, bool in1, bool in2) noexcept;
using BooleanFunctionBody = decltype(genericBooleanFunction);
constexpr bool isFailSignal(bool be0, bool be1, bool blast) noexcept {
    return (!blast) && (be0 && be1);
}
constexpr bool generateOscillatingPattern(bool low, bool /* middle */, bool /* high */) noexcept {
        return !low;
}
static_assert(!generateOscillatingPattern(true, false, true));
static_assert(generateOscillatingPattern(false, false, true)); // this should be true
constexpr uint8_t computeCCLValueBasedOffOfFunction(BooleanFunctionBody fn) noexcept {
    uint8_t result = 0;
    for (int i = 0; i < 8; ++i) {
        if (fn(i & 0b001, 
               i & 0b010,
               i & 0b100)) {
            result |= (1 << i);
        }
    }
    return result;
}
static_assert(computeCCLValueBasedOffOfFunction(isFailSignal) == 0b0000'1000);
static_assert(computeCCLValueBasedOffOfFunction(generateOscillatingPattern) == 0b01010101);
template<bool useOutputPin = false>
void
configureDivideByTwoCCL(Logic& even, Logic& odd) {
  constexpr uint8_t truthTableValue = computeCCLValueBasedOffOfFunction(generateOscillatingPattern);
  constexpr auto outSrc = useOutputPin ? out::enable : out::disable;
  even.enable = true;
  even.input0 = in::feedback;
  even.input1 = in::disable;
  even.input2 = in::event_a;
  even.clocksource = clocksource::in2; 
  even.output = outSrc;
  even.truth = truthTableValue;
  even.sequencer = sequencer::jk_flip_flop;
  odd.enable = true;
  odd.input0 = in::event_a;
  odd.input1 = in::disable;
  odd.input2 = in::event_a;
  odd.output = out::disable;
  odd.sequencer = sequencer::disable;
  odd.clocksource = clocksource::in2;
  odd.truth = truthTableValue;
  even.init();
  odd.init();
}
void
configureFailCircuit(Logic& even, Logic& odd) {
    // BE0 is high, BE1 is high, and BLAST is LOW for at least one CLK cycle
    // 0b000 -> 0
    // 0b001 -> 0
    // 0b010 -> 0
    // 0b011 -> 0
    // 0b100 -> 0
    // 0b101 -> 0
    // 0b110 -> 1
    // 0b111 -> 0
    // so 0b0100'0000
  constexpr uint8_t failCircuitTruthTable = computeCCLValueBasedOffOfFunction(isFailSignal);
  even.enable = true;
  even.input0 = in::input_pullup;
  even.input1 = in::input_pullup;
  even.input2 = in::input_pullup;
  even.clocksource = clocksource::clk_per; 
  even.output = out::enable;
  even.truth = failCircuitTruthTable;
  even.sequencer = sequencer::d_flip_flop;
  odd.enable = true;
  odd.input0 = in::event_a; // x
  odd.input1 = in::disable; // y
  odd.input2 = in::event_a; // z
  odd.output = out::disable;
  odd.sequencer = sequencer::disable;
  odd.clocksource = clocksource::in2;
  odd.truth = computeCCLValueBasedOffOfFunction(generateOscillatingPattern);
  odd.init();
  even.init();
}
void
configureDTRLogic(Logic& dtrHandler) noexcept {
    dtrHandler.enable = true;
    dtrHandler.input0 = in::disable;
    dtrHandler.input1 = in::input_pullup;
    dtrHandler.input2 = in::disable;
    dtrHandler.clocksource = clocksource::clk_per;
    dtrHandler.output = out::enable;
    dtrHandler.sequencer = sequencer::disable;
    dtrHandler.output_swap = out::pin_swap;
    dtrHandler.truth = computeCCLValueBasedOffOfFunction([](bool, bool dtr, bool) { return dtr; });
    dtrHandler.init();
}
void
configureDENLogic(Logic& lo16, Logic& hi16) noexcept {
    lo16.enable = true;
    lo16.input0 = in::input_pullup;
    lo16.input1 = in::input_pullup;
    lo16.input2 = in::disable;
    lo16.clocksource = clocksource::clk_per;
    lo16.output = out::enable;
    lo16.sequencer = sequencer::disable;
    lo16.truth = computeCCLValueBasedOffOfFunction([](bool a1, bool den, bool) { return !((!a1) && (!den)); });
    
    hi16.enable = true;
    hi16.input0 = in::event_a;
    hi16.input1 = in::event_b;
    hi16.input2 = in::disable;
    hi16.clocksource = clocksource::clk_per;
    hi16.output = out::disable;
    hi16.sequencer = sequencer::disable;
    hi16.truth = computeCCLValueBasedOffOfFunction([](bool a1, bool den, bool) { return !((a1) && (!den)); });
    hi16.init();
    lo16.init();
}
void
updateClockFrequency(uint32_t frequency) noexcept {
    CLKSpeeds[0] = frequency;
    CLKSpeeds[1] = frequency / 2;
}
void 
configureCCLs() {
  
  // PA0 uses TCA0 to generate a 12MHz clock source
  Event0.set_generator(gen0::pin_pa0);
  Event0.set_user(user::tcb1_cnt); // route it to TCB0 as clock source
  Event0.set_user(user::ccl3_event_a); // route it to CCL3 for the fail circuit
                                       
  Event1.set_generator(gen1::pin_pa3); // take from TCB1's single shot
  Event1.set_user(user::evoutc_pin_pc7);
  // PA5 is the ready signal from the teensy
  Event2.set_generator(gen2::pin_pc0); // use PC0 as the input 
  Event2.set_user(user::tcb1_capt); // use PA5 as the trigger source for the
                                    // single shot timer in TCB0
  // Event3

  // Event4

  // Event5
  // route A1 into the EVSYS
  Event6.set_generator(gen7::pin_pg0);
  Event6.set_user(user::ccl4_event_a);
  // route DEN into the EVSYS
  Event7.set_generator(gen7::pin_pg1);
  Event7.set_user(user::ccl4_event_b);
  // DEN_HI16 output handler
  Event8.set_generator(gen::ccl4_out);
  Event8.set_user(user::evoutg_pin_pg2);
  // configure Event9 to do the DTR handler
  Event9.set_generator(gen::ccl0_out);
  Event9.set_user(user::evouta_pin_pa2);

  updateClockFrequency(F_CPU); // we are making CLK2 run at 24MHz
  configureDTRLogic(Logic0);
  configureFailCircuit(Logic2, Logic3);
  configureDENLogic(Logic5, Logic4);

  // create a divide by two clock generator
  PORTMUX.TCAROUTEA = (PORTMUX.TCAROUTEA & ~(PORTMUX_TCA0_gm)) | PORTMUX_TCA0_PORTA_gc; // enable TCA0 on PORTA
  TCA0.SINGLE.CMP0 = 0; // 12MHz compare

  TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm |  // cmp0 enable
      TCA_SINGLE_WGMODE_FRQ_gc;  // frequency

  // okay, so start configuring the secondary single shot setup
  PORTMUX.TCAROUTEA = (PORTMUX.TCAROUTEA & (~PORTMUX_TCB1_bm)) | PORTMUX_TCB1_bm; // enable output on PA3
  TCB1.CCMP = 1; // two cycles
  TCB1.CNT = 1; // 
  TCB1.EVCTRL = TCB_CAPTEI_bm | TCB_EDGE_bm; // enable EVSYS input
  TCB1.CTRLB = TCB_CNTMODE_SINGLE_gc | // enable single shot mode
               TCB_CCMPEN_bm; // enable output via GPIO
  TCB1.CTRLA = TCB_RUNSTDBY_bm | // run in standby
               TCB_ENABLE_bm | // enable
               TCB_CLKSEL_EVENT_gc; // clock comes from EVSYS
  TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm | // turn the device on
      TCA_SINGLE_RUNSTDBY_bm; // run in standby

  PORTA.PIN3CTRL |= PORT_INVEN_bm; // invert the pulse automatically
  PORTC.PIN0CTRL |= PORT_INVEN_bm; // make the input inverted for simplicity
  PORTC.PIN7CTRL |= PORT_INVEN_bm; // output the invr
  Event0.start();
  Event1.start();
  Event2.start();
  Event6.start();
  Event7.start();
  Event8.start();
  Event9.start();
  // make sure that power 
  CCL.CTRLA |= CCL_RUNSTDBY_bm; // run ccl in standby
  Logic::start();
}
void onReceiveHandler(int count);
void onRequestHandler();
void 
setupRandomSeed() noexcept {
    for (auto a : serialNumberCache) {
        rSeed += a;
    }
    for (auto a : devidCache) {
        rSeed += a;
    }
    rSeed += analogRead(A1);
    rSeed += analogRead(A2);
    rSeed += analogRead(A3);
    rSeed += analogRead(A4);
    rSeed += analogRead(A5);
    rSeed += analogRead(A6);
    rSeed += analogRead(A7);
    rSeed += analogRead(A16);
    rSeed += analogRead(A17);
    rSeed += SIGROW.TEMPSENSE0;
    rSeed += SIGROW.TEMPSENSE1;
}
void
setup() {
    Wire.swap(2); // it is supposed to be PC2/PC3 TWI ms
    Wire.begin(0x08);
    Wire.onReceive(onReceiveHandler);
    Wire.onRequest(onRequestHandler);
#define X(index) serialNumberCache[ index ] = SIGROW.SERNUM ## index 
    X(0);
    X(1);
    X(2);
    X(3);
    X(4);
    X(5);
    X(6);
    X(7);
    X(8);
    X(9);
    X(10);
    X(11);
    X(12);
    X(13);
    X(14);
    X(15);
#undef X
#define X(index) devidCache[ index ] = SIGROW.DEVICEID ## index 
    X(0);
    X(1);
    X(2);
#undef X
    setupRandomSeed();
    randomSeed(rSeed);
    EEPROM.begin();
    setupSystemClocks();
    takeOverTCA0();
    configurePins();
    configureCCLs();
    chipIsUp = true;
}
void
loop() {
    delay(10);
}

volatile ManagementEngineRequestOpcode currentMode = ManagementEngineRequestOpcode::CPUClockConfiguration;
void
sinkWire() {
    if (Wire.available()) {
        while (1 < Wire.available()) {
            (void)Wire.read();
        }
        (void)Wire.read();
    }
}
void
onReceiveHandler(int howMany) {
    if (howMany >= 1) {
        switch (static_cast<ManagementEngineReceiveOpcode>(Wire.read())) {
            case ManagementEngineReceiveOpcode::SetMode: 
                if (auto currentOpcode = static_cast<ManagementEngineRequestOpcode>(Wire.read()); valid(currentOpcode)) {
                    currentMode = currentOpcode;
                }
                break;
            case ManagementEngineReceiveOpcode::PutInReset: 
                digitalWrite(RESET960, LOW); 
                break;
            case ManagementEngineReceiveOpcode::PullOutOfReset: 
                digitalWrite(RESET960, HIGH); 
                break;
            case ManagementEngineReceiveOpcode::HoldBus:
                digitalWrite(HOLD960, HIGH);
                break;
            case ManagementEngineReceiveOpcode::ReleaseBus:
                digitalWrite(HOLD960, LOW);
                break;
            default:
                break;
        }
    }
    sinkWire();
}

void
onRequestHandler() {
    switch (currentMode) {
        case ManagementEngineRequestOpcode::CPUClockConfiguration:
            Wire.write(reinterpret_cast<char*>(CLKSpeeds), sizeof(CLKSpeeds));
            break;
        case ManagementEngineRequestOpcode::BusIsHeld: 
            Wire.write(isBusHeld()); 
            break;
        case ManagementEngineRequestOpcode::BusIsLocked: 
            Wire.write(isBusLocked()); 
            break;
        case ManagementEngineRequestOpcode::RevID:
            Wire.write(SYSCFG.REVID);
            break;
        case ManagementEngineRequestOpcode::SerialNumber:
            Wire.write(serialNumberCache, sizeof(serialNumberCache));
            break;
        case ManagementEngineRequestOpcode::DevID:
            Wire.write(devidCache, sizeof(devidCache));
            break;
        case ManagementEngineRequestOpcode::RandomSeed:
            Wire.write(reinterpret_cast<char*>(&rSeed), sizeof(rSeed));
            break;
        case ManagementEngineRequestOpcode::ChipIsReady:
            Wire.write(chipIsUp ? 0xFF : 0x00);
            break;
        default:
            break;

    }
}
