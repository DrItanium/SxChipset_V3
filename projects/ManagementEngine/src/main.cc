#include <Arduino.h>
#include <Event.h>
#include <Logic.h>
#include <Wire.h>
#include <EEPROM.h>
#include "ManagementEngineProtocol.h"



constexpr auto CLK2Out = PIN_PA0;
// PIN_PA1
constexpr auto READY_OUT = PIN_PA2;
constexpr auto CLK1Out = PIN_PA3;
constexpr auto SystemUp = PIN_PA4;
constexpr auto READY_IN = PIN_PA5;
// PIN_PA6
constexpr auto CLKOUT = PIN_PA7;

// PIN_PC0
// PIN_PC1
// PIN_PC2 -- Wire
// PIN_PC3 -- Wire

// PIN_PD1
// PIN_PD2
// PIN_PD3
constexpr auto RESET960 = PIN_PD4;
constexpr auto HOLD960 = PIN_PD5;
constexpr auto LOCK960 = PIN_PD6;
constexpr auto HLDA960 = PIN_PD7;

// PIN_PF0
// PIN_PF1

// will be updated on startup
uint32_t CLKSpeeds [] {
    0, 0,
};
// make it really easy to transmit the serial number cache as fast as possible
uint8_t serialNumberCache[16] = { 0 };
uint8_t devidCache[3] = { 0 };
long rSeed = 0;

void
configurePins() noexcept {
    pinMode(HLDA960, INPUT);
    pinMode(HOLD960, OUTPUT);
    digitalWrite(HOLD960, LOW);
    pinMode(LOCK960, INPUT); // this is open drain
    pinMode(RESET960, OUTPUT);
    digitalWrite(RESET960, LOW);
    pinMode(CLKOUT, OUTPUT);
    pinMode(CLK1Out, OUTPUT);
    pinMode(CLK2Out, OUTPUT);
    pinMode(READY_IN, INPUT);
    pinMode(READY_OUT, OUTPUT);
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
    CLKCTRL.MCLKCTRLA = 0b1000'0000; 
    asm volatile("nop"); // then wait one cycle
#if (F_CPU == 24000000L) 
    // configure the high frequency oscillator
    CCP = 0xD8; 
    CLKCTRL.OSCHFCTRLA = 0b1'0'1001'0'0; 
    asm volatile("nop");
#endif

    // configure the 32khz oscillator to run in standby
    CCP = 0xD8;
    CLKCTRL.OSC32KCTRLA = 0b1'0000000;
    asm volatile("nop");
}
template<bool useOutputPin = false>
void
configureDivideByTwoCCL(Logic& even, Logic& odd) {
  constexpr uint8_t truthTableValue = 0b0101'0101;
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
updateClockFrequency(uint32_t frequency) noexcept {
    CLKSpeeds[0] = frequency;
    CLKSpeeds[1] = frequency / 2;
}
void 
configureCCLs() {
  // CCL2 and CCL3 are used to create a divide by two clock signal in the exact
  // way one would implement it using an 74AHC74. We divide the 24/20 MHz
  // signal down to a 12/10MHz signal
  //
  // Then we take the output from that and feed it into a second divide by two
  // circuit using CCLs 4 and 5 to create a divide by 4 circuit. 
  // This will create a 6/5MHz signal
  //
  // I do know that I could either use an external chip or the timers but I
  // find that to not be as easy because it feels like a waste. Those timers
  // and extra board space can be better utilized in the future.
  
  // Redirect the output of PA7 in CLKOUT mode to act as a way to send the
  // clock signal to other sources. This makes the design of the clock dividers
  // far simpler at the cost of wasting an event channel (although it isn't
  // actually a waste).
  //
  // PA0 uses TCA0 to generate a 12MHz clock source
  Event0.set_generator(gen0::pin_pa0);
  Event0.set_user(user::ccl0_event_a); // route it to CCL0
  Event0.set_user(user::ccl1_event_a); // route it to CCL1
  Event0.set_user(user::tcb0_cnt); // route it to TCB0 as clock source

  // PA5 is the ready signal from the teensy
  Event1.set_generator(gen0::pin_pa5); // use PA5 as the input 
  Event1.set_user(user::tcb0_capt);

  Event2.set_generator(gen::ccl0_out);
  //Event2.set_user(user::tcb0_cnt);
  configureDivideByTwoCCL<true>(Logic0, Logic1); // divide by two (v2)
  //updateClockFrequency(F_CPU / 2);
  updateClockFrequency(F_CPU);

  // create a divide by two clock generator
  PORTMUX.TCAROUTEA = (PORTMUX.TCAROUTEA & ~(PORTMUX_TCA0_gm)) | PORTMUX_TCA0_PORTA_gc; // enable TCA0 on PORTA
  TCA0.SINGLE.CMP0 = 0; // 12MHz compare
  TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_FRQ_gc; 

  // okay, so start configuring the secondary single shot setup
  PORTMUX.TCBROUTEA = 0; // output on PA2
  TCB0.CCMP = 1; // two cycles
  TCB0.CNT = 1; // 
  TCB0.EVCTRL = TCB_CAPTEI_bm | TCB_EDGE_bm; // enable EVSYS input
  TCB0.CTRLB = TCB_CNTMODE_SINGLE_gc | // enable single shot mode
               TCB_CCMPEN_bm; // enable output via GPIO
  TCB0.CTRLA = TCB_RUNSTDBY_bm | // run in standby
               TCB_ENABLE_bm | // enable
               TCB_CLKSEL_EVENT_gc; // clock comes from EVSYS
  TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm | TCA_SINGLE_RUNSTDBY_bm;

  PORTA.PIN2CTRL |= PORT_INVEN_bm; // make a low pulse
  PORTA.PIN5CTRL |= PORT_INVEN_bm; // make it inverted
  Event0.start();
  Event1.start();
  Event2.start();
  // make sure that power 
  CCL.CTRLA |= CCL_RUNSTDBY_bm;
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
    pinMode(SystemUp, OUTPUT);
    digitalWrite(SystemUp, LOW);
    setupSystemClocks();
    takeOverTCA0();
    configurePins();
    configureCCLs();
    Wire.swap(2); // it is supposed to be PC2/PC3 TWI ms
    Wire.begin(0x08);
    Wire.onReceive(onReceiveHandler);
    Wire.onRequest(onRequestHandler);
    digitalWrite(SystemUp, HIGH);
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
        default:
            break;

    }
}
