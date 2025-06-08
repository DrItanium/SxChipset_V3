#include <Arduino.h>
#include <Event.h>
#include <Logic.h>
#include <Wire.h>
#include "ManagementEngineProtocol.h"
constexpr auto CLKOUT = PIN_PA7;

constexpr auto CLK1Out = PIN_PB3;
constexpr auto SystemUp = PIN_PB6;

constexpr auto RP2350_READY_IN = PIN_PC0;
constexpr auto RP2350_READY_SYNC = PIN_PC6;

constexpr auto CLK2Out = PIN_PD3;
constexpr auto i960_READY_SYNC = PIN_PD7;

constexpr auto RESET960 = PIN_PG4;
constexpr auto HOLD960 = PIN_PG5;
constexpr auto LOCK960 = PIN_PG6;
constexpr auto HLDA960 = PIN_PG7;

// will be updated on startup
uint32_t CLKSpeeds [] {
    0, 0,
};

void
configurePins() noexcept {
    pinMode(HLDA960, INPUT);
    pinMode(HOLD960, OUTPUT);
    digitalWrite(HOLD960, LOW);
    pinMode(LOCK960, INPUT); // this is open drain
    pinMode(RESET960, OUTPUT);
    digitalWrite(RESET960, LOW);
    pinMode(i960_READY_SYNC, OUTPUT);  
    pinMode(CLKOUT, OUTPUT);
    pinMode(CLK1Out, OUTPUT);
    pinMode(CLK2Out, OUTPUT);
    pinMode(RP2350_READY_IN, INPUT);
    pinMode(RP2350_READY_SYNC, OUTPUT);
    digitalWrite(i960_READY_SYNC, HIGH);
}
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

    // configure the high frequency oscillator
    CCP = 0xD8; 
    CLKCTRL.OSCHFCTRLA = 0b1'0'1001'0'0; 
    asm volatile("nop");

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
configureReadySynchronizer() noexcept {
  // ready signal detector
  //
  // The idea is that this piece of hardware generates a low pulse for exactly one CLK1 cycle
  // to tell the i960 that data can be read/write off of the bus.
  //
  // We send the pulse to two places: the teensy (for synchronization) and to
  // the i960 (as expected). 
  //
  Logic1.enable = true;
  Logic1.input0 = in::pin; // wait for the signal on PC0
  Logic1.input1 = in::disable;
  Logic1.input2 = in::event_a; // CLK1 source
  Logic1.output = out::enable; // enable pin output
  Logic1.output_swap = out::pin_swap; // we want to use PC6 since PC3 is
                                      // reserved for the Wire interface
  Logic1.clocksource = clocksource::in2; // use input2 for the clock source
  Logic1.edgedetect = edgedetect::enable; // we want the edge detector hardware
  Logic1.sequencer = sequencer::disable; // not using any sequencer
  // this synchronizer does introduce some delay but the upshot is that I can
  // reduce the number of chips on the board. If that becomes a problem (which
  // I do not think so) then I will handle then.
  //
  // Ideally, these extra delay cycles are the perfect time to get the teensy
  // ready for the next part of the transaction.
  Logic1.filter = filter::synch; 
  Logic1.truth = 0b1111'0101; // falling edge detector that generates a high
                              // pulse
  Logic1.init();
  // invert the output pin to make it output the correct pulse shape :). 
  // This is done to make it easier to reason about how the edge detector
  // works. It actually generates a high pulse but through the use of INVEN it
  // will generate a low pulse that the i960 expects
  PORTC.PIN6CTRL |= PORT_INVEN_bm;
  PORTD.PIN7CTRL |= PORT_INVEN_bm; 

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

  Event0.set_generator(gen0::pin_pa7); // 24/20MHz
  Event0.set_user(user::ccl2_event_a);
  Event0.set_user(user::ccl3_event_a);
  // we redirect the output from the CCL2 to CCL4 and CCL5
  Event1.set_generator(gen::ccl2_out); // 12/10MHz
  Event1.set_user(user::ccl4_event_a);
  Event1.set_user(user::ccl5_event_a);
  // event 2 and event 3 are used for the ready signal detector
  Event2.set_generator(gen::ccl4_out); // 6/5MHz
  Event2.set_user(user::ccl1_event_a); // we want to use this as the source of
                                       // our clock signal for the signal
                                       // detector

  // redirect the output to PD7 for the purpose of sending it off
  // Right now, it is a duplication of PC6's output since the AVR128DB64 is
  // actually running completely at 3.3v. However, I have the flexibility to
  // move the AVR128DB64 into the i960's 5V domain and not need extra
  // conversion chips. 
  Event3.set_generator(gen::ccl1_out);
  Event3.set_user(user::evoutd_pin_pd7); // i960 "5V" Ready transmit


  configureDivideByTwoCCL<true>(Logic2, Logic3); // divide by two
  configureDivideByTwoCCL<true>(Logic4, Logic5); // divide by four
  updateClockFrequency(F_CPU / 2);
  configureReadySynchronizer();

  Event0.start();
  Event1.start();
  Event2.start();
  Event3.start();
  // make sure that power 
  CCL.CTRLA |= CCL_RUNSTDBY_bm;
  Logic::start();

 
}
void onReceiveHandler(int count);
void onRequestHandler();
void
setup() {
    pinMode(SystemUp, OUTPUT);
    digitalWrite(SystemUp, LOW);
    setupSystemClocks();
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
        default:
            break;

    }
}
