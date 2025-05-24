#include <Arduino.h>
#include <Event.h>
#include <Logic.h>
#include <Wire.h>

constexpr auto CLKOUT = PIN_PA7;
constexpr auto CLK1Out = PIN_PB3;
constexpr auto RP2350_READY_IN = PIN_PC0;
constexpr auto RP2350_READY_SYNC = PIN_PC6;
constexpr auto SensorChannel0 = PIN_PD0;
constexpr auto SensorChannel1 = PIN_PD1;
constexpr auto SensorChannel2 = PIN_PD2;
constexpr auto CLK2Out = PIN_PD3;
constexpr auto SensorChannel3 = PIN_PD4;
constexpr auto SensorChannel4 = PIN_PD5;
constexpr auto i960_READY_SYNC = PIN_PD7;
constexpr auto SensorChannel5 = PIN_PE0;
constexpr auto SensorChannel6 = PIN_PE1;
constexpr auto SensorChannel7 = PIN_PE2;

// will be updated on startup
volatile uint32_t CLK2Rate = 0;
volatile uint32_t CLK1Rate = 0;

constexpr auto NumberOfAnalogChannels = 8;
decltype(analogRead(SensorChannel0)) CurrentChannelSamples[NumberOfAnalogChannels] { 0 };
constexpr decltype(SensorChannel0) AnalogChannels[NumberOfAnalogChannels] {
    SensorChannel0,
    SensorChannel1,
    SensorChannel2,
    SensorChannel3,
    SensorChannel4,
    SensorChannel5,
    SensorChannel6,
    SensorChannel7,
};
void
configurePins() noexcept {
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

    // at some point, I want to use the internal 32k oscillator (OSC32K) to
    // help with timer related interrupts.
    /// @todo put code to configure the PLL if needed here
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
    CLK2Rate = frequency;
    CLK1Rate = CLK2Rate / 2;
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
  setupSystemClocks();
  configurePins();
  configureCCLs();
  Wire.swap(2); // it is supposed to be PC2/PC3 TWI ms
  Wire.begin(0x08);
  Wire.onReceive(onReceiveHandler);
  Wire.onRequest(onRequestHandler);
}
void
sampleAnalogChannels() noexcept {
    for (int i = 0; i < NumberOfAnalogChannels; ++i) {
        CurrentChannelSamples[i] = analogRead(AnalogChannels[i]);
    }
}
void
loop() {
    // sample analog channels every second
    sampleAnalogChannels();
    delay(1000);
}

enum class WireReceiveOpcode : uint8_t {
    SetMode,
};
enum class WireRequestOpcode : uint8_t {
    CPUClockConfiguration,
    CPUClockConfiguration_CLK2,
    CPUClockConfiguration_CLK1,
    AnalogSensors,
    AnalogSensors_Ch0,
    AnalogSensors_Ch1,
    AnalogSensors_Ch2,
    AnalogSensors_Ch3,
    AnalogSensors_Ch4,
    AnalogSensors_Ch5,
    AnalogSensors_Ch6,
    AnalogSensors_Ch7,
};
constexpr bool valid(WireRequestOpcode code) noexcept {
    switch (code) {
        case WireRequestOpcode::CPUClockConfiguration:
        case WireRequestOpcode::CPUClockConfiguration_CLK2:
        case WireRequestOpcode::CPUClockConfiguration_CLK1:
        case WireRequestOpcode::AnalogSensors:
        case WireRequestOpcode::AnalogSensors_Ch0:
        case WireRequestOpcode::AnalogSensors_Ch1:
        case WireRequestOpcode::AnalogSensors_Ch2:
        case WireRequestOpcode::AnalogSensors_Ch3:
        case WireRequestOpcode::AnalogSensors_Ch4:
        case WireRequestOpcode::AnalogSensors_Ch5:
        case WireRequestOpcode::AnalogSensors_Ch6:
        case WireRequestOpcode::AnalogSensors_Ch7:
            return true;
        default:
            return false;
    }
}
volatile WireRequestOpcode currentWireMode = WireRequestOpcode::CPUClockConfiguration;
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
        switch (static_cast<WireReceiveOpcode>(Wire.read())) {
            case WireReceiveOpcode::SetMode: 
                if (auto currentOpcode = static_cast<WireRequestOpcode>(Wire.read()); valid(currentOpcode)) {
                    currentWireMode = currentOpcode;
                }
                break;
            default:
                break;
        }
    }
    sinkWire();
}

void
onRequestHandler() {
    switch (currentWireMode) {
        case WireRequestOpcode::CPUClockConfiguration:
            Wire.write(CLK2Rate);
            Wire.write(CLK1Rate);
            break;
        case WireRequestOpcode::CPUClockConfiguration_CLK2:
            Wire.write(CLK2Rate);
            break;
        case WireRequestOpcode::CPUClockConfiguration_CLK1:
            Wire.write(CLK1Rate);
            break;
        case WireRequestOpcode::AnalogSensors:
            Wire.write(reinterpret_cast<char*>(CurrentChannelSamples), sizeof(CurrentChannelSamples));
            break;
#define X(index) case WireRequestOpcode::AnalogSensors_Ch ## index : Wire.write(CurrentChannelSamples [ index ] ); break
            X(0);
            X(1);
            X(2);
            X(3);
            X(4);
            X(5);
            X(6);
            X(7);
#undef X
        default:
            break;

    }
}
