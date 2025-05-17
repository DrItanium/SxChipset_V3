#include <Arduino.h>
#include <Event.h>
#include <Logic.h>
#include <Wire.h>

constexpr auto CLK2Out = PIN_PD3;
constexpr auto CLK1Out = PIN_PB3;
constexpr auto CLKOUT = PIN_PA7;
constexpr auto RP2350_READY_IN = PIN_PC0;
constexpr auto RP2350_READY_SYNC = PIN_PC6;
constexpr auto i960_READY_SYNC = PIN_PE7;


void
configurePins() noexcept {
  pinMode(i960_READY_SYNC, OUTPUT);  
  pinMode(CLKOUT, OUTPUT);
  pinMode(CLK1Out, OUTPUT);
  pinMode(CLK2Out, OUTPUT);
  pinMode(RP2350_READY_IN, INPUT);
  pinMode(RP2350_READY_SYNC, OUTPUT);
}
void
setupSystemClocks() noexcept {
  // configure CLKOUT
  CCP = 0xD8;
  CLKCTRL.MCLKCTRLA = 0b1000'0000; // CLKOUT + Internal oscillator
  asm volatile("nop");
  CCP = 0xD8;
  CLKCTRL.OSCHFCTRLA = 0b1'0'1001'0'0;
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
configureCCLs() {
  
  Event0.set_generator(gen0::pin_pa7); // 24/20MHz
  Event0.set_user(user::ccl2_event_a);
  Event0.set_user(user::ccl3_event_a);
  Event1.set_generator(gen::ccl2_out); // 12/10MHz
  Event1.set_user(user::ccl4_event_a);
  Event1.set_user(user::ccl5_event_a);
  Event2.set_generator(gen::ccl4_out); // 6/5MHz
  Event2.set_user(user::ccl1_event_a);
  Event3.set_generator(gen::ccl1_out);
  Event3.set_user(user::evoute_pin_pe7); // i960 "5V" Ready transmit

  configureDivideByTwoCCL<true>(Logic2, Logic3); // divide by two
  configureDivideByTwoCCL<true>(Logic4, Logic5); // divide by four
  // ready signal detector
  Logic1.enable = true;
  Logic1.input0 = in::pin;
  Logic1.input1 = in::disable;
  Logic1.input2 = in::event_a;
  Logic1.output = out::enable;
  Logic1.output_swap = out::pin_swap;
  Logic1.clocksource = clocksource::in2;
  Logic1.edgedetect = edgedetect::enable;
  Logic1.sequencer = sequencer::disable;
  Logic1.filter = filter::synch;
  Logic1.truth = 0b1111'0101; // falling edge detector 
  Logic1.init();
  // invert the output pin to make it output the correct pulse shape :)
  PORTC.PIN6CTRL |= PORT_INVEN_bm;

  Event0.start();
  Event1.start();
  Event2.start();
  // make sure that power 
  CCL.CTRLA |= CCL_RUNSTDBY_bm;
  Logic::start();

 
}
void
setup() {
  setupSystemClocks();
  configurePins();
  configureCCLs();
  Wire.swap(2); // it is supposed to be PC2/PC3 TWI ms
  Wire.begin();
}

void
loop() {
  // put your main code here, to run repeatedly:
}

