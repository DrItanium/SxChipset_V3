#include <Arduino.h>
#include <Event.h>
#include <Logic.h>
#include <Wire.h>
#include <EEPROM.h>
#include <SPI.h>
#include "ManagementEngineProtocol.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <microshell.h>
#include "InterfaceEngineCommon.h"
#include "i960CommonInterface.h"

// Mapped/Registered Peripherals
// TCA0: CLK/2 generator (CLK1OUT)
// TCA1:
// TCB0: INT1 (potentially)
// TCB1: Ready pulse generator
// TCB2: micros/millis (reserved by DxCore)
// TCB3: INT2 (potentially)
// TCB4: INT0 (potentially)
// TCD0: 
// CCL0: 
// CCL1: 
// CCL2: 
// CCL3: 
// CCL4: INT3 (potentially)
// CCL5: INT0 (potentially)
// TWI0: Teensy Communication Channel
// TWI1: 
// USART0:
// USART1: Teensy Communication Channel
// USART2:
// USART3:
// USART4:
// USART5:
// SPI0: ILI9341 display
// SPI1:
// registered pins
constexpr auto CLK1OUT = PIN_PA0;
constexpr auto INT1_IN = PIN_PA1;
constexpr auto INT1_OUT = PIN_PA2;
// gets logic level converted as well to save a timer and VDDIO2 pins
constexpr auto READY_OUT = PIN_PA3;
// PIN_PA4
constexpr auto READY_IN = PIN_PA5;
// PIN_PA6
constexpr auto CLK2OUT = PIN_PA7;

constexpr auto INT3_IN = PIN_PB0;
// PIN_PB1
// PIN_PB2
constexpr auto INT3_OUT = PIN_PB3;
// PIN_PB4
constexpr auto INT2_OUT = PIN_PB5;
// PIN_PB6
// PIN_PB7

constexpr auto TEENSY_SER_TX = PIN_PC0;
constexpr auto TEENSY_SER_RX = PIN_PC1;
constexpr auto WIRE_SDA = PIN_PC2;
constexpr auto WIRE_SCL = PIN_PC3;
// PIN_PC4
// PIN_PC5
// PIN_PC6
// PIN_PC7

// PIN_PD0
// PIN_PD1
// PIN_PD2
// PIN_PD3
constexpr auto ADS = PIN_PD4;
constexpr auto BLAST = PIN_PD5;
// PIN_PD6
constexpr auto DEN = PIN_PD7;

constexpr auto DISPLAY_MOSI = PIN_PE0;
constexpr auto DISPLAY_MISO = PIN_PE1;
constexpr auto DISPLAY_SCK = PIN_PE2;
constexpr auto WR = PIN_PE3;
constexpr auto BE0 = PIN_PE4;
constexpr auto BE1 = PIN_PE5;
constexpr auto DISPLAY_DC = PIN_PE6;
constexpr auto DISPLAY_RST = PIN_PE7;

// PIN_PF0
// PIN_PF1
// PIN_PF2
// PIN_PF3
constexpr auto DISPLAY_CS = PIN_PF4;
constexpr auto INT2_IN = PIN_PF5; 
// PIN_PF6 (input only/Reset)

constexpr auto INT0_IN = PIN_PG0;
// PIN_PG1
// PIN_PG2
constexpr auto INT0_OUT = PIN_PG3;
constexpr auto HLDA960 = PIN_PG4;
constexpr auto RESET960 = PIN_PG5;
constexpr auto HOLD960 = PIN_PG6;
constexpr auto LOCK960 = PIN_PG7;

// will be updated on startup
uint32_t CLKSpeeds [] {
    0, 0,
};
// make it really easy to transmit the serial number cache as fast as possible
uint8_t serialNumberCache[16] = { 0 };
uint8_t devidCache[3] = { 0 };
long rSeed = 0;
bool chipIsUp = false;
bool cpuIsInReset = true;
Adafruit_ILI9341 tft{DISPLAY_CS, DISPLAY_DC, DISPLAY_RST};
void setupDisplay() noexcept;
void
configurePins() noexcept {
    pinMode(CLK1OUT, OUTPUT);
    pinMode(INT1_IN, INPUT);
    pinMode(INT1_OUT, OUTPUT);
    digitalWrite(INT1_OUT, HIGH);
    pinMode(READY_OUT, OUTPUT);
    digitalWrite(READY_OUT, HIGH);
    pinMode(READY_IN, INPUT);
    pinMode(CLK2OUT, OUTPUT);
    pinMode(RESET960, OUTPUT);
    digitalWrite(RESET960, LOW);
    pinMode(INT3_IN, INPUT);
    pinMode(INT3_OUT, OUTPUT);
    digitalWrite(INT3_OUT, HIGH);
    pinMode(INT2_OUT, OUTPUT);
    digitalWrite(INT2_OUT, LOW);
    pinMode(ADS, INPUT);
    pinMode(BLAST, INPUT);
    pinMode(DEN, INPUT);
    pinMode(WR, INPUT);
    pinMode(BE0, INPUT);
    pinMode(BE1, INPUT);
    pinMode(INT2_IN, INPUT);
    pinMode(INT0_IN, INPUT);
    pinMode(INT0_OUT, OUTPUT);
    digitalWrite(INT0_OUT, HIGH);
    pinMode(HLDA960, INPUT);
    pinMode(HOLD960, OUTPUT);
    digitalWrite(HOLD960, LOW);
    pinMode(LOCK960, INPUT);
}
namespace i960 {
bool isBusHeld() noexcept { return digitalRead(HLDA960) == HIGH; }
bool isBusLocked() noexcept { return digitalRead(LOCK960) == LOW; }

bool 
cpuRunning() noexcept { 
    return !cpuIsInReset; 
}

void 
putCPUInReset() noexcept {
    digitalWrite(RESET960, LOW); 
    cpuIsInReset = true;
}
void
pullCPUOutOfReset() noexcept {
    digitalWrite(RESET960, HIGH); 
    cpuIsInReset = false;
}
void
holdBus() noexcept {
    digitalWrite(HOLD960, HIGH);
}
void
releaseBus() noexcept {
    digitalWrite(HOLD960, LOW);
}
}
using i960::isBusHeld;
using i960::isBusLocked;
using i960::cpuRunning;
using i960::putCPUInReset;
using i960::pullCPUOutOfReset;
using i960::holdBus;
using i960::releaseBus;

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
static_assert(computeCCLValueBasedOffOfFunction(generateOscillatingPattern) == 0b01010101);

void
updateClockFrequency(uint32_t frequency) noexcept {
    CLKSpeeds[0] = frequency;
    CLKSpeeds[1] = frequency / 2;
}
static_assert(gen0::pin_pa5 == gen1::pin_pa5);
void
setupInterrupt0Pins() noexcept {
    // INT0 from the teensy is connected to INT0_IN 
    // INT0_OUT is separated from the two
    //
    // There are different ways to do this but right now INT0_IN is connected
    // to PG0 and INT0_OUT is connected to PG3. Thus I can either use:
    // 1. CCL to act as a passthrough
    // 2. CCL to enable edge detection (bleh)
    // 3. Use EVSYS to hook INT0_IN to TCB4's trigger pulse
    // 4. Use CCL and EVSYS to route INT0_IN to TCB4's trigger pulse (allows for more flexible EVSYS usage)
}
void
configureReadyPulseGenerator() noexcept {
  // okay, so start configuring the secondary single shot setup
  PORTMUX.TCBROUTEA = (PORTMUX.TCBROUTEA & ~(PORTMUX_TCB1_bm));
  TCB1.CCMP = 1; // two cycles
  TCB1.CNT = 1; // 
  TCB1.EVCTRL = TCB_CAPTEI_bm | TCB_EDGE_bm; // enable EVSYS input
  TCB1.CTRLB = TCB_CNTMODE_SINGLE_gc | // enable single shot mode
               TCB_CCMPEN_bm; // enable output via GPIO
  TCB1.CTRLA = TCB_RUNSTDBY_bm | // run in standby
               TCB_ENABLE_bm | // enable
               TCB_CLKSEL_EVENT_gc; // clock comes from EVSYS
}
void
configureDivideByTwoClockGenerator() noexcept {
  // create a divide by two clock generator
  PORTMUX.TCAROUTEA = (PORTMUX.TCAROUTEA & ~(PORTMUX_TCA0_gm)) | PORTMUX_TCA0_PORTA_gc; // enable TCA0 on PORTA
  TCA0.SINGLE.CMP0 = 0; // 12MHz compare

  TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm |  // cmp0 enable
      TCA_SINGLE_WGMODE_FRQ_gc;  // frequency
}
void
startDivideByTwoClockGenerator() noexcept {
  TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm | // turn the device on
      TCA_SINGLE_RUNSTDBY_bm; // run in standby
  PORTA.PIN3CTRL |= PORT_INVEN_bm; // invert the pulse automatically
  PORTA.PIN5CTRL |= PORT_INVEN_bm; // make the input inverted for simplicity
}
void
configureINT2PulseGenerator(Event& clk1out, Event& router) noexcept {
  clk1out.set_user(user::tcb3_cnt); // route it to TCB3 as clock source
  router.set_generator(gen5::pin_pf5); // use PF5/INT2_IN as the generator source for Event5
  router.set_user(user::tcb3_capt); // use PF5 to trigger TCB3 and generate the INT2 pulse
  /// @todo 
}
void 
configureCCLs() {
  
  // PA0 uses TCA0 to generate a 12MHz clock source
  // Useful for various triggers and such
  Event0.set_generator(gen0::pin_pa0);
  Event0.set_user(user::tcb1_cnt); // route it to TCB1 as clock source
  Event0.set_user(user::tcb4_cnt); // route it to TCB4 as clock source
  // Event 1: Route PA5 to TCB1 trigger source
  // setting this to gen1 causes problems (compiler bug?)
  Event1.set_generator(gen0::pin_pa5); // use PA5 as the input
  Event1.set_user(user::tcb1_capt); // use PA5 to trigger TCB1 and generate the READY pulse
  updateClockFrequency(F_CPU); // we are making CLK2 run at 24MHz
  configureINT2PulseGenerator(Event0, Event5);
  configureDivideByTwoClockGenerator();
  configureReadyPulseGenerator();
  startDivideByTwoClockGenerator();
  Event0.start();
  Event1.start();
  Event5.start();
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
void configureMicroshellInterface();
void serviceShell();
void
setup() {
    Wire.swap(2); // it is supposed to be PC2/PC3 TWI ms
    Wire.begin(0x08);
    SPI.swap(SPI0_SWAP1); 
    SPI.begin();
    Serial1.begin(115200);
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
    cpuIsInReset = true;
    chipIsUp = true;
    setupDisplay();
    configureMicroshellInterface();
}
uint16_t
randomColor() noexcept {
    uint32_t baseColor = random();
    return tft.color565(static_cast<uint8_t>(baseColor), static_cast<uint8_t>(baseColor >> 8), static_cast<uint8_t>(baseColor >> 16));
}
class FizzleFade {
    public:
    FizzleFade(int w, int h, uint16_t color = randomColor()) : _width(w), _height(h), _color(color) { }
    void cycle() noexcept {
        {
            uint16_t y = _rndval & 0x001FF;
            uint16_t x = (_rndval & 0x3FE00) >> 9;
            unsigned lsb = _rndval & 1;
            _rndval >>= 1;
            if (lsb) {
                _rndval ^= 0x0001'2000;
            }
            if (x < _width && y < _height) {
                tft.drawPixel(x, y, _color);
            }
        }
        if (_rndval == 1) {
            _rndval = 1;
            _color = randomColor();
        }
    }
    private:
        int _width;
        int _height;
        uint16_t _color;
        uint32_t _rndval = 1;

};
FizzleFade ff(tft.width(), tft.height());
void
loop() {
    ff.cycle();
    serviceShell();
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
                putCPUInReset();
                break;
            case ManagementEngineReceiveOpcode::PullOutOfReset: 
                pullCPUOutOfReset();
                break;
            case ManagementEngineReceiveOpcode::HoldBus:
                holdBus();
                break;
            case ManagementEngineReceiveOpcode::ReleaseBus:
                releaseBus();
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
            Wire.write(isBusHeld() ? 0xFF : 0x00); 
            break;
        case ManagementEngineRequestOpcode::BusIsLocked: 
            Wire.write(isBusLocked() ? 0xFF : 0x00); 
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
        case ManagementEngineRequestOpcode::CpuRunning:
            Wire.write(cpuRunning() ? 0xFF : 0x00);
            break;
        default:
            break;

    }
}

void
setupDisplay() noexcept {
    tft.begin();
    tft.fillScreen(ILI9341_BLACK);
}

int ushRead(struct ush_object* self, char* ch) {
    if (Serial1.available()) {
        *ch = Serial1.read();
        return 1;
    }
    return 0;
}

int ushWrite(struct ush_object* self, char ch) {
    return (Serial1.write(ch) == 1);
}

const ush_io_interface ioInterface = { .read = ushRead, .write = ushWrite };

char ushInputBuffer[256];
char ushOutputBuffer[256];

constexpr auto PATH_MAX_SIZE = 256;

struct ush_object ush;

const ush_descriptor PROGMEM_MAPPED descriptor = {
    .io = &ioInterface,
    .input_buffer = ushInputBuffer,
    .input_buffer_size = sizeof(ushInputBuffer),
    .output_buffer = ushOutputBuffer,
    .output_buffer_size = sizeof(ushOutputBuffer),
    .path_max_length = PATH_MAX_SIZE,
    .hostname = "management_engine",
};

size_t infoGetDataCallback(ush_object* self, ush_file_descriptor const* file, uint8_t** data) {
    static const char* message = "AVR Console\r\n";
    *data = (uint8_t*)message;
    return strlen(message);
}
const ush_file_descriptor PROGMEM_MAPPED rootFiles[] {
    {
        .name = "info.txt",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = infoGetDataCallback,
    }
};

const ush_file_descriptor devFiles[] {
    INTERFACE_ENGINE_COMMON_DEVICES,
    {
        .name = "rseed",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](ush_object* self, ush_file_descriptor const* file, uint8_t** data) {
            static char buffer[16];
            snprintf(buffer, sizeof(buffer), "%ld\r\n", rSeed);
            buffer[sizeof(buffer) - 1] = 0;
            *data = (uint8_t*)buffer;
            return strlen((char*)(*data));
        },
        .set_data = [](ush_object* self, ush_file_descriptor const* file, uint8_t* data, size_t size) {
            long value = 0;
            if (sscanf((const char*)data, "%lu", &value) == EOF) {
                ush_print_status(self, USH_STATUS_ERROR_COMMAND_SYNTAX_ERROR);
                return;
            }
            rSeed = value;
            randomSeed(rSeed);
        },
    },
};
const ush_file_descriptor displayFiles[] {
    {
        .name = "width",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](ush_object* self, ush_file_descriptor const* file, uint8_t** data) {
            static char buffer[8];
            snprintf(buffer, sizeof(buffer), "%ld\r\n", (long)tft.width());
            buffer[sizeof(buffer) - 1] = 0;
            *data = (uint8_t*)buffer;
            return strlen((char*)(*data));
        },
    },
    {
        .name = "height",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](ush_object* self, ush_file_descriptor const* file, uint8_t** data) {
            static char buffer[8];
            snprintf(buffer, sizeof(buffer), "%ld\r\n", (long)tft.height());
            buffer[sizeof(buffer) - 1] = 0;
            *data = (uint8_t*)buffer;
            return strlen((char*)(*data));
        },
    },
    {
        .name = "rotation",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](ush_object* self, ush_file_descriptor const* file, uint8_t** data) {
            static char buffer[8];
            snprintf(buffer, sizeof(buffer), "%ld\r\n", (long)tft.getRotation());
            buffer[sizeof(buffer) - 1] = 0;
            *data = (uint8_t*)buffer;
            return strlen((char*)(*data));
        },
        .set_data = [](ush_object* self, ush_file_descriptor const* file, uint8_t* data, size_t size) {
            long value = 0;
            if (sscanf((const char*)data, "%lu", &value) == EOF) {
                ush_print_status(self, USH_STATUS_ERROR_COMMAND_SYNTAX_ERROR);
                return;
            }
            tft.setRotation(static_cast<uint8_t>(value));
        },
    },
    {
        .name = "cursor_x",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](ush_object* self, ush_file_descriptor const* file, uint8_t** data) {
            static char buffer[8];
            snprintf(buffer, sizeof(buffer), "%ld\r\n", (long)tft.getCursorX());
            buffer[sizeof(buffer) - 1] = 0;
            *data = (uint8_t*)buffer;
            return strlen((char*)(*data));
        },
    },
    {
        .name = "cursor_y",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = [](ush_object* self, ush_file_descriptor const* file, uint8_t** data) {
            static char buffer[8];
            snprintf(buffer, sizeof(buffer), "%ld\r\n", (long)tft.getCursorY());
            buffer[sizeof(buffer) - 1] = 0;
            *data = (uint8_t*)buffer;
            return strlen((char*)(*data));
        },
    },
    {
        .name = "invert",
        .description = nullptr,
        .help = nullptr,
        .exec = nullptr,
        .get_data = nullptr,
        .set_data = [](ush_object* self, ush_file_descriptor const* file, uint8_t* data, size_t size) {
            long value = 0;
            if (sscanf((const char*)data, "%lu", &value) == EOF) {
                ush_print_status(self, USH_STATUS_ERROR_COMMAND_SYNTAX_ERROR);
                return;
            }
            tft.invertDisplay(value != 0);
        },
    },
};

ush_node_object root;
ush_node_object dev;
ush_node_object displayNode;

void
configureMicroshellInterface() noexcept {
    ush_init(&ush, &descriptor);

    InterfaceEngine::installCommonCommands(&ush);
    InterfaceEngine::installI960Commands(&ush);
    ush_node_mount(&ush, "/", &root, rootFiles, ComputeFileSize(rootFiles));
    ush_node_mount(&ush, "/dev", &dev, devFiles, ComputeFileSize(devFiles));
    InterfaceEngine::installEepromDeviceDirectory(&ush);
    InterfaceEngine::installI960Devices(&ush);
    ush_node_mount(&ush, "/dev/display", &displayNode, displayFiles, ComputeFileSize(displayFiles));
}

void
serviceShell() noexcept {
    ush_service(&ush);
}

