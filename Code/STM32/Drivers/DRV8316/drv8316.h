

#ifndef SIMPLEFOC_DRV8316
#define SIMPLEFOC_DRV8316

#include "stm32g4xx_hal.h"
#include "./drv8316_registers.h"



enum DRV8316_PWMMode {
	PWM6_Mode = 0b00,
	PWM3_Mode = 0b10,
};

enum DRV8316_Slew {
	Slew_25Vus = 0b00,
	Slew_50Vus = 0b01,
	Slew_150Vus = 0b10,
	Slew_200Vus = 0b11
};

enum DRV8316_CSAGain {
	Gain_0V15 = 0b00,
	Gain_0V3 = 0b01,
	Gain_0V6 = 0b10,
	Gain_0V1_2 = 0b11
};

enum DRV8316_DelayTarget {
	Delay_0us = 0x0,
	Delay_0us4 = 0x1,
	Delay_0us6 = 0x2,
	Delay_0us8 = 0x3,
	Delay_1us = 0x4,
	Delay_1us2 = 0x5,
	Delay_1us4 = 0x6,
	Delay_1us6 = 0x7,
	Delay_1us8 = 0x8,
	Delay_2us = 0x9,
	Delay_2us2 = 0xA,
	Delay_2us4 = 0xB,
	Delay_2us6 = 0xC,
	Delay_2us8 = 0xD,
	Delay_3us = 0xE,
	Delay_3us2 = 0xF
};

enum DRV8316_Recirculation {
	BrakeMode = 0b00, // FETs
	CoastMode = 0b01  // Diodes
};



/* ===== Optional config passed to init() / configureBasic() =====
 *
 * The defaults match the original 3PWM-trapezoidal configuration so
 * existing call sites of init() (with no arguments) continue to work.
 */
struct DRV8316Config {
    uint8_t pwmMode    = PWM3_Mode;
    uint8_t slew       = Slew_25Vus;
    uint8_t sdoMode    = 1;
    uint8_t csaGain    = 0b10;
    bool    enAsr      = false;
    bool    enAar      = true;
    uint8_t ocpMode    = 0b00;       // latched, for diagnosis
    bool    ocpCbc     = true;
    uint8_t ocpDeg     = 0b00;       // NEW: 0.2 µs (fastest)
    uint8_t ocpRetry   = 0;          // NEW: 5 ms retry
    uint8_t ocpLvl     = 0;          // NEW: 16 A threshold
    bool    ovpEn      = true;
    bool    otwRep     = true;
    bool    spiFltRep  = false;
    bool    buckDis    = true;
    bool    buckPsDis  = true;
    bool    dlyCmpEn   = true;
    uint8_t dlyTarget  = 0b1111;
};



/* ===== Driver Class ===== */
class DRV8316 {
public:
	DRV8316(SPI_HandleTypeDef* hspi,
			GPIO_TypeDef* csPort, uint16_t csPin,
			GPIO_TypeDef* drvoffPort, uint16_t drvoffPin,
			GPIO_TypeDef* sleepPort, uint16_t sleepPin);

	/* Init.
	 * Without arguments uses the default DRV8316Config (3PWM, ASR off).
	 * Pass a config to override — e.g. 6PWM mode with ASR enabled. */
	void init();
	void init(const DRV8316Config &cfg);
	void configureBasic(const DRV8316Config &cfg);

	void enable();
	void disable();
	void sleep();
	void wake();

	/* SPI */
	uint16_t readRegister(uint8_t addr);
	void writeRegister(uint8_t addr, uint8_t data);

	/* Control */
	void setPWMMode(Control__2 &reg, uint8_t mode);
	void clearFault();

	/* Status */
	IC_Status getStatus0();
	Status__1 getStatus1();
	Status__2 getStatus2();

private:
	SPI_HandleTypeDef* _hspi;

	GPIO_TypeDef* _csPort;
	uint16_t _csPin;

	GPIO_TypeDef* _drvoffPort;
	uint16_t _drvoffPin;

	GPIO_TypeDef* _sleepPort;
	uint16_t _sleepPin;

	uint8_t computeParity(uint16_t data);
	uint16_t transfer16(uint16_t tx);
};



#endif
