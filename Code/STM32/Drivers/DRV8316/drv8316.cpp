#include "drv8316.h"

/* ===== Constructor ===== */
DRV8316::DRV8316(SPI_HandleTypeDef* hspi,
                 GPIO_TypeDef* csPort, uint16_t csPin,
                 GPIO_TypeDef* drvoffPort, uint16_t drvoffPin,
                 GPIO_TypeDef* sleepPort, uint16_t sleepPin)
{
    _hspi = hspi;

    _csPort = csPort;
    _csPin = csPin;

    _drvoffPort = drvoffPort;
    _drvoffPin = drvoffPin;

    _sleepPort = sleepPort;
    _sleepPin = sleepPin;
}

/* ===== Init (default 3PWM config) ===== */
void DRV8316::init()
{
    DRV8316Config cfg;
    init(cfg);
}

/* ===== Init (explicit config) ===== */
void DRV8316::init(const DRV8316Config &cfg)
{
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);

    disable();

    wake();

    configureBasic(cfg);

    enable();
}

/* ===== Enable / Disable ===== */
void DRV8316::enable()
{
    HAL_GPIO_WritePin(_drvoffPort, _drvoffPin, GPIO_PIN_RESET);
}

void DRV8316::disable()
{
    HAL_GPIO_WritePin(_drvoffPort, _drvoffPin, GPIO_PIN_SET);
}

void DRV8316::sleep()
{
	HAL_GPIO_WritePin(_sleepPort, _sleepPin, GPIO_PIN_RESET);
}

void DRV8316::wake()
{
	HAL_GPIO_WritePin(_sleepPort, _sleepPin, GPIO_PIN_SET);
	HAL_Delay(1);	// 1 ms delay
}


/* ===== SPI ===== */
uint16_t DRV8316::transfer16(uint16_t tx)
{
    uint16_t rx = 0;

    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive(_hspi,
                            (uint8_t*)&tx,
                            (uint8_t*)&rx,
                            1,
                            HAL_MAX_DELAY);

    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);

    return rx;
}

/* ===== Parity ===== */
uint8_t DRV8316::computeParity(uint16_t data)
{
    uint8_t count = 0;

    for (int i = 0; i < 16; i++) {
        if ((data >> i) & 1) count++;
    }

    return (count % 2) ? 1 : 0;
}

/* ===== Read register ===== */
uint16_t DRV8316::readRegister(uint8_t addr)
{
    uint16_t frame = 0;

    frame |= (1 << 15);
    frame |= (addr & 0x3F) << 9;

    if (computeParity(frame))
        frame |= (1 << 8);

    return transfer16(frame);
}

/* ===== Write register ===== */
void DRV8316::writeRegister(uint8_t addr, uint8_t data)
{
    uint16_t frame = 0;

    frame |= (addr & 0x3F) << 9;
    frame |= data;

    if (computeParity(frame))
        frame |= (1 << 8);

    transfer16(frame);
}

/* ===== Basic configuration =====
 *
 * Per datasheet: PWM_MODE must only be changed while INHx/INLx are all
 * low. We're called from init() before the timer is started, so the
 * GPIOs are still idle low at this point. */
void DRV8316::configureBasic(const DRV8316Config &cfg)
{
    /* --- Unlock registers --- */
    Control__1 ctrl1 = {};
    ctrl1.REG_LOCK = REG_LOCK_UNLOCK;
    writeRegister(Control__1_ADDR, ctrl1.reg);
    HAL_Delay(1);

    /* --- Control 2: PWM mode + slew --- */
    Control__2 ctrl2 = {};
    ctrl2.CLR_FLT  = 0;
    ctrl2.PWM_MODE = cfg.pwmMode;
    ctrl2.SLEW     = cfg.slew;
    ctrl2.SDO_MODE = cfg.sdoMode;
    writeRegister(Control__2_ADDR, ctrl2.reg);
    HAL_Delay(1);

    /* --- Control 3: protection --- */
    Control__3 ctrl3 = {};
    ctrl3.OVP_EN      = cfg.ovpEn       ? OVP_EN_ENABLE       : OVP_EN_DISABLE;
    ctrl3.SPI_FLT_REP = cfg.spiFltRep   ? SPI_FLT_REP_ENABLE  : SPI_FLT_REP_DISABLE;
    ctrl3.OTW_REP     = cfg.otwRep      ? OTW_REP_ENABLE      : OTW_REP_DISABLE;
    writeRegister(Control__3_ADDR, ctrl3.reg);
    HAL_Delay(1);

    /* --- Control 4: OCP --- */
    Control__4 ctrl4 = {};
    ctrl4.OCP_MODE  = cfg.ocpMode;
    ctrl4.OCP_CBC   = cfg.ocpCbc ? OCP_CBC_ENABLE : OCP_CBC_DISABLE;
    ctrl4.OCP_DEG   = cfg.ocpDeg;       // NEW
    ctrl4.OCP_RETRY = cfg.ocpRetry;     // NEW
    ctrl4.OCP_LVL   = cfg.ocpLvl;       // NEW
    ctrl4.DRV_OFF   = DRV_OFF_DISABLE;
    writeRegister(Control__4_ADDR, ctrl4.reg);
    HAL_Delay(1);

    /* --- Control 5: CSA gain + sync/async rectification --- */
    Control__5 ctrl5 = {};
    ctrl5.CSA_GAIN = cfg.csaGain;
    ctrl5.EN_ASR   = cfg.enAsr ? EN_ASR_ENABLE : EN_ASR_DISABLE;
    ctrl5.EN_AAR   = cfg.enAar ? EN_AAR_ENABLE : EN_AAR_DISABLE;
    writeRegister(Control__5_ADDR, ctrl5.reg);
    HAL_Delay(1);

    /* --- Control 6: buck --- */
    Control__6 ctrl6 = {};
    ctrl6.BUCK_DIS    = cfg.buckDis   ? BUCK_DIS_BUCK_DISABLE : BUCK_DIS_BUCK_ENABLE;
    ctrl6.BUCK_PS_DIS = cfg.buckPsDis ? BUCK_PS_DIS_DISABLE   : BUCK_PS_DIS_ENABLE;
    writeRegister(Control__6_ADDR, ctrl6.reg);
    HAL_Delay(1);

    /* --- Control 10: delay compensation --- */
    Control__10 ctrl10 = {};
    ctrl10.DLYCMP_EN  = cfg.dlyCmpEn ? DLYCMP_EN_ENABLE : DLYCMP_EN_DISABLE;
    ctrl10.DLY_TARGET = cfg.dlyTarget;
    writeRegister(Control__10_ADDR, ctrl10.reg);
    HAL_Delay(1);
}

/* ===== Set PWM mode (live) =====
 *
 * Caller is responsible for ensuring all INHx/INLx are LOW before
 * calling this — the datasheet warns against changing PWM_MODE while
 * the FETs are switching. */
void DRV8316::setPWMMode(Control__2 &reg, uint8_t mode)
{
    reg.PWM_MODE = mode;
    writeRegister(Control__2_ADDR, reg.reg);
}

/* ===== Clear fault ===== */
void DRV8316::clearFault()
{
    uint16_t res = readRegister(Control__2_ADDR);

    Control__2 ctrl2;
    ctrl2.reg = res & 0xFF;

    ctrl2.CLR_FLT = 1;

    writeRegister(Control__2_ADDR, ctrl2.reg);
}

/* ===== Status ===== */
IC_Status DRV8316::getStatus0()
{
    uint16_t res = readRegister(IC_Status_ADDR);

    IC_Status status;
    status.reg = res & 0xFF;

    return status;
}

Status__1 DRV8316::getStatus1()
{
    uint16_t res = readRegister(Status__1_ADDR);

    Status__1 status;
    status.reg = res & 0xFF;

    return status;
}

Status__2 DRV8316::getStatus2()
{
    uint16_t res = readRegister(Status__2_ADDR);

    Status__2 status;
    status.reg = res & 0xFF;

    return status;
}
