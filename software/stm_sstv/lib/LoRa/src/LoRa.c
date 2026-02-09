/**
 * LoRa SX1276/SX1278 library for STM32
 * Based on work by Sslman Motlaq
 * Extended with FSK Continuous mode support for AFSK/SSTV transmission
 */

#include "LoRa.h"

/* Internal functions */
static void LoRa_readReg(LoRa* _LoRa, uint8_t* address, uint16_t r_length, uint8_t* output, uint16_t w_length);
static void LoRa_writeReg(LoRa* _LoRa, uint8_t* address, uint16_t r_length, uint8_t* values, uint16_t w_length);
static void LoRa_BurstWrite(LoRa* _LoRa, uint8_t address, uint8_t *value, uint8_t length);
static void LoRa_setLowDaraRateOptimization(LoRa* _LoRa, uint8_t value);
static void LoRa_setAutoLDO(LoRa* _LoRa);
static void LoRa_setTOMsb_setCRCon(LoRa* _LoRa);

LoRa newLoRa(void)
{
    LoRa new_LoRa;

    new_LoRa.frequency             = 433;
    new_LoRa.spredingFactor        = SF_7;
    new_LoRa.bandWidth             = BW_125KHz;
    new_LoRa.crcRate               = CR_4_5;
    new_LoRa.power                 = POWER_20db;
    new_LoRa.overCurrentProtection = 100;
    new_LoRa.preamble              = 8;
    new_LoRa.is_fsk_mode           = 0;

    return new_LoRa;
}

void LoRa_reset(LoRa* _LoRa)
{
    HAL_GPIO_WritePin(_LoRa->reset_port, _LoRa->reset_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(_LoRa->reset_port, _LoRa->reset_pin, GPIO_PIN_SET);
    HAL_Delay(100);
}

static void LoRa_readReg(LoRa* _LoRa, uint8_t* address, uint16_t r_length, uint8_t* output, uint16_t w_length)
{
    HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(_LoRa->hSPIx, address, r_length, TRANSMIT_TIMEOUT);
    while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY);
    HAL_SPI_Receive(_LoRa->hSPIx, output, w_length, RECEIVE_TIMEOUT);
    while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY);
    HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_SET);
}

static void LoRa_writeReg(LoRa* _LoRa, uint8_t* address, uint16_t r_length, uint8_t* values, uint16_t w_length)
{
    HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(_LoRa->hSPIx, address, r_length, TRANSMIT_TIMEOUT);
    while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY);
    HAL_SPI_Transmit(_LoRa->hSPIx, values, w_length, TRANSMIT_TIMEOUT);
    while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY);
    HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_SET);
}

uint8_t LoRa_read(LoRa* _LoRa, uint8_t address)
{
    uint8_t read_data;
    uint8_t data_addr;

    data_addr = address & 0x7F;
    LoRa_readReg(_LoRa, &data_addr, 1, &read_data, 1);

    return read_data;
}

void LoRa_write(LoRa* _LoRa, uint8_t address, uint8_t value)
{
    uint8_t data;
    uint8_t addr;

    addr = address | 0x80;
    data = value;
    LoRa_writeReg(_LoRa, &addr, 1, &data, 1);
}

static void LoRa_BurstWrite(LoRa* _LoRa, uint8_t address, uint8_t *value, uint8_t length)
{
    uint8_t addr;
    addr = address | 0x80;

    HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(_LoRa->hSPIx, &addr, 1, TRANSMIT_TIMEOUT);
    while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY);
    HAL_SPI_Transmit(_LoRa->hSPIx, value, length, TRANSMIT_TIMEOUT);
    while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY);
    HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_SET);
}

void LoRa_gotoMode(LoRa* _LoRa, int mode)
{
    uint8_t read;
    uint8_t data;

    read = LoRa_read(_LoRa, RegOpMode);
    data = (read & 0xF8) | (mode & 0x07);
    _LoRa->current_mode = mode;

    LoRa_write(_LoRa, RegOpMode, data);
}

static void LoRa_setLowDaraRateOptimization(LoRa* _LoRa, uint8_t value)
{
    uint8_t data;
    uint8_t read;

    read = LoRa_read(_LoRa, RegModemConfig3);

    if (value)
        data = read | 0x08;
    else
        data = read & 0xF7;

    LoRa_write(_LoRa, RegModemConfig3, data);
    HAL_Delay(10);
}

static void LoRa_setAutoLDO(LoRa* _LoRa)
{
    double BW[] = {7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0, 500.0};
    LoRa_setLowDaraRateOptimization(_LoRa, (long)((1 << _LoRa->spredingFactor) / ((double)BW[_LoRa->bandWidth])) > 16.0);
}

void LoRa_setFrequency(LoRa* _LoRa, int freq)
{
    uint8_t data;
    uint32_t F;
    F = (freq * 524288) >> 5;

    data = F >> 16;
    LoRa_write(_LoRa, RegFrMsb, data);
    HAL_Delay(5);

    data = F >> 8;
    LoRa_write(_LoRa, RegFrMid, data);
    HAL_Delay(5);

    data = F >> 0;
    LoRa_write(_LoRa, RegFrLsb, data);
    HAL_Delay(5);
}

void LoRa_setSpreadingFactor(LoRa* _LoRa, int SF)
{
    uint8_t data;
    uint8_t read;

    if (SF > 12) SF = 12;
    if (SF < 7) SF = 7;

    read = LoRa_read(_LoRa, RegModemConfig2);
    HAL_Delay(10);

    data = (SF << 4) + (read & 0x0F);
    LoRa_write(_LoRa, RegModemConfig2, data);
    HAL_Delay(10);

    LoRa_setAutoLDO(_LoRa);
}

void LoRa_setPower(LoRa* _LoRa, uint8_t power)
{
    LoRa_write(_LoRa, RegPaConfig, power);
    HAL_Delay(10);
}

void LoRa_setOCP(LoRa* _LoRa, uint8_t current)
{
    uint8_t OcpTrim = 0;

    if (current < 45) current = 45;
    if (current > 240) current = 240;

    if (current <= 120)
        OcpTrim = (current - 45) / 5;
    else if (current <= 240)
        OcpTrim = (current + 30) / 10;

    OcpTrim = OcpTrim + (1 << 5);
    LoRa_write(_LoRa, RegOcp, OcpTrim);
    HAL_Delay(10);
}

static void LoRa_setTOMsb_setCRCon(LoRa* _LoRa)
{
    uint8_t read, data;
    read = LoRa_read(_LoRa, RegModemConfig2);
    data = read | 0x07;
    LoRa_write(_LoRa, RegModemConfig2, data);
    HAL_Delay(10);
}

void LoRa_setSyncWord(LoRa* _LoRa, uint8_t syncword)
{
    LoRa_write(_LoRa, RegSyncWord, syncword);
    HAL_Delay(10);
}

uint8_t LoRa_transmit(LoRa* _LoRa, uint8_t* data, uint8_t length, uint16_t timeout)
{
    uint8_t read;
    int mode = _LoRa->current_mode;

    LoRa_gotoMode(_LoRa, STNBY_MODE);
    read = LoRa_read(_LoRa, RegFiFoTxBaseAddr);
    LoRa_write(_LoRa, RegFiFoAddPtr, read);
    LoRa_write(_LoRa, RegPayloadLength, length);
    LoRa_BurstWrite(_LoRa, RegFiFo, data, length);
    LoRa_gotoMode(_LoRa, TRANSMIT_MODE);

    while (1) {
        read = LoRa_read(_LoRa, RegIrqFlags);
        if ((read & 0x08) != 0) {
            LoRa_write(_LoRa, RegIrqFlags, 0xFF);
            LoRa_gotoMode(_LoRa, mode);
            return 1;
        } else {
            if (--timeout == 0) {
                LoRa_gotoMode(_LoRa, mode);
                return 0;
            }
        }
        HAL_Delay(1);
    }
}

void LoRa_startReceiving(LoRa* _LoRa)
{
    LoRa_gotoMode(_LoRa, RXCONTIN_MODE);
}

uint8_t LoRa_receive(LoRa* _LoRa, uint8_t* data, uint8_t length)
{
    uint8_t read;
    uint8_t number_of_bytes;
    uint8_t min = 0;

    for (int i = 0; i < length; i++)
        data[i] = 0;

    LoRa_gotoMode(_LoRa, STNBY_MODE);
    read = LoRa_read(_LoRa, RegIrqFlags);

    if ((read & 0x40) != 0) {
        LoRa_write(_LoRa, RegIrqFlags, 0xFF);
        number_of_bytes = LoRa_read(_LoRa, RegRxNbBytes);
        read = LoRa_read(_LoRa, RegFiFoRxCurrentAddr);
        LoRa_write(_LoRa, RegFiFoAddPtr, read);
        min = length >= number_of_bytes ? number_of_bytes : length;
        for (int i = 0; i < min; i++)
            data[i] = LoRa_read(_LoRa, RegFiFo);
    }
    LoRa_gotoMode(_LoRa, RXCONTIN_MODE);
    return min;
}

int LoRa_getRSSI(LoRa* _LoRa)
{
    uint8_t read;
    read = LoRa_read(_LoRa, RegPktRssiValue);
    return -164 + read;
}

uint16_t LoRa_init(LoRa* _LoRa)
{
    uint8_t data;
    uint8_t read;

    LoRa_gotoMode(_LoRa, SLEEP_MODE);
    HAL_Delay(10);

    /* Turn on LoRa mode */
    read = LoRa_read(_LoRa, RegOpMode);
    HAL_Delay(10);
    data = read | OPMODE_LORA;
    LoRa_write(_LoRa, RegOpMode, data);
    HAL_Delay(100);

    _LoRa->is_fsk_mode = 0;

    LoRa_setFrequency(_LoRa, _LoRa->frequency);
    LoRa_setPower(_LoRa, _LoRa->power);
    LoRa_setOCP(_LoRa, _LoRa->overCurrentProtection);
    LoRa_write(_LoRa, RegLna, 0x23);
    LoRa_setTOMsb_setCRCon(_LoRa);
    LoRa_setSpreadingFactor(_LoRa, _LoRa->spredingFactor);
    LoRa_write(_LoRa, RegSymbTimeoutL, 0xFF);

    data = (_LoRa->bandWidth << 4) + (_LoRa->crcRate << 1);
    LoRa_write(_LoRa, RegModemConfig1, data);
    LoRa_setAutoLDO(_LoRa);

    LoRa_write(_LoRa, RegPreambleMsb, _LoRa->preamble >> 8);
    LoRa_write(_LoRa, RegPreambleLsb, _LoRa->preamble >> 0);

    read = LoRa_read(_LoRa, RegDioMapping1);
    data = read | 0x3F;
    LoRa_write(_LoRa, RegDioMapping1, data);

    LoRa_gotoMode(_LoRa, STNBY_MODE);
    _LoRa->current_mode = STNBY_MODE;
    HAL_Delay(10);

    read = LoRa_read(_LoRa, RegVersion);
    if (read == 0x12)
        return LORA_OK;
    else
        return LORA_NOT_FOUND;
}

/**
 * Initialize SX1276/78 in FSK mode for AFSK transmission
 * This sets up the module to accept audio input via DIO2 pin
 */
uint16_t LoRa_beginFSK(LoRa* _LoRa)
{
    uint8_t read;

    /* Go to sleep mode first */
    LoRa_gotoMode(_LoRa, SLEEP_MODE);
    HAL_Delay(10);

    /* Set FSK mode (clear LoRa bit, bit 7 = 0) */
    /* RegOpMode: bit 7 = 0 (FSK), bits 6-5 = modulation (00 = FSK) */
    read = LoRa_read(_LoRa, RegOpMode);
    LoRa_write(_LoRa, RegOpMode, (read & 0x7F));  /* Clear bit 7 for FSK mode */
    HAL_Delay(10);

    /* Go to standby mode */
    LoRa_gotoMode(_LoRa, STNBY_MODE);
    HAL_Delay(10);

    _LoRa->is_fsk_mode = 1;

    /* Set frequency */
    LoRa_setFrequency(_LoRa, _LoRa->frequency);

    /* Set power */
    LoRa_setPower(_LoRa, _LoRa->power);

    /* Set OCP */
    LoRa_setOCP(_LoRa, _LoRa->overCurrentProtection);

    /* Set bitrate (not used in continuous mode, but required)
     * Bitrate = FXOSC / BitRate = 32MHz / 4800 = 6667 */
    LoRa_write(_LoRa, RegBitrateMsb, 0x1A);
    LoRa_write(_LoRa, RegBitrateLsb, 0x0B);

    /* Set frequency deviation
     * Fdev = Fstep * Fdev(15,0)
     * For 2.5 kHz: Fdev = 2500 / 61.035 = 41 = 0x0029 */
    LoRa_write(_LoRa, RegFdevMsb, 0x00);
    LoRa_write(_LoRa, RegFdevLsb, 0x29);

    /* Check version */
    read = LoRa_read(_LoRa, RegVersion);
    if (read == 0x12)
        return LORA_OK;
    else
        return LORA_NOT_FOUND;
}

/**
 * Start direct/continuous mode for AFSK
 * This configures DIO2 as DATA input for external audio modulation
 * Call this after beginFSK() and before transmitDirect()
 */
void LoRa_startDirectMode(LoRa* _LoRa)
{
    uint8_t read;

    /* Go to standby mode first */
    LoRa_gotoMode(_LoRa, STNBY_MODE);
    HAL_Delay(10);

    /* Configure DIO mapping for continuous mode:
     * RegDioMapping1 (0x40):
     *   bits 7-6: DIO0
     *   bits 5-4: DIO1 = DCLK (0b00)
     *   bits 3-2: DIO2 = DATA (0b00)
     *   bits 1-0: DIO3
     * We need to set bits 5-2 to 0 for continuous mode */
    read = LoRa_read(_LoRa, RegDioMapping1);
    read = (read & 0xC3) | DIO1_CONT_DCLK | DIO2_CONT_DATA;  /* Clear bits 5-2, set for continuous */
    LoRa_write(_LoRa, RegDioMapping1, read);

    /* Disable AFC/AGC trigger (not needed for TX)
     * RegRxConfig (0x0D in FSK mode): bits 2-0 = trigger
     * 0x00 = None (for TX we don't need this) */
    read = LoRa_read(_LoRa, RegRxConfig);
    LoRa_write(_LoRa, RegRxConfig, (read & 0xF8) | RX_TRIGGER_NONE);

    /* Set continuous mode (not packet mode)
     * RegPacketConfig2 (0x31): bit 6 = DataMode
     *   0 = Continuous mode
     *   1 = Packet mode */
    read = LoRa_read(_LoRa, RegPacketConfig2);
    LoRa_write(_LoRa, RegPacketConfig2, read & ~DATA_MODE_PACKET);  /* Clear bit 6 for continuous */
}

/**
 * Start transmitting in direct/continuous mode
 * Audio on DIO2 pin will modulate the carrier
 */
void LoRa_transmitDirect(LoRa* _LoRa)
{
    LoRa_gotoMode(_LoRa, TRANSMIT_MODE);
}

/**
 * Stop transmission, go to standby mode
 */
void LoRa_standby(LoRa* _LoRa)
{
    LoRa_gotoMode(_LoRa, STNBY_MODE);
}
