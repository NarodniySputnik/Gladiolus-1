/**
 * LoRa SX1276/SX1278 library for STM32
 * Based on work by Sslman Motlaq
 * Extended with FSK Continuous mode support for AFSK/SSTV transmission
 */

#ifndef LORA_H
#define LORA_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(STM32F1)
#include "stm32f1xx_hal.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#else
#include "stm32f1xx_hal.h"
#endif

#include <stdint.h>

#define TRANSMIT_TIMEOUT    2000
#define RECEIVE_TIMEOUT     2000

/* Modes (RegOpMode bits 2-0) */
#define SLEEP_MODE          0
#define STNBY_MODE          1
#define FSTX_MODE           2
#define TRANSMIT_MODE       3
#define FSRX_MODE           4
#define RXCONTIN_MODE       5
#define RXSINGLE_MODE       6
#define CAD_MODE            7

/* Bandwidth */
#define BW_7_8KHz           0
#define BW_10_4KHz          1
#define BW_15_6KHz          2
#define BW_20_8KHz          3
#define BW_31_25KHz         4
#define BW_41_7KHz          5
#define BW_62_5KHz          6
#define BW_125KHz           7
#define BW_250KHz           8
#define BW_500KHz           9

/* Coding rate */
#define CR_4_5              1
#define CR_4_6              2
#define CR_4_7              3
#define CR_4_8              4

/* Spreading factors */
#define SF_7                7
#define SF_8                8
#define SF_9                9
#define SF_10               10
#define SF_11               11
#define SF_12               12

/* Power gain */
#define POWER_11db          0xF6
#define POWER_14db          0xF9
#define POWER_17db          0xFC
#define POWER_20db          0xFF

/* Registers - Common */
#define RegFiFo             0x00
#define RegOpMode           0x01
#define RegBitrateMsb       0x02
#define RegBitrateLsb       0x03
#define RegFdevMsb          0x04
#define RegFdevLsb          0x05
#define RegFrMsb            0x06
#define RegFrMid            0x07
#define RegFrLsb            0x08
#define RegPaConfig         0x09
#define RegOcp              0x0B
#define RegLna              0x0C
#define RegRxConfig         0x0D
#define RegDioMapping1      0x40
#define RegDioMapping2      0x41
#define RegVersion          0x42

/* Registers - FSK mode specific */
#define RegPreambleDetect   0x1F
#define RegOsc              0x24
#define RegPreambleMsb_FSK  0x25
#define RegPreambleLsb_FSK  0x26
#define RegSyncConfig       0x27
#define RegSyncValue1       0x28
#define RegPacketConfig1    0x30
#define RegPacketConfig2    0x31
#define RegPayloadLength_FSK 0x32
#define RegFifoThresh       0x35
#define RegIrqFlags1        0x3E
#define RegIrqFlags2        0x3F

/* Registers - LoRa mode specific */
#define RegFiFoAddPtr       0x0D
#define RegFiFoTxBaseAddr   0x0E
#define RegFiFoRxBaseAddr   0x0F
#define RegFiFoRxCurrentAddr 0x10
#define RegIrqFlags         0x12
#define RegRxNbBytes        0x13
#define RegPktRssiValue     0x1A
#define RegModemConfig1     0x1D
#define RegModemConfig2     0x1E
#define RegSymbTimeoutL     0x1F
#define RegPreambleMsb      0x20
#define RegPreambleLsb      0x21
#define RegPayloadLength    0x22
#define RegModemConfig3     0x26
#define RegSyncWord         0x39

/* Status codes */
#define LORA_OK             200
#define LORA_NOT_FOUND      404
#define LORA_LARGE_PAYLOAD  413
#define LORA_UNAVAILABLE    503

/* OpMode flags */
#define OPMODE_LORA         0x80
#define OPMODE_FSK          0x00

/* DIO Mapping for Continuous mode (RegDioMapping1) */
/* DIO1: bits 5-4, DIO2: bits 3-2 */
#define DIO1_CONT_DCLK      0x00    /* DIO1 = DCLK in continuous mode */
#define DIO2_CONT_DATA      0x00    /* DIO2 = DATA in continuous mode */

/* PacketConfig2 (0x31) */
#define DATA_MODE_PACKET    0x40    /* Bit 6 = 1: Packet mode */
#define DATA_MODE_CONTINUOUS 0x00   /* Bit 6 = 0: Continuous mode */

/* RxConfig (0x0D in FSK) - AFC/AGC Trigger */
#define RX_TRIGGER_NONE     0x00    /* No trigger */
#define RX_TRIGGER_RSSI     0x01    /* Trigger on RSSI */

typedef struct LoRa_setting {
    /* Hardware settings */
    GPIO_TypeDef*       CS_port;
    uint16_t            CS_pin;
    GPIO_TypeDef*       reset_port;
    uint16_t            reset_pin;
    GPIO_TypeDef*       DIO0_port;
    uint16_t            DIO0_pin;
    SPI_HandleTypeDef*  hSPIx;

    /* Module settings */
    int                 current_mode;
    int                 frequency;          /* Base RF frequency in MHz */
    uint8_t             spredingFactor;
    uint8_t             bandWidth;
    uint8_t             crcRate;
    uint16_t            preamble;
    uint8_t             power;
    uint8_t             overCurrentProtection;

    /* FSK mode settings */
    uint8_t             is_fsk_mode;        /* 1 if in FSK mode */
} LoRa;

/* Basic functions */
LoRa     newLoRa(void);
void     LoRa_reset(LoRa* _LoRa);
uint16_t LoRa_init(LoRa* _LoRa);
uint8_t  LoRa_read(LoRa* _LoRa, uint8_t address);
void     LoRa_write(LoRa* _LoRa, uint8_t address, uint8_t value);
void     LoRa_gotoMode(LoRa* _LoRa, int mode);

/* Configuration */
void     LoRa_setFrequency(LoRa* _LoRa, int freq);
void     LoRa_setSpreadingFactor(LoRa* _LoRa, int SP);
void     LoRa_setPower(LoRa* _LoRa, uint8_t power);
void     LoRa_setOCP(LoRa* _LoRa, uint8_t current);
void     LoRa_setSyncWord(LoRa* _LoRa, uint8_t syncword);

/* LoRa TX/RX */
uint8_t  LoRa_transmit(LoRa* _LoRa, uint8_t* data, uint8_t length, uint16_t timeout);
void     LoRa_startReceiving(LoRa* _LoRa);
uint8_t  LoRa_receive(LoRa* _LoRa, uint8_t* data, uint8_t length);
int      LoRa_getRSSI(LoRa* _LoRa);

/* FSK Continuous mode for AFSK/SSTV */
uint16_t LoRa_beginFSK(LoRa* _LoRa);
void     LoRa_startDirectMode(LoRa* _LoRa);
void     LoRa_transmitDirect(LoRa* _LoRa);
void     LoRa_standby(LoRa* _LoRa);

#ifdef __cplusplus
}
#endif

#endif /* LORA_H */
