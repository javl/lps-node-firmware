/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2016, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* uwb.c: UWB radio implementation — DW3000 (DWM3000 module) low-level handling */

#include "driver/gpio.h"
#include "esp_attr.h"

#include "uwb.h"

/* DW3000 driver API */
#include "libdw3000_shim.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "dwOps.h"

/* dwOps.c exposes these helpers so we can bracket dwt_initialise() with the
 * correct SPI speed (DW3000 requires <= 7 MHz during initialise).         */
extern void dwSpiSetSpeedSlow(void);
extern void dwSpiSetSpeedFast(void);

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// Implemented UWB algoritm. The dummy one is at the end of this file.
static uwbAlgorithm_t dummyAlgorithm;
extern uwbAlgorithm_t uwbTwrAnchorAlgorithm;
extern uwbAlgorithm_t uwbTwrTagAlgorithm;
extern uwbAlgorithm_t uwbSnifferAlgorithm;
extern uwbAlgorithm_t uwbTdoaAlgorithm;
extern uwbAlgorithm_t uwbTdoa2Algorithm;
extern uwbAlgorithm_t uwbTdoa3Algorithm;
extern uwbAlgorithm_t uwbTdoa2TagAlgorithm;
static uwbAlgorithm_t *algorithm = &dummyAlgorithm;

/* Forward declaration — defined later in this file */
static void IRAM_ATTR dw_isr_handler(void *arg);

struct {
  uwbAlgorithm_t *algorithm;
  char *name;
} availableAlgorithms[] = {
  {.algorithm = &uwbTwrAnchorAlgorithm, .name = "TWR Anchor"},
  {.algorithm = &uwbTwrTagAlgorithm,    .name = "TWR Tag"},
  {.algorithm = &uwbSnifferAlgorithm,   .name = "Sniffer"},
  {.algorithm = &uwbTdoa2Algorithm,     .name = "TDoA Anchor V2"},
  {.algorithm = &uwbTdoa3Algorithm,     .name = "TDoA Anchor V3"},
  {.algorithm = &uwbTdoa2TagAlgorithm,  .name = "TDoA Tag V2"},
  {NULL, NULL},
};

// Low level radio handling context (to be separated)
static bool isInit = false;
static int uwbErrorCode = 0;
static const char *uwbInitFailStage = "uwbInit() not yet called";
static SemaphoreHandle_t irqSemaphore;
static dwDevice_t dwm_device;
static dwDevice_t *dwm = &dwm_device;

// System configuration
static struct uwbConfig_s config = {
  address: {0,0,0,0,0,0,0xcf,0xbc},
};

static uint32_t timeout;

static void txcallback(dwDevice_t *dev)
{
  timeout = algorithm->onEvent(dev, eventPacketSent);
}

static void rxcallback(dwDevice_t *dev)
{
  timeout = algorithm->onEvent(dev, eventPacketReceived);
}

static void rxTimeoutCallback(dwDevice_t * dev) {
  timeout = algorithm->onEvent(dev, eventReceiveTimeout);
}

static void rxfailedcallback(dwDevice_t *dev) {
  timeout = algorithm->onEvent(dev, eventReceiveFailed);
}


/* -----------------------------------------------------------------------
 * DW3000 default antenna delay (RX and TX, same value).
 *
 * 16436 is the Qorvo reference value for the DWM3000 module at 64 MHz PRF
 * on Channel 5 (units: DW timestamp ticks, 1 tick ≈ 15.65 ps).
 *
 * Calibrate against a known reference distance: increasing this value
 * increases the reported range by approximately 1 mm per tick.
 * -----------------------------------------------------------------------*/
#define DW3000_ANT_DLY 16436U

void uwbInit()
{
  /* ---- FreeRTOS semaphore for ISR→task signalling -------------------- */
  static StaticSemaphore_t irqSemaphoreBuffer;
  irqSemaphore = xSemaphoreCreateBinaryStatic(&irqSemaphoreBuffer);

  /* ---- 1. Hardware init: SPI bus + CS / RST / WAKEUP GPIOs ----------- */
  dwOpsInit(dwm);

  /* ---- 2. Hardware reset per DW3000 datasheet ------------------------
   * Drive RST low, then release to high-Z so the internal pull-up controls
   * the line.  Never drive RST high from an output — it can damage the IC.
   */
  #if PIN_DW_RST
  gpio_set_direction(PIN_DW_RST, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_DW_RST, 0);
  vTaskDelay(pdMS_TO_TICKS(1));
  gpio_set_direction(PIN_DW_RST, GPIO_MODE_INPUT);   /* release → high-Z */
  vTaskDelay(pdMS_TO_TICKS(5));                       /* PLL lock margin  */
  #endif

  /* ---- 2b. SPI sanity check: verify DEV_ID before doing anything ------
   * This mirrors the checkForDevID() pattern used in DWM3000.cpp.
   * If SPI wiring is broken or the wrong pins are configured every read
   * returns 0x00000000 or 0xFFFFFFFF; dwt_checkidlerc() would then spin
   * for the full 500 ms before failing with a misleading "IDLE_RC" error.
   * Checking DEV_ID here gives an immediate, actionable diagnostic.
   *
   * All known production DW3000 silicon:
   *   C0 non-PDOA  0xDECA0302
   *   C0 PDOA      0xDECA0312
   *   B0 non-PDOA  0xDECA0301   B0 PDOA  0xDECA0311
   *   A0 non-PDOA  0xDECA0300   A0 PDOA  0xDECA0310
   */
  {
    uint32_t dev_id = dwt_readdevid();
    int id_ok = (dev_id == DWT_C0_DEV_ID)      || (dev_id == DWT_C0_PDOA_DEV_ID) ||
                (dev_id == DWT_B0_DEV_ID)      || (dev_id == DWT_B0_PDOA_DEV_ID) ||
                (dev_id == DWT_A0_DEV_ID)      || (dev_id == DWT_A0_PDOA_DEV_ID);
    if (!id_ok) {
      printf("UWB\t: DEV_ID mismatch -- SPI failure or wrong pins? "
             "Read 0x%08lX, expected 0xDECA03xx\r\n", (unsigned long)dev_id);
      uwbInitFailStage = "DEV_ID mismatch (SPI wiring problem?)";
      uwbErrorCode = DWT_ERROR;
      return;
    }
    printf("UWB\t: DEV_ID OK (0x%08lX)\r\n", (unsigned long)dev_id);
  }

  /* ---- 3. Wait for DW3000 to reach IDLE_RC state --------------------- */
  int idleWait = 0;
  while (!dwt_checkidlerc()) {
    vTaskDelay(1);
    if (++idleWait > 500) {
      printf("UWB\t: DW3000 did not reach IDLE_RC within 500 ms\r\n");
      uwbInitFailStage = "IDLE_RC timeout (chip not responding after reset)";
      uwbErrorCode = DWT_ERROR;
      return;
    }
  }

  /* ---- 4. Initialise chip (OTP calibration, LDO/bias/XTAL trim) ------
   * SPI must be <= 7 MHz here; dwOpsInit starts at 2 MHz (slow).
   */
  uwbErrorCode = dwt_initialise(DWT_DW_INIT);
  if (uwbErrorCode != DWT_SUCCESS) {
    printf("UWB\t: dwt_initialise failed (%d)\r\n", uwbErrorCode);
    uwbInitFailStage = "dwt_initialise() failed (OTP/LDO/XTAL calibration error)";
    return;
  }

  /* ---- 5. Switch to fast SPI for all subsequent operations ----------- */
  dwSpiSetSpeedFast();

  /* ---- 6. Read bitrate / preamble preferences from NVS --------------- */
  uint8_t useLowBitrate = 0;
  cfgReadU8(cfgLowBitrate, &useLowBitrate);
#ifdef LPS_LONGER_RANGE
  useLowBitrate = 1;
#endif
  config.lowBitrate = (useLowBitrate == 1);

  uint8_t useLongPreamble = 0;
  cfgReadU8(cfgLongPreamble, &useLongPreamble);
  config.longPreamble = (useLongPreamble == 1);

  /* ---- 7. Build DW3000 radio config ----------------------------------
   * DW3000 supports only channels 5 and 9; channel 2 used by the DW1000
   * code is not available.  Channel 5 (6.49 GHz) is used here because
   * the DWM3000 PCB antenna is matched for it.
   *
   * 110 kbps (DW1000 low-range mode) does not exist in DW3000; use 850K
   * as the nearest low-bitrate option.
   *
   * Preamble codes 9–12 are valid for channel 5 in DW3000.
   */
  dwt_config_t dw3000_cfg = {
    .chan           = 5,
    .txPreambLength = config.longPreamble ? DWT_PLEN_1024 : DWT_PLEN_128,
    .rxPAC          = config.longPreamble ? DWT_PAC32     : DWT_PAC8,
    .txCode         = 9,
    .rxCode         = 9,
    /* DecaWave 8-symbol enhanced SFD (sfdType=1) provides better noise
     * immunity than the IEEE standard SFD (sfdType=0) at the cost of a
     * slightly longer preamble.                                         */
    .sfdType        = 1,
    .dataRate       = config.lowBitrate ? DWT_BR_850K : DWT_BR_6M8,
    .phrMode        = DWT_PHRMODE_STD,
    .phrRate        = DWT_PHRRATE_STD,
    /* SFD timeout = preamble symbols + 1 + SFD length - PAC size.
     * For DWT_PLEN_128 / DWT_PAC8:   129 + 1 + 8 - 8  = 130  (≈ 129 used)
     * For DWT_PLEN_1024 / DWT_PAC32: 1025 + 1 + 8 - 32 = 1002          */
    .sfdTO          = config.longPreamble ? (1025 + 8 - 32) : (129 + 8 - 8),
    .stsMode        = DWT_STS_MODE_OFF,   /* no Scrambled Timestamp Seq  */
    .stsLength      = DWT_STS_LEN_64,     /* irrelevant when STS is off  */
    .pdoaMode       = DWT_PDOA_M0,        /* no Phase Difference of Arrival */
  };

  if (dwt_configure(&dw3000_cfg) != DWT_SUCCESS) {
    printf("UWB\t: dwt_configure failed — PLL or RX calibration error\r\n");
    uwbInitFailStage = "dwt_configure() failed (PLL or RX calibration error)";
    uwbErrorCode = DWT_ERROR;
    return;
  }

  /* ---- 8. TX spectrum configuration (Channel 5 room-temperature defaults)
   * PG delay 0x34 and TX power 0xfdfdfdfd match the Qorvo recommended
   * values for DW3000 channel 5.  Calibrate PGcount via OTP if available.
   */
  dwt_txconfig_t dw3000_txcfg = {
    .PGdly   = 0x34,
    .power   = 0xfdfdfdfd,
    .PGcount = 0,            /* 0 = use PGdly directly, no auto-calibration */
  };

  /* ---- 9. Custom TX power from NVS (overrides the default above) ----- */
  uint8_t forceTxPower = 0;
  cfgReadU8(cfgForceTxPower, &forceTxPower);
  config.forceTxPower = forceTxPower != 0;
  if (forceTxPower) {
    uint32_t txPower = 0xfdfdfdfd;
    cfgReadU32(cfgTxPower, &txPower);
    config.txPower  = txPower;
    dw3000_txcfg.power = txPower;
  }
  dwt_configuretxrf(&dw3000_txcfg);

  /* Smart TX power is a DW1000-specific feature (automatic power reduction
   * for short preambles).  DW3000 does not have an equivalent; record the
   * setting in config for compatibility but do nothing hardware-side.    */
  uint8_t enableSmartPower = 1;
  cfgReadU8(cfgSmartPower, &enableSmartPower);
  config.smartPower = enableSmartPower != 0;

  /* ---- 10. Antenna delay --------------------------------------------- */
  dwt_setrxantennadelay(DW3000_ANT_DLY);
  dwt_settxantennadelay(DW3000_ANT_DLY);

  /* ---- 11. Enable on-chip LEDs for debug ----------------------------- */
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  /* ---- 12. Read node address, mode, anchor list from NVS ------------- */
  cfgReadU8(cfgAddress, &config.address[0]);
  cfgReadU8(cfgMode, &config.mode);
  cfgFieldSize(cfgAnchorlist, &config.anchorListSize);
  if (config.anchorListSize <= MAX_ANCHORS) {
    cfgReadU8list(cfgAnchorlist, config.anchors, config.anchorListSize);
  }

  if (config.mode < uwbAlgorithmCount()) {
    algorithm = availableAlgorithms[config.mode].algorithm;
  } else {
    algorithm = &dummyAlgorithm;
  }

  config.positionEnabled = cfgReadFP32list(cfgAnchorPos, config.position, 3);

  /* ---- 13. Register shim callbacks -----------------------------------
   * dwInit() stores dwm as the global device pointer and calls
   * dwt_setcallbacks() with the shim's internal ISR forwarders.
   * dwAttach*Handler() then stores the application-level callbacks
   * (txcallback, rxcallback, …) into dwm for forwarding.
   */
  dwInit(dwm, NULL);
  dwAttachSentHandler(dwm, txcallback);
  dwAttachReceivedHandler(dwm, rxcallback);
  dwAttachReceiveTimeoutHandler(dwm, rxTimeoutCallback);
  dwAttachReceiveFailedHandler(dwm, rxfailedcallback);

  /* ---- 14. Enable DW3000 interrupt sources ---------------------------- */
  dwt_setinterrupt(
      SYS_ENABLE_LO_TXFRS_ENABLE_BIT_MASK  |  /* TX frame sent             */
      SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK  |  /* RX good frame (CRC ok)    */
      SYS_ENABLE_LO_RXFTO_ENABLE_BIT_MASK  |  /* RX frame timeout          */
      SYS_ENABLE_LO_RXPTO_ENABLE_BIT_MASK  |  /* RX preamble timeout       */
      SYS_ENABLE_LO_RXPHE_ENABLE_BIT_MASK  |  /* RX PHY header error       */
      SYS_ENABLE_LO_RXFCE_ENABLE_BIT_MASK,    /* RX FCS error              */
      0,                                       /* hi-word mask (unused)     */
      DWT_ENABLE_INT);

  /* ---- 15. Configure IRQ GPIO and register ESP32 ISR ----------------- */
  {
    gpio_config_t irq_cfg = {
      .pin_bit_mask = (1ULL << PIN_DW_IRQ),
      .mode         = GPIO_MODE_INPUT,
      .pull_up_en   = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type    = GPIO_INTR_POSEDGE,
    };
    gpio_config(&irq_cfg);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_DW_IRQ, dw_isr_handler, NULL);
  }

  isInit = true;
  uwbInitFailStage = NULL;   /* success — no failure */
}

bool uwbTest()
{
  return isInit;
}

int uwbAlgorithmCount()
{
  int count = 0;

  while (availableAlgorithms[count].algorithm != NULL) {
    count ++;
  }
  return count;
}

char * uwbAlgorithmName(unsigned int id)
{
  if (id < uwbAlgorithmCount()) {
    return availableAlgorithms[id].name;
  } else {
    return "UKNOWN";
  }
}

/* ESP32 ISR handler — called on rising edge of PIN_DW_IRQ */
static void IRAM_ATTR dw_isr_handler(void *arg)
{
  BaseType_t higherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(irqSemaphore, &higherPriorityTaskWoken);
  portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

static int checkIrq()
{
  return gpio_get_level(PIN_DW_IRQ);
}

static void uwbTask(void* parameters)
{
  configASSERT(isInit);

  algorithm->init(&config, dwm);

  while(1) {
    if (xSemaphoreTake(irqSemaphore, timeout/portTICK_PERIOD_MS)) {
      do{
          dwHandleInterrupt(dwm);
      } while(checkIrq() != 0);
    } else {
      timeout = algorithm->onEvent(dwm, eventTimeout);
    }
  }
}

void uwbStart()
{
  static StaticTask_t uwbStaticTask;
  static StackType_t uwbStaticStack[2*configMINIMAL_STACK_SIZE];

  if (isInit) {
    xTaskCreateStatic(uwbTask, "uwb", 2*configMINIMAL_STACK_SIZE, NULL,
                      configMAX_PRIORITIES - 1, uwbStaticStack, &uwbStaticTask);
  }
}

char * uwbStrError()
{
  return dwStrError(uwbErrorCode);
}

char * uwbInitStage()
{
  return (char *)(uwbInitFailStage ? uwbInitFailStage : "(none)");
}

struct uwbConfig_s * uwbGetConfig()
{
  return &config;
}

/**** DWM1000 interrupt handling *****/
/**** DWM1000 interrupt handling *****/

/* Dummy algorithm (used if UKNOWN algorithm is selected ...)*/
static void dummyInit(uwbConfig_t * config, dwDevice_t *dev)
{
  ;
}

static uint32_t dummyOnEvent(dwDevice_t *dev, uwbEvent_t event)
{
  return MAX_TIMEOUT;
}

static uwbAlgorithm_t dummyAlgorithm = {
  .init = dummyInit,
  .onEvent = dummyOnEvent,
};
