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
/* uwb.c: Uwb radio implementation, low level handling */

#include "driver/gpio.h"
#include "esp_attr.h"

#include "uwb.h"

#include "libdw1000.h"
#include "dwOps.h"

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
  {NULL, NULL},
};

// Low level radio handling context (to be separated)
static bool isInit = false;
static int uwbErrorCode = 0;
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


void uwbInit()
{
  // Initializing the low level radio handling
  static StaticSemaphore_t irqSemaphoreBuffer;
  irqSemaphore = xSemaphoreCreateBinaryStatic(&irqSemaphoreBuffer);

  dwInit(dwm, &dwOps);       // Init libdw
  dwOpsInit(dwm);

  // Configure DW1000 IRQ pin and register ESP32 ISR
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
  uwbErrorCode = dwConfigure(dwm); // Configure the dw1000 chip
  if (uwbErrorCode == 0) {
    dwEnableAllLeds(dwm);
  } else {
    return;
  }
  dwTime_t delay = {.full = 0};
  dwSetAntenaDelay(dwm, delay);

  // Reading and setting node configuration
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

  dwAttachSentHandler(dwm, txcallback);
  dwAttachReceivedHandler(dwm, rxcallback);
  dwAttachReceiveTimeoutHandler(dwm, rxTimeoutCallback);
  dwAttachReceiveFailedHandler(dwm, rxfailedcallback);

  dwNewConfiguration(dwm);
  dwSetDefaults(dwm);

  uint8_t useLowBitrate = 0;
  cfgReadU8(cfgLowBitrate, &useLowBitrate);
  #ifdef LPS_LONGER_RANGE
  useLowBitrate = 1;
  #endif
  config.lowBitrate = (useLowBitrate == 1);

  uint8_t useLongPreamble = 0;
  cfgReadU8(cfgLongPreamble, &useLongPreamble);
  config.longPreamble = (useLongPreamble == 1);

  const uint8_t* mode = MODE_SHORTDATA_FAST_ACCURACY;
  if (useLowBitrate && !useLongPreamble) {
    mode = MODE_SHORTDATA_MID_ACCURACY;
  } else if (!useLowBitrate && useLongPreamble) {
    mode = MODE_LONGDATA_FAST_ACCURACY;
  } else if (useLowBitrate && useLongPreamble) {
    mode = MODE_LONGDATA_MID_ACCURACY;
  }
  dwEnableMode(dwm, mode);

  dwSetChannel(dwm, CHANNEL_2);

  // Enable smart power by default
  uint8_t enableSmartPower = 1;
  cfgReadU8(cfgSmartPower, &enableSmartPower);
  config.smartPower = enableSmartPower != 0;
  if (enableSmartPower) {
    dwUseSmartPower(dwm, true);
  }

  // Do not force power by default
  uint8_t forceTxPower = 0;
  cfgReadU8(cfgForceTxPower, &forceTxPower);
  config.forceTxPower = forceTxPower != 0;
  if (forceTxPower) {
    uint32_t txPower = 0x1F1F1F1Ful;
    cfgReadU32(cfgTxPower, &txPower);
    config.txPower = txPower;
    dwSetTxPower(dwm, txPower);
  }

  dwSetPreambleCode(dwm, PREAMBLE_CODE_64MHZ_9);

  dwCommitConfiguration(dwm);

  isInit = true;
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
