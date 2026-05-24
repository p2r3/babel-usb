/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"

int init_hardware(void);
int init_software(void);
int init_tasks(void);

void app_main(void)
{
  init_hardware();
  init_software();
  init_tasks();
}

#include "monolith/usb_mtp_impl.c.h"
#include "monolith/usb_descriptors.c.h"

void TaskTinyusb(void *pvParameters)
{
  (void) pvParameters;
  while (1) {
    tud_task();
  }
}
