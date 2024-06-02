#include <stdio.h>
#include <bq76930.h>
#include "driver/i2c.h"
#include "esp_log.h"

int sda_pin = 5;
int scl_pin = 4;

int boot_pin = 6;
int alert_pin = 7;

int address = 0x08;

bq76930 bms(address, sda_pin, scl_pin);

extern "C" { void app_main(void);}

void update_task(void *pvParameter) {
    while(1) {
        bms.update();
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

void app_main() {
    bms.initialize(alert_pin, boot_pin);
    bms.setShuntResistorValue(5);
    bms.setOvercurrentChargeProtection(5000);
    bms.setCellUndervoltageProtection(3200, 2);
    bms.setCellOvervoltageProtection(4240, 2);
    bms.setBalancingThresholds(0, 3700, 15);
    bms.setIdleCurrentThreshold(100);
    bms.toggleAutoBalancing(true);
    
    if(bms.toggleCharging(true)) {
        ESP_LOGI("BMS", "Charging toggled");
    } else {
        ESP_LOGI("BMS", "Charging not allowed");
    }

    xTaskCreate(&update_task, "update_task", 2048, NULL, 5, NULL);

    while(1) {
        printf("cell1: %d\n", bms.getCellVoltage(1));
        printf("cell2: %d\n", bms.getCellVoltage(2));
        printf("cell3: %d\n", bms.getCellVoltage(3));
        printf("cell4: %d\n", bms.getCellVoltage(4));
        printf("cell5: %d\n", bms.getCellVoltage(5));
        printf("cell6: %d\n", bms.getCellVoltage(6));
        printf("cell7: %d\n", bms.getCellVoltage(7));
        printf("cell8: %d\n", bms.getCellVoltage(8));
        printf("cell9: %d\n", bms.getCellVoltage(9));
        printf("cell10: %d\n", bms.getCellVoltage(10));
        printf("Voltage: %d mV\n", bms.getBatteryVoltage());
        printf("current: %d mA\n", bms.getBatteryCurrent());
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


