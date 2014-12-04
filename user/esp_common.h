#ifndef __ESP_COMMON_H
#define __ESP_COMMON_H

#define ESP_PARAM_START_SEC   0x3C
#define ESP_PARAM_SAVE_0    1
#define ESP_PARAM_SAVE_1    2
#define ESP_PARAM_FLAG      3

struct esp_platform_sec_flag_param {
  uint8 flag;
  uint8 pad[3];
};

void user_esp_platform_load_param(void *param, uint16_t len);
void user_esp_platform_save_param(void *param, uint16_t len);

#endif
