#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "Peripheral.h"
#include "hal_bsp_sht20.h"
#include "hal_bsp_log.h"
#include "hal_bsp_ap3216c.h"
#include "hal_bsp_ssd1306.h"
#include "nfc.h"
#include "iot_gpio.h"
#include "hi_io.h"
#include "iot_errno.h"
#include "iot_i2c.h"
#include "hi_i2c.h"
#include "hi_adc.h"

#include "hal_bsp_nfc.h"

#include "config.h"
#include "mpu6050.h"

/* ========== 全局变量 ========== */
uint8_t displayBuff_0[20] = {0};
uint8_t displayBuff_1[20] = {0};
uint8_t displayBuff_2[20] = {0};
uint8_t displayBuff_3[20] = {0};

osSemaphoreId_t sem1;           // 定义信号量

float temperature = 0;          // 温度数据
float humidity = 0;             // 湿度数据

uint16_t ir = 0;                // 人体红外传感器
uint16_t als = 0;               // 光照强度传感器
uint16_t ps = 0;                // 接近传感器

hi_s16 mpu_dates[7];
hi_u8 mpu_ds[14] = {0};
hi_u8 mpu_ln = 14;

#define IoT_KEY2_GPIO_13        13

/* ========== ADC 电压获取函数 ========== */
float Get_Voltage(void)
{
    unsigned int ret;
    unsigned short data;

    ret = hi_adc_read(HI_ADC_CHANNEL_6, &data, HI_ADC_EQU_MODEL_8, HI_ADC_CUR_BAIS_DEFAULT, 0xff);
    if (ret != IOT_SUCCESS) {
        return 0;
    }

    return (float)data * 1.8 * 4 / 4096.0;
}

/* ========== 任务1：读取所有传感器 + 释放信号量 ========== */
static void Stage2_Task1(void)
{
    float Voltage;

    Peripheral_Init();  // 外设功能初始化（GPIO + OLED + I2C）
    hi_io_set_pull(IoT_KEY2_GPIO_13, HI_IO_PULL_UP); // 上拉，让按键未按下时 GPIO_13 保持高电平

    SSD1306_CLS();      // 清屏

    Mpu6050_Init();     // MPU6050 加速度传感器初始化

    while (1) {
        /* 释放 sem1 信号量，唤醒 Task2 */
        osSemaphoreRelease(sem1);

        /* 读取 SHT20 温湿度传感器 */
        SHT20_ReadData(&temperature, &humidity);
        printf("temperature = %.2f; humidity = %.2f\r\n", temperature, humidity);

        /* 读取 AP3216C 三合一传感器 */
        AP3216C_ReadData(&ir, &als, &ps);
        printf("ir = %d; als = %d; ps = %d\r\n", ir, als, ps);

        /* 读取 MPU6050 加速度 + 陀螺仪 */
        Mpu6050_Measure_By(mpu_ds, mpu_ln);   // iic 读取初始化
        Mpu6050_Measure_Sh(mpu_dates);         // iic 信息采集
        printf("accel x=%d, y=%d, z=%d, gyro x=%d, y=%d, z=%d\r\n",
               mpu_dates[0], mpu_dates[1], mpu_dates[2],
               mpu_dates[3], mpu_dates[4], mpu_dates[5]);

        /* 读取 ADC 电压 */
        Voltage = Get_Voltage();
        printf("Voltage = %.3fV\r\n", Voltage);

        /* 保存到对应显示缓冲区 */
        memset(displayBuff_0, 0, sizeof(displayBuff_0));
        memset(displayBuff_1, 0, sizeof(displayBuff_1));
        memset(displayBuff_2, 0, sizeof(displayBuff_2));
        memset(displayBuff_3, 0, sizeof(displayBuff_3));

        snprintf((char *)displayBuff_0, sizeof(displayBuff_0),
                 "T:%.1fC H:%.1f%%", temperature, humidity);
        snprintf((char *)displayBuff_1, sizeof(displayBuff_1),
                 "ALS:%d PS:%d", als, ps);
        snprintf((char *)displayBuff_2, sizeof(displayBuff_2),
                 "AC:%d,%d,%d", mpu_dates[0], mpu_dates[1], mpu_dates[2]);
        snprintf((char *)displayBuff_3, sizeof(displayBuff_3),
                 "V:%.2fV", Voltage);

        sleep(3);
    }
}

/* ========== 任务2：OLED 显示传感器数据 ========== */
static void Stage2_Task2(void)
{
    sleep(3);
    SSD1306_CLS();  // 清屏

    while (1) {
        /* 等待 sem1 信号量 */
        osSemaphoreAcquire(sem1, osWaitForever);

        /* 温湿度显示在第 1 行 */
        SSD1306_ShowStr(0, 0, (uint8_t *)displayBuff_0, 16);

        /* 光照 + 接近传感器显示在第 2 行 */
        SSD1306_ShowStr(0, 1, (uint8_t *)displayBuff_1, 16);

        /* 加速度数据显示在第 3 行 */
        SSD1306_ShowStr(0, 2, (uint8_t *)displayBuff_2, 16);

        /* 电压显示在第 4 行 */
        SSD1306_ShowStr(0, 3, (uint8_t *)displayBuff_3, 16);
    }
}

/* ========== 任务3：NFC 读取 ========== */
static void Stage2_Task3(void)
{
    uint8_t ndefLen = 0;         // ndef 包的长度
    uint8_t ndef_Header = 0;     // ndef 消息开始标志位

    uint8_t ssd[20] = {0};
    uint8_t secret[20] = {0};
    uint8_t Keep_Ssd_Flag = 0;
    uint8_t Keep_Sec_Flag = 0;
    uint8_t Keep_Ssd_long = 0;
    uint8_t Keep_Sec_long = 0;

    sleep(1);

    /* NFC 初始化 */
    if (nfc_Init() != true) {
        printf("nfc_Init Failed\n");
    }

    /* 读整个数据的包头部分，读出整个数据的长度 */
    if (NT3HReadHeaderNfc(&ndefLen, &ndef_Header) == 0) {
        printf("NT3HReadHeaderNfc Failed\n");
        return;
    }

    /* 加上头部字节 */
    ndefLen += NDEF_HEADER_SIZE;

    printf("ndefLen=%d\n", ndefLen);

    /* 申请内存 */
    uint8_t *ndefBuff = (uint8_t *)malloc(ndefLen + 1);
    if (ndefBuff == NULL) {
        printf("ndefBuff malloc Falied!\n");
        return;
    }

    /* 获取 NFC 中的数据包 */
    if (get_NDEFDataPackage(ndefBuff, ndefLen) != 0) {
        printf("get_NDEFDataPackage Failed\n");
        free(ndefBuff);
        return;
    }

    /* 将数据包中以 "n" 之后的字符为开始的 ssid 字段至 "," 之前为止 */
    for (size_t i = 0; i < ndefLen; i++) {
        if (0x6e == ndefBuff[i]) {
            Keep_Ssd_Flag = i + 1;
            break;
        }
    }

    /* 将数据包中以 "," 为分割线标记记号 */
    for (size_t i = 0; i < ndefLen; i++) {
        if (0x2c == ndefBuff[i]) {
            Keep_Sec_Flag = i + 1;
            break;
        }
    }

    /* 提取账号，密码长度 */
    Keep_Ssd_long = Keep_Sec_Flag - Keep_Ssd_Flag - 1;
    Keep_Sec_long = ndefLen - Keep_Sec_Flag;

    /* 将账号存入数组 */
    for (size_t i = 0; i < Keep_Ssd_long; i++) {
        ssd[i] = ndefBuff[i + Keep_Ssd_Flag];
    }
    /* 将密码存入数组 */
    for (size_t i = 0; i < Keep_Sec_long; i++) {
        secret[i] = ndefBuff[i + Keep_Sec_Flag];
    }

    printf("\n");
    printf("Keep_Ssd_Flag:%s\n", ssd);
    printf("Keep_Sec_Flag:%s\n", secret);

    free(ndefBuff);

    while (1) {
        sleep(1);
    }
}

/* ========== 主入口：创建 3 个任务 + 信号量 ========== */
static void Experiment_Stage2(void)
{
    osThreadAttr_t attr;
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 4;

    attr.name = "Stage2_Task1";
    attr.priority = 25;
    if (osThreadNew((osThreadFunc_t)Stage2_Task1, NULL, &attr) == NULL) {
        printf("Falied to create Stage2_Task1!\n");
    }

    attr.name = "Stage2_Task2";
    attr.priority = 25;
    if (osThreadNew((osThreadFunc_t)Stage2_Task2, NULL, &attr) == NULL) {
        printf("Falied to create Stage2_Task2!\n");
    }

    attr.name = "Stage2_Task3";
    attr.priority = 25;
    if (osThreadNew((osThreadFunc_t)Stage2_Task3, NULL, &attr) == NULL) {
        printf("Falied to create Stage2_Task3!\n");
    }

    sem1 = osSemaphoreNew(4, 0, NULL);  // 创建信号量初始值为 0，最大值为 4
    if (sem1 == NULL) {
        printf("Falied to create Semaphore1!\n");
    }
}

APP_FEATURE_INIT(Experiment_Stage2);
