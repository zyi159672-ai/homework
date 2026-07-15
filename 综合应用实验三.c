#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifi_connect.h"
#include "lwip/sockets.h"
#include "oc_mqtt.h"
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
#include "iot_pwm.h"
#include "hi_pwm.h"
#include "iot_uart.h"
#include "hi_uart.h"
#include "cJSON.h"
#include "iot_flash.h"

#include "hal_bsp_nfc.h"
#include "hal_bsp_log.h"

#include "config.h"
#include "mpu6050.h"

#define MSGQUEUE_OBJECTS 16 // number of Message Queue Objects

#define CLIENT_ID "6a55e3ac7f2e6c302f8505d8_cold_chain_001_0_0_2026071408"
#define USERNAME "6a55e3ac7f2e6c302f8505d8_cold_chain_001"
#define PASSWORD "8e01cc2922351d2cb439835e2ebc698e6b580816bcc50c26da320c3ea95ac99d"

#define UART_BUFF_SIZE 1000//串口发送数据大小

uint8_t uart_buff[UART_BUFF_SIZE] = {0};
uint8_t *uart_buff_ptr = uart_buff;


uint8_t displayBuff_0[20] = {0};
uint8_t displayBuff_1[20] = {0};
uint8_t displayBuff_2[20] = {0};
uint8_t displayBuff_3[20] = {0};
uint8_t displayBuff_4[20] = {0};


uint8_t ssd[20]={0};
uint8_t secret[20]={0};
uint8_t Keep_Ssd_Flag = 0;
uint8_t Keep_Sec_Flag = 0;
uint8_t Keep_Ssd_long = 0;
uint8_t Keep_Sec_long = 0;

typedef struct{
    char *Buf;
    uint8_t Idx;
} MSGQUEUE_OBJ_t;

typedef enum{
    en_msg_cmd = 0,
    en_msg_report,
} en_msg_type_t;

typedef struct{
    char *request_id;
    char *payload;
} cmd_t;

typedef struct{
    int lum;
    int temp;
    int hum;
    char Longitude[10];
    char Latitude[9];
} report_t;

typedef struct{
    en_msg_type_t msg_type;//定义en_msg_type_t结构体别名为msg_type

    //定义联合体并定义为msg(数据共用同一段内存，内存大小为最大数据所占内存)
    union{
        cmd_t cmd;//结构体别名创建为cmd
        report_t report;//结构体别名创建为report
    } msg;
} app_msg_t;

typedef struct{
    int connected;
    int led;
    int motor;
} app_cb_t;

typedef struct{
    volatile uint16_t latitude;
    volatile uint16_t latitude_Min;
    volatile uint16_t latitude_Sec;
    volatile uint16_t longitude;
    volatile uint16_t longitude_Min;
    volatile uint16_t longitude_Sec;
    uint8_t flags;
    uint8_t Northern;
    uint8_t East;
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
} GPS_INFO;
GPS_INFO GPS_Store;

static app_cb_t g_app_cb;
MSGQUEUE_OBJ_t msg;
osMessageQueueId_t mid_MsgQueue; // 消息队列ID
float temperature = 0,humidity = 0;//温湿度数据
uint16_t ir = 0,als = 0,ps = 0;   // 人体红外传感器 接近传感器 光照强度传感器


osMutexId_t mutex_id;
app_msg_t *app_msg_down = NULL;
app_msg_t *app_msg_up = NULL;;
char *app_msg_request_id = NULL;

uint32_t exec1;
uint32_t timerDelay_1;
osTimerId_t id1;

/***************************************************************
* 函数名称: Timer1_Callback
* 说    明: MPU6050 定时器回调，关闭蜂鸣器
***************************************************************/
void Timer1_Callback()
{
    //(void)arg;
    IoTGpioSetOutputVal(HI_IO_NAME_GPIO_0,IOT_GPIO_VALUE1);
    //IoTGpioSetOutputVal(HI_IO_NAME_GPIO_8,IOT_GPIO_VALUE0);

    osTimerStop(id1);
}

/***************************************************************
* 函数名称: GPS_Parameters
* 说    明: GPS 串口数据解析函数
***************************************************************/
void GPS_Parameters()
{
    IoTUartRead(HI_UART_IDX_2,uart_buff_ptr,UART_BUFF_SIZE);
    //printf("uart_buff_ptr:[%s]\n",uart_buff_ptr);

    if(strstr((char*)uart_buff_ptr,"GPRMC") == NULL){
        memset(uart_buff_ptr,0,UART_BUFF_SIZE);
    }

    if(strstr((char*)uart_buff_ptr,"GPRMC") != NULL){


        IoTUartRead(HI_UART_IDX_2,uart_buff_ptr,UART_BUFF_SIZE);
        GPS_Store.longitude = ((uart_buff_ptr[3] -0X30)*10+(uart_buff_ptr[4] - 0X30));
        GPS_Store.longitude_Min = ((uart_buff_ptr[5] -0X30)*10+(uart_buff_ptr[6] - 0X30));

        if(uart_buff_ptr[1] == 'A'){
            GPS_Store.flags =1;
        }else{
            GPS_Store.flags =0;
        }



            IoTUartRead(HI_UART_IDX_2,uart_buff_ptr,UART_BUFF_SIZE);


            if(uart_buff_ptr[12] == 'E'){
                GPS_Store.East = 1;

                printf("东经");
                printf("%d度%01d分\n",GPS_Store.latitude,GPS_Store.latitude_Min);
            }else if(uart_buff_ptr[12] == 'W'){
                GPS_Store.East = 0;

                printf("西经");
                printf("%d度%01d分\n",GPS_Store.latitude,GPS_Store.latitude_Min);
            }

            //IoTUartRead(HI_UART_IDX_2,uart_buff_ptr,UART_BUFF_SIZE);
            //IoTUartRead(HI_UART_IDX_2,uart_buff_ptr,UART_BUFF_SIZE);
    }

    memset(uart_buff_ptr,0,UART_BUFF_SIZE);
}

/***************************************************************
* 函数名称: deal_report_msg
* 说    明: 上传数据到华为云平台（包含全部属性）
***************************************************************/
static void deal_report_msg(report_t *report)
{
    oc_mqtt_profile_service_t service;
    oc_mqtt_profile_kv_t temperature;
    oc_mqtt_profile_kv_t humidity;
    oc_mqtt_profile_kv_t motor;
    oc_mqtt_profile_kv_t Longitude_value;
    oc_mqtt_profile_kv_t Latitude_value;

    service.event_time = NULL;
    service.service_id = "Environment";
    service.service_property = &temperature;
    service.nxt = NULL;

    temperature.key = "Temperature";
    temperature.value = &report->temp;
    temperature.type = EN_OC_MQTT_PROFILE_VALUE_INT;
    temperature.nxt = &humidity;

    humidity.key = "Humidity";
    humidity.value = &report->hum;
    humidity.type = EN_OC_MQTT_PROFILE_VALUE_INT;
    humidity.nxt = &Longitude_value;

    Longitude_value.key = "Longitude";
    Longitude_value.value = &report->Latitude;
    Longitude_value.type = EN_OC_MQTT_PROFILE_VALUE_STRING;
    Longitude_value.nxt = &Latitude_value;

    Latitude_value.key = "Latitude";
    Latitude_value.value = &report->Longitude;
    Latitude_value.type = EN_OC_MQTT_PROFILE_VALUE_STRING;
    Latitude_value.nxt = &motor;

    motor.key = "MotorStatus";
    motor.value = g_app_cb.motor ? "ON" : "OFF";
    motor.type = EN_OC_MQTT_PROFILE_VALUE_STRING;
    motor.nxt = NULL;

    //将整理好的数据值打包好
    oc_mqtt_profile_propertyreport(USERNAME, &service);

    return;
}

/***************************************************************
* 函数名称: oc_cmd_get_request_id_len
***************************************************************/
int oc_cmd_get_request_id_len(uint8_t *topic_data,size_t topic_size)
{
    uint8_t *p;

    p = strchr(topic_data,'=');
    if(p != NULL){
        return (topic_size - ( p - topic_data) - 1);
    }else{
        return 0;
    }

}

/***************************************************************
* 函数名称: oc_cmd_rsp_cb
* 说    明: 命令响应回调函数，有命令下发时进入
***************************************************************/
void oc_cmd_rsp_cb(uint8_t *topic_data,size_t topic_size,uint8_t *recv_data,size_t recv_size,uint8_t **resp_data,size_t *resp_size)
{
    int ret = 0;//创建消息队列发送情况变量
    int len = 0;

    //printf("topic:[%d]\n[%s]\n",topic_size,topic_data);
    //printf("json:[%d]\n[%s]\n",recv_size,recv_data);

    len  = oc_cmd_get_request_id_len(topic_data,topic_size);
    //printf("len[%d]\n",len);

    app_msg_request_id = malloc(len + 1);
    memset(app_msg_request_id,0x00,len + 1);
    memcpy(app_msg_request_id,topic_data + topic_size - len,len);
    //printf("app_msg_request_id[%s]\n",app_msg_request_id);

    app_msg_down = malloc(sizeof(app_msg_t));//申请内存
    app_msg_down->msg_type = en_msg_cmd;//设置消息类型为命令
    app_msg_down->msg.cmd.payload = malloc(recv_size + 1);//拷贝payload（recv_data在回调返回后失效）
    memcpy(app_msg_down->msg.cmd.payload, recv_data, recv_size);
    app_msg_down->msg.cmd.payload[recv_size] = '\0';
    app_msg_down->msg.cmd.request_id = app_msg_request_id;//获取request_id

    printf("Recv:[%d][%s]\n",recv_size,recv_data);//打印接收到的命令

    //发送消息队列
    ret = osMessageQueuePut(mid_MsgQueue, &app_msg_down, 0U, 0U);
    if(ret != 0){
        free(app_msg_down);
        free(app_msg_request_id);
    }

    sleep(1);

    *resp_data = NULL;//初始化为0
    *resp_size = 0;//出始化为0
}

/***************************************************************
* 函数名称: deal_cmd_msg
* 说    明: 处理华为云平台下发的命令
***************************************************************/
static void deal_cmd_msg(cmd_t *cmd)
{
    cJSON *obj_root;
    cJSON *obj_cmdname;
    cJSON *obj_paras;
    cJSON *obj_para;
    int cmdret = 1;

    oc_mqtt_profile_cmdresp_t cmdresp;

    obj_root = cJSON_Parse(cmd->payload);
    if (NULL == obj_root){
        cmdret = -1;
        goto EXIT_JSONPARSE;
    }

    obj_cmdname = cJSON_GetObjectItem(obj_root, "command_name");
    if (NULL == obj_cmdname){
        cmdret = -2;
        goto EXIT_CMDOBJ;
    }

    if (0 == strcmp(cJSON_GetStringValue(obj_cmdname), "Control_Valves")){
        obj_paras = cJSON_GetObjectItem(obj_root, "Paras");
        if(NULL == obj_paras){
            cmdret = -3;
            goto EXIT_OBJPARAS;
        }

        obj_para = cJSON_GetObjectItem(obj_paras, "Motor");
        if(NULL == obj_para){
            cmdret = -4;
            goto EXIT_OBJPARA;
        }

        if(0 == strcmp(cJSON_GetStringValue(obj_para), "ON")){
            //g_app_cb.motor = 1;
            Motor_StatusSet_RGB_R(ON);
            printf("RGB Red On\n");
        }else{
            //g_app_cb.motor = 0;
            Motor_StatusSet_RGB_R(OFF);
            printf("RGB Red Off\n");
        }
    }else if(0 == strcmp(cJSON_GetStringValue(obj_cmdname), "Control_G")){
        obj_paras = cJSON_GetObjectItem(obj_root, "Paras");
        if(NULL == obj_paras){
            cmdret = -3;
            goto EXIT_OBJPARAS;
        }

        obj_para = cJSON_GetObjectItem(obj_paras, "RGB_G");
        if(NULL == obj_para){
            cmdret = -4;
            goto EXIT_OBJPARA;
        }

        if(0 == strcmp(cJSON_GetStringValue(obj_para), "ON")){
            g_app_cb.motor = 1;
            Motor_StatusSet_RGB_G(ON);
            printf("RGB Green On\n");
        }else{
            g_app_cb.motor = 0;
            Motor_StatusSet_RGB_G(OFF);
            printf("RGB Green Off\n");
        }
    }

    cmdret = 0;

EXIT_OBJPARA:
EXIT_OBJPARAS:
EXIT_CMDOBJ:
    cJSON_Delete(obj_root);
EXIT_JSONPARSE:
    free(cmd->payload); // 释放拷贝的payload内存
    cmdresp.paras = NULL;
    cmdresp.request_id = cmd->request_id;
    cmdresp.ret_code = cmdret;
    cmdresp.ret_name = "Control_Valves";

    //printf("-d-d[%s]\n",cmd->request_id);

    oc_mqtt_profile_cmdresp(USERNAME, &cmdresp);

    return;
}

/***************************************************************
* 函数名称: Task_NFC
* 说    明: 任务6 - NFC读取WiFi账号密码
***************************************************************/
void Task_NFC(void)
{
    osMutexAcquire(mutex_id,osWaitForever);//先获取互斥锁

    uint8_t ndefLen = 0;  // ndef包的长度
    uint8_t ndef_Header = 0; // ndef消息开始标志位-用不到

    nfc_Init();//NFC初始化
    //sleep(3);//系统沉睡3秒

    // 读整个数据的包头部分，读出整个数据的长度
    if(NT3HReadHeaderNfc(&ndefLen,&ndef_Header) == 0 ){
        printf("Get Header Err\n");
        //NFC读取失败，使用默认WiFi
        strcpy((char*)ssd,"Mate60pro");
        strcpy((char*)secret,"88888888");
        printf("Use default WiFi: %s\n",ssd);
        osMutexRelease(mutex_id);
        return ;
    }

    ndefLen += NDEF_HEADER_SIZE; // 加上头部字节

    printf("ndefLen=%d",ndefLen);

    uint8_t *ndefBuff = (uint8_t*)malloc(ndefLen+1);//申请内存
    if(ndefBuff == NULL){
        printf("Get Buff Err\n");
        strcpy((char*)ssd,"Mate60pro");
        strcpy((char*)secret,"88888888");
        osMutexRelease(mutex_id);
        return;
    }

    //获取NFC中的数据包
    get_NDEFDataPackage(ndefBuff, ndefLen+1);

    //printf("start print ndefBuff.\n");

    //将数据包中以"n"之后的第一个字符算起至","为止，是ssid
    for(size_t i = 0; i < ndefLen; i++){
        //printf("0x%x ",ndefBuff[i]);

        if(0x6e == ndefBuff[i]){
            Keep_Ssd_Flag = i+1;
            break;
        }
    }

    //将数据包中以","为分割线标记记号
    for(size_t i = Keep_Ssd_Flag; i < ndefLen; i++){
        if(0x2c == ndefBuff[i]){
            Keep_Sec_Flag = i+1;
            break;
        }
    }

    printf("\r\n");

    //提取账号，密码长度
    Keep_Ssd_long = Keep_Sec_Flag - Keep_Ssd_Flag-1;
    Keep_Sec_long = ndefLen - Keep_Sec_Flag;

    //将账号存入数组
    for(size_t i = 0; i < Keep_Ssd_long; i++){
        ssd[i] = ndefBuff[i+Keep_Ssd_Flag];
    }

    // 去掉SSID开头的'n'（NDEF语言代码"en"中的n被误识别）
    if(ssd[0] == 'n' && Keep_Ssd_long > 1){
        for(size_t i = 0; i < Keep_Ssd_long - 1; i++){
            ssd[i] = ssd[i+1];
        }
        ssd[Keep_Ssd_long - 1] = '\0';
        Keep_Ssd_long--;
    }

    //将密码存入数组
    for(size_t i = 0; i < Keep_Sec_long; i++){
        secret[i] = ndefBuff[i+Keep_Sec_Flag];
    }
    free(ndefBuff );

    printf("SSID:%s\r\n",ssd);
    printf("PWD :%s\r\n",secret);

    osMutexRelease(mutex_id);//释放互斥锁
}

/***************************************************************
* 函数名称: Task_Main
* 说    明: 任务1 - WiFi连接 + MQTT连接 + 消息队列处理
***************************************************************/
static int Task_Main(void)
{
    app_msg_t *app_msg;//结构体别名创建

    sleep(3);
    osMutexAcquire(mutex_id,osWaitForever);//先获取互斥锁

    //WIFI连接（使用NFC读取的ssid和secret）
    if(WifiConnect((char*)ssd,(char*)secret) == 0){
        sprintf((char*)displayBuff_4," WIFI CONNECT!  ");//保存到displayBuff_4
        printf("WiFi Connected: %s\n",ssd);
    }else{
        sprintf((char*)displayBuff_4," WIFI DISCONNECT");//保存到displayBuff_4
        printf("WiFi Failed\n");
    }


    SSD1306_CLS();//清屏
    SSD1306_ShowStr(0,2,(uint8_t *)displayBuff_4,16);//OLED显示
    sleep(1);
    SSD1306_CLS();//清屏

    device_info_init(CLIENT_ID,USERNAME,PASSWORD);//保存三元组至相应结构体中
    if(oc_mqtt_init() != 0){
        printf("Mqtt conn failed\n");
    }
    oc_set_cmd_rsp_cb(oc_cmd_rsp_cb);//设置命令响应回调函数，有命令下发时进入命令响应回调函数

    sleep(3);
    osMutexRelease(mutex_id);//释放互斥锁

    SSD1306_CLS();//清屏
    printf("Task_Main enter wait\n");

    while(1){
        app_msg = NULL;
        if(osMessageQueueGet(mid_MsgQueue, (void **)&app_msg, NULL, osWaitForever) == 0){
            printf("G temp %d; hum %d; ps %d\n",
                app_msg->msg.report.temp,
                app_msg->msg.report.hum,
                app_msg->msg.report.lum
            );

            switch (app_msg->msg_type)
            {
                case en_msg_cmd:
                    deal_cmd_msg(&app_msg->msg.cmd);
                    free(app_msg_down);
                    free(app_msg_request_id);
                break;
                case en_msg_report:
                    deal_report_msg(&app_msg->msg.report);
                break;
                default:
                break;
            }
        }
    }

    return 0;
}

/***************************************************************
* 函数名称: Task_Sensor
* 说    明: 任务2 - 传感器数据采集
***************************************************************/
static int Task_Sensor(void)
{
    app_msg_up = malloc(sizeof(app_msg_t));//申请一块内存

    while(1){
        if(NULL != app_msg_up){
            SHT20_ReadData(&temperature, &humidity);//读取温湿度传感器
            AP3216C_ReadData(&ir, &als, &ps);//读取三合一传感器

            app_msg_up->msg_type = en_msg_report;
            app_msg_up->msg.report.hum = humidity;
            app_msg_up->msg.report.lum = ps;
            app_msg_up->msg.report.temp = temperature;

            //填充GPS经纬度
            if(GPS_Store.flags == 1){
                snprintf(app_msg_up->msg.report.Longitude, 10, "%d.%d",
                    GPS_Store.longitude, GPS_Store.longitude_Min);
                snprintf(app_msg_up->msg.report.Latitude, 9, "%d.%d",
                    GPS_Store.latitude, GPS_Store.latitude_Min);
            }else{
                strcpy(app_msg_up->msg.report.Longitude, "0.0");
                strcpy(app_msg_up->msg.report.Latitude, "0.0");
            }

            printf("P temp %d; hum %d; ps %d\n",
                app_msg_up->msg.report.temp,
                app_msg_up->msg.report.hum,
                app_msg_up->msg.report.lum
            );

            //更新OLED显示缓冲区
            snprintf((char*)displayBuff_0, 20, "Temp: %dC", app_msg_up->msg.report.temp);
            snprintf((char*)displayBuff_1, 20, "Hum : %d%%", app_msg_up->msg.report.hum);
            snprintf((char*)displayBuff_2, 20, "Lum : %d", app_msg_up->msg.report.lum);
            snprintf((char*)displayBuff_3, 20, "Motor:%s", g_app_cb.motor ? "ON " : "OFF");

            //发送消息队列
            osMessageQueuePut(mid_MsgQueue, &app_msg_up, 0U, 0U);
            sleep(5);
        }
    }
    return 0;
}

/***************************************************************
* 函数名称: Task_Mpu
* 说    明: 任务3 - MPU6050 加速度传感器检测（移动报警）
***************************************************************/
static void Task_Mpu(void)
{
    sleep(10);
    Mpu6050_Init();//加速度传感器初始化
    hi_s16 mpu_keep = 0;
    uint8_t mpu_keep_flag = 0;
    //uint8_t mpu_flag = 0;
    //hi_s32 mpu_acc[2] = {0};
    hi_u8 mpu_ds[14] = {0};
    hi_u8 mpu_ln = 14;
    hi_s16 mpu_dates[7];

    exec1 = 1U;
    id1 = osTimerNew(Timer1_Callback,osTimerPeriodic,&exec1,NULL);
    if(id1 != NULL){
        timerDelay_1 = 300U;
    }

    while(1){
        //读取MPU6050加速度数据
        Mpu6050_Measure_By(mpu_ds, mpu_ln);
        Mpu6050_Measure_Sh(mpu_dates);

        //检测加速度变化（X轴或Y轴超过阈值则触发报警）
        if(abs(mpu_dates[0]) > 15000 || abs(mpu_dates[1]) > 15000){
            if(mpu_keep_flag == 0){
                mpu_keep_flag = 1;
                printf("MPU Motion Detected! Buzzer ON\n");
                //蜂鸣器报警
                IoTGpioSetOutputVal(HI_IO_NAME_GPIO_0, IOT_GPIO_VALUE0);
                osTimerStart(id1, timerDelay_1);
            }
        }else{
            if(mpu_keep_flag != 0){
                mpu_keep_flag = 0;
                IoTGpioSetOutputVal(HI_IO_NAME_GPIO_0, IOT_GPIO_VALUE1);
                printf("MPU Stable\n");
            }
        }

        sleep(1);
    }
    return;
}

/***************************************************************
* 函数名称: Task_GPS
* 说    明: 任务4 - GPS数据接收
***************************************************************/
static void Task_GPS(void)
{
    //printf("UART Test Start\n");
    while(1){
        //串口接收信息处理函数
        GPS_Parameters();
        usleep(500000);//500ms
    }
}

/***************************************************************
* 函数名称: Task_Disp
* 说    明: 任务5 - OLED显示更新
***************************************************************/
static void Task_Disp(void)
{
    IoTGpioSetOutputVal(HI_IO_NAME_GPIO_0,IOT_GPIO_VALUE0);//设置GPIO为低电平，蜂鸣器ON
    osDelay(10);//鸣叫时间
    IoTGpioSetOutputVal(HI_IO_NAME_GPIO_0,IOT_GPIO_VALUE1);//设置GPIO为高电平，蜂鸣器OFF
    while(1){
        SSD1306_CLS();
        SSD1306_ShowStr(0, 0, displayBuff_0, 16);
        SSD1306_ShowStr(0, 2, displayBuff_1, 16);
        SSD1306_ShowStr(0, 4, displayBuff_2, 16);
        SSD1306_ShowStr(0, 6, displayBuff_3, 16);
        sleep(2);
    }
}

/***************************************************************
* 函数名称: Experiment_Stage3
* 说    明: 任务创建与启动（主入口）
***************************************************************/
static void Experiment_Stage3(void)
{
    mid_MsgQueue = osMessageQueueNew(MSGQUEUE_OBJECTS,sizeof(app_msg_t),NULL);//设置消息队列管理功能
    if(mid_MsgQueue == NULL){
        printf("MQ Err\n");
    }

    Peripheral_Init();//外设功能初始化
    SSD1306_CLS(); // 清屏
    nfc_Init();

    //创建六个任务
    osThreadAttr_t attr;
    attr.name = "Task_Main";//任务1
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;

    attr.stack_size = 10240;
    attr.priority = 23;
    if(osThreadNew((osThreadFunc_t)Task_Main,NULL,&attr) == NULL){
        printf("Task_Main Err\n");
    }

    attr.stack_size = 4096;//任务2
    attr.priority = 24;
    attr.name = "Task_Sensor";
    if(osThreadNew((osThreadFunc_t)Task_Sensor,NULL,&attr) == NULL){
        printf("Task_Sensor Err\n");
    }

    attr.stack_size = 2048;//任务3
    attr.priority = 25;
    attr.name = "Task_Mpu";
    if(osThreadNew((osThreadFunc_t)Task_Mpu,NULL,&attr) == NULL){
        printf("Task_Mpu Err\n");
    }

    attr.stack_size = 2048;//任务4
    attr.priority = 25;
    attr.name = "Task_GPS";
    if(osThreadNew((osThreadFunc_t)Task_GPS,NULL,&attr) == NULL){
        printf("Task_GPS Err\n");
    }

    attr.stack_size = 2048;//任务5
    attr.priority = 25;
    attr.name = "Task_Disp";
    if(osThreadNew((osThreadFunc_t)Task_Disp,NULL,&attr) == NULL){
        printf("Task_Disp Err\n");
    }

    attr.stack_size = 2048;//任务6
    attr.priority = 25;
    attr.name = "Task_NFC";
    if(osThreadNew((osThreadFunc_t)Task_NFC,NULL,&attr) == NULL){
        printf("Task_NFC Err\n");
    }

    mutex_id = osMutexNew(NULL);//创建互斥锁
    if(mutex_id == NULL){
        printf("Mutex Err\n");
    }

}

APP_FEATURE_INIT(Experiment_Stage3);
