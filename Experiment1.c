#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "hi_io.h"
#include "iot_pwm.h"
#include "hi_pwm.h"
#include "iot_uart.h"
#include "hi_uart.h"
#include "iot_errno.h"

#define UART_BUFF_SIZE 1000//串口数据大小
#define MSGQUEUE_OBJECTS 16//消息队列对象的数量


typedef struct{
    osTimerId_t id;//创建定时器ID
    uint32_t timerDelay_1;//创建延时时间
    osTimerFunc_t func;
    osTimerType_t type;
}TIMER_OBJ_t;//定义定时器对象

typedef struct{
    char *Buf;
}MSGQUEUE_OBJ_t_rx;//定义消息队列对象

osMessageQueueId_t mid_MsgQueue;//消息队列ID
osMutexId_t mutex_id;//互斥锁ID

static void F1_Pressed(char *arg)
{
    (void)arg;
    IoTGpioSetOutputVal(HI_IO_NAME_GPIO_7,1);
    IoTGpioSetOutputVal(HI_IO_NAME_GPIO_8,1);
    printf("KEY1!\n");
}
static void F2_Pressed(char *arg)
{
    (void)arg;
    IoTGpioSetOutputVal(HI_IO_NAME_GPIO_7,0);
    IoTGpioSetOutputVal(HI_IO_NAME_GPIO_8,0);
    printf("KEY2!\n");
}

static void Stage1_Task1(void)
{
    unsigned int i = 0;
    int ret;

    uint8_t uart_buff_recv[UART_BUFF_SIZE] = {0};
    const char *test_data = "It is UART TX Data!\r\n";

    osMutexAcquire(mutex_id,osWaitForever);//获取互斥锁
    sleep(10);
    osMutexRelease(mutex_id);//释放互斥锁

    //通过串口2发送数据
    ret = IoTUartWrite(HI_UART_IDX_2,(unsigned char *)test_data,strlen(test_data));
    printf("--------------ret[%d]--------------\n",ret);
    if(ret <= 0){
        printf("Uart2 TX Data Failed,Error[%d]\n",ret);
    }else{
        usleep(10000);

        //通过串口2接收数据
        ret = IoTUartRead(HI_UART_IDX_2,uart_buff_recv,sizeof(uart_buff_recv));
        if(ret > 0){
            printf("Uart2 RX Data:[%s]\n",uart_buff_recv);
        }else{
            printf("Uart2 RX Data Failed,Error[%d]\n",ret);
        }
    }

    while(1){
        printf("%s is Running!\n",__FUNCTION__);

        for(i = 1; i < 100; i++){
            IoTPwmStart(HI_PWM_PORT_PWM3,i,40000);//输出不同占空比的PWM波
            usleep(10000);
        }
        sleep(1);

        for(i = 100; i > 0; i--){
            IoTPwmStart(HI_PWM_PORT_PWM3,i,40000);//输出不同占空比的PWM波
            usleep(10000);
        }
        sleep(1);
    }
}

static void Stage1_Task2(void)
{
    osStatus_t status = osError;
    MSGQUEUE_OBJ_t_rx Msgq;

    sleep(3);

    osMutexAcquire(mutex_id,osWaitForever);//获取互斥锁
    IoTGpioSetOutputVal(HI_IO_NAME_GPIO_1,0);//开启风扇
    osMutexRelease(mutex_id);//释放互斥锁

    Msgq.Buf = "It is MsgQ Data for testing message queue";
    while(1){
        printf("%s is Running!\n",__FUNCTION__);
        status = osMessageQueuePut(mid_MsgQueue,&Msgq,0U,0U);//消息队列发送
        if(status != osOK){
            printf("Message Queue Put failed: %d\n",status);
        }
        sleep(3);
    }
}

void Timer1_Callback()
{
    printf("-------------Buzzer is ON-------------\n");
    IoTGpioSetOutputVal(HI_IO_NAME_GPIO_0,0);//设置GPIO为低电平 蜂鸣器ON
    sleep(1);
    printf("-------------Buzzer is oFF------------\n");
    IoTGpioSetOutputVal(HI_IO_NAME_GPIO_0,1);//设置GPIO为高电平 蜂鸣器OFF

}

void Stage1_Task3(void)
{
    osStatus_t status;//返回参数
    TIMER_OBJ_t timer_obj;//定时器对象参数
    MSGQUEUE_OBJ_t_rx Msgq;//消息队列数据变量



    //设置定时器参数
    //Hi3861 1U=10ms,100U=1S
    timer_obj.timerDelay_1 = 1000;
    timer_obj.func = Timer1_Callback;
    timer_obj.type = osTimerPeriodic;

    //创建定时器
    timer_obj.id = osTimerNew(timer_obj.func,timer_obj.type,NULL,NULL);//创建定时器并获取ID
    if(timer_obj.id != NULL){
        status = osTimerStart(timer_obj.id,timer_obj.timerDelay_1);//定时器开启并获取返回参数
        if(status != osOK){
            printf("Timer could not be started!\n");
        }
    }

    while(1){
        printf("%s is Running!\n",__FUNCTION__);
        status = osMessageQueueGet(mid_MsgQueue,&Msgq,NULL,0U);//消息队列接收
        if(status == osOK){
            printf("Message Queue Get Data:[%s]\n",Msgq.Buf);
        }else{
            printf("Message Queue Get failed: %d\n",status);
        }

        sleep(3);
    }
}

void Peripheral_Init(void)
{
    //GPIO功能初始化
    IoTGpioInit(HI_IO_NAME_GPIO_0);
    IoTGpioInit(HI_IO_NAME_GPIO_1);
    IoTGpioInit(HI_IO_NAME_GPIO_2);
    IoTGpioInit(HI_IO_NAME_GPIO_6);
    IoTGpioInit(HI_IO_NAME_GPIO_7);
    IoTGpioInit(HI_IO_NAME_GPIO_8);
    IoTGpioInit(HI_IO_NAME_GPIO_11);
    IoTGpioInit(HI_IO_NAME_GPIO_12);
    IoTGpioInit(HI_IO_NAME_GPIO_13);
    IoTGpioInit(HI_IO_NAME_GPIO_14);

    //GPIO_11复用为UART2_TXD
    hi_io_set_func(HI_IO_NAME_GPIO_11,HI_IO_FUNC_GPIO_11_UART2_TXD);
    //GPIO_12复用为UART2_RXD
    hi_io_set_func(HI_IO_NAME_GPIO_12,HI_IO_FUNC_GPIO_12_UART2_RXD);

    //UART初始化
    IotUartAttribute uart_attr;
    uart_attr.baudRate = 9600;//波特率: 9600
    uart_attr.dataBits = IOT_UART_DATA_BIT_8;//数据位: 8bits
    uart_attr.stopBits = IOT_UART_STOP_BIT_1;//停止位：1bit
    uart_attr.parity = IOT_UART_PARITY_NONE;//校验位：None
    uart_attr.rxBlock = IOT_UART_BLOCK_STATE_BLOCK;
    uart_attr.txBlock = IOT_UART_BLOCK_STATE_BLOCK;
    IoTUartInit(HI_UART_IDX_2,&uart_attr);

    //Fan：低电平触发
    hi_io_set_func(HI_IO_NAME_GPIO_1,HI_IO_FUNC_GPIO_1_GPIO);//风扇：低电平触发
    IoTGpioSetDir(HI_IO_NAME_GPIO_1,IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(HI_IO_NAME_GPIO_1,1);

    //R：高电平触发
    hi_io_set_func(HI_IO_NAME_GPIO_2,HI_IO_FUNC_GPIO_2_GPIO);//设置GPIO_2的复用功能为普通GPIO
    IoTGpioSetDir(HI_IO_NAME_GPIO_2,IOT_GPIO_DIR_OUT);    //设置GPIO_2为输出模式
    IoTGpioSetOutputVal(HI_IO_NAME_GPIO_2,0);//设置GPIO_2为输出低电平

    //G：高电平触发
    hi_io_set_func(HI_IO_NAME_GPIO_7,HI_IO_FUNC_GPIO_7_GPIO);
    IoTGpioSetDir(HI_IO_NAME_GPIO_7,IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(HI_IO_NAME_GPIO_7,0);

    //B：高电平触发
    hi_io_set_func(HI_IO_NAME_GPIO_8,HI_IO_FUNC_GPIO_8_GPIO);
    IoTGpioSetDir(HI_IO_NAME_GPIO_8,IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(HI_IO_NAME_GPIO_8,0);

    //蜂鸣器：低电平触发
    hi_io_set_func(HI_IO_NAME_GPIO_0,HI_IO_FUNC_GPIO_0_GPIO);//GPIO功能复用为普通GPIO功能
    IoTGpioSetDir(HI_IO_NAME_GPIO_0,IOT_GPIO_DIR_OUT);//设置GPIO功能为输出
    IoTGpioSetOutputVal(HI_IO_NAME_GPIO_0,1);//设置GPIO为低电平

    hi_io_set_func(HI_IO_NAME_GPIO_6,HI_IO_FUNC_GPIO_6_PWM3_OUT);//设置GPIO_6引脚复用功能为PWM
    IoTGpioSetDir(HI_IO_NAME_GPIO_6,IOT_GPIO_DIR_OUT);//设置GPIO_6引脚为输出模式
    IoTPwmInit(HI_PWM_PORT_PWM3);//初始化PWM端口

    //初始化F1按键，设置为下降沿触发中断
    hi_io_set_func(HI_IO_NAME_GPIO_14,HI_IO_FUNC_GPIO_14_GPIO);//设置GPIO功能
    IoTGpioSetDir(HI_IO_NAME_GPIO_14,IOT_GPIO_DIR_IN);//设置GPIO输入
    hi_io_set_pull(HI_IO_NAME_GPIO_14,HI_IO_PULL_UP);
    IoTGpioRegisterIsrFunc(HI_IO_NAME_GPIO_14,IOT_INT_TYPE_EDGE,IOT_GPIO_EDGE_FALL_LEVEL_LOW,F1_Pressed,NULL);

    //初始化F2按键，设置为下降沿触发中断
    hi_io_set_func(HI_IO_NAME_GPIO_13,HI_IO_FUNC_GPIO_13_GPIO);
    IoTGpioSetDir(HI_IO_NAME_GPIO_13,IOT_GPIO_DIR_IN);
    hi_io_set_pull(HI_IO_NAME_GPIO_13,HI_IO_PULL_UP);
    IoTGpioRegisterIsrFunc(HI_IO_NAME_GPIO_13,IOT_INT_TYPE_EDGE,IOT_GPIO_EDGE_FALL_LEVEL_LOW,F2_Pressed,NULL);
}

static void Experiment_Stage1(void)
{
    //外设功能初始化
    Peripheral_Init();

    //创建消息队列
    mid_MsgQueue = osMessageQueueNew(MSGQUEUE_OBJECTS,100,NULL);
    if(mid_MsgQueue == NULL){
        printf("Falied to create Message Queue!\n");
    }

    //创建互斥锁
    mutex_id = osMutexNew(NULL);
    if(mutex_id == NULL){
        printf("Failed to create mutex!\n");
    }


    //设置线程参数
    osThreadAttr_t attr;
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 4;

    attr.name = "Stage1_Task1";
    attr.priority = 25;
    if(osThreadNew((osThreadFunc_t)Stage1_Task1,NULL,&attr) == NULL){
        printf("Falied to create Stage1_Task1!\n");
    }

    attr.name = "Stage1_Task2";
    attr.priority = 25;
    if(osThreadNew((osThreadFunc_t)Stage1_Task2,NULL,&attr) == NULL){
        printf("Falied to create Stage1_Task2!\n");
    }

    attr.name = "Stage1_Task3";
    attr.priority = 25;
    if(osThreadNew((osThreadFunc_t)Stage1_Task3,NULL,&attr) == NULL){
        printf("Falied to create Stage1_Task3!\n");
    }
}

APP_FEATURE_INIT(Experiment_Stage1);