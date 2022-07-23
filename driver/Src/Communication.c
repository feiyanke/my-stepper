#include "main.h"
#include "Communication.h"
#include "Motor.h"


//packet queue相关
#define PACKET_QUEUE_SIZE   8   //需要是2^n,便于优化
static Packet  queue[PACKET_QUEUE_SIZE];
static uint8_t queue_head = 0;
static uint8_t queue_end = 0;
static uint8_t queue_count = 0;

static uint8_t queue_overflow = 0;

static Packet* queue_enqueue_start()
{
    if(queue_count == PACKET_QUEUE_SIZE)
    {
        queue_overflow = 1;
        return NULL;
    }
    else
    {
        return &(queue[queue_end]);
    }
}
static void queue_enqueue_end()
{
    queue_end = (queue_end+1U)&(PACKET_QUEUE_SIZE-1U);
    queue_count ++;
}

static Packet* queue_dequeue_start()
{
    if(queue_count == 0)
    {
        return NULL;
    }
    else
    {
        return &(queue[queue_head]);
    }
}

static void queue_dequeue_end()
{
    queue_head = (queue_head+1U)&(PACKET_QUEUE_SIZE-1U);
    queue_count --;
}

//uart dma传输相关
//static uint8_t first = 1;
static void transmit(uint8_t* data, uint8_t length)
{
    DMA1->IFCR = DMA_FLAG_TC4;
    DMA1_Channel4->CCR &=  ~DMA_CCR_EN;
    DMA1_Channel4->CNDTR = length;
    DMA1_Channel4->CMAR = (uint32_t)data;
    DMA1_Channel4->CCR |=  DMA_CCR_EN;
}

#define IS_END() ((DMA1->ISR) & DMA_FLAG_TC4)

static uint8_t tx_buffer[MAX_PACKET_LENGTH];

static void transmit_packet(Packet* packet)
{
    uint8_t i,sum=0;
    tx_buffer[0] = SYNC;
    tx_buffer[1] = packet->cmd;
    tx_buffer[2] = packet->length;
    sum = tx_buffer[0] + tx_buffer[1] + tx_buffer[2];
    for(i=0;i<packet->length;i++)
    {
        tx_buffer[3+i] = packet->data[i];
        sum += tx_buffer[3+i];
    }
    tx_buffer[3+i] = sum;
    transmit(tx_buffer, packet->length + 4);
}

typedef enum
{
    Sync,
    Cmd,
    Length,
    Data,
    Checksum
} State;
static State state = Sync;
static Packet received_packet;
static uint8_t checksum;
static uint8_t data_index;

static void CmdMoveAngleExecute(CmdMoveAngle* cmd)
{
    MoveAngle(&motor0, cmd->angle, cmd->speed, cmd->type);
}

static void CmdMoveSpeedExecute(CmdMoveSpeed* cmd)
{
    MoveSpeed(&motor0, cmd->velocity, cmd->accelerate);
}

static void CmdMoveResetExecute(CmdReset* cmd)
{
    MoveReset(&motor0, cmd->speed, cmd->mode);
}

static void CmdMoveForwardSettingExecute(CmdForwardSetting* cmd)
{
    RobodySetForward(cmd->angle, cmd->type);
}

void TransmitMotorStop()
{
    Packet* packet;
    CmdMotorStop* data;
    ENTER_CRITICAL();
    packet = queue_enqueue_start();
    if(packet != NULL)
    {
        packet->cmd = CMD_UPDATE_MOTOR_STOP;
        packet->length = sizeof(CmdMotorStop);
        data = (CmdMotorStop* )(packet->data);
        data->forward = RobodyGetForward();
        data->angle = MotorGetPosition(&motor0);
        queue_enqueue_end();
    }
    EXIT_CRITICAL();
}

static void TransmitUpdateInformation()
{
    Packet* packet;
    CmdUpdateInformation* data;
    ENTER_CRITICAL();
    packet = queue_enqueue_start();
    if(packet != NULL)
    {
        packet->cmd = CMD_UPDATE_INFORMATION;
        packet->length = sizeof(CmdUpdateInformation);
        data = (CmdUpdateInformation* )(packet->data);
        data->nAppVer = APP_INFO->nAppVer;
        data->nAppChecksum = APP_INFO->nAppChecksum;
        queue_enqueue_end();
    }
    EXIT_CRITICAL();
}

static void ReceivePacket()
{
    uint8_t cmd = received_packet.cmd;
    uint8_t* data = received_packet.data;
    
    if(cmd == CMD_MOVE_ANGLE)
    {
        CmdMoveAngleExecute((CmdMoveAngle*)data);
    }
    else if(cmd == CMD_MOVE_SPEED)
    {
        CmdMoveSpeedExecute((CmdMoveSpeed*)data);
    }
    else if(cmd == CMD_MOVE_SPEED_DIRECT)
    {
        MoveSpeedDirect(&motor0, *(float*)data);
    }
    else if(cmd == CMD_MOVE_RESET)
    {
        CmdMoveResetExecute((CmdReset*)data);
    }
    else if(cmd == CMD_FORWARD_SETTING)
    {
        CmdMoveForwardSettingExecute((CmdForwardSetting*)data);
    }
    else if(cmd == CMD_READ_FORWARD)
    {
        TransmitFloat(CMD_UPDATE_FORWARD, RobodyGetForward());
    }
    else if(cmd == CMD_READ_POSITION)
    {
        TransmitFloat(CMD_UPDATE_POSITION, MotorGetPosition(&motor0));
    }
    else if(cmd == CMD_READ_MOTOR_STOP)
    {
        if(IsMotorStop(&motor0)) 
        {
            TransmitMotorStop();
        }
    }
    else if(cmd == CMD_READ_INFORMATION)
    {
        TransmitUpdateInformation();
    }
}
    

void ReceiveByte(uint8_t c)
{
    if(state == Sync)
    {
        if(c == SYNC)
        {
            state = Cmd;
            checksum = SYNC;
        }
    }
    else if(state == Cmd)
    {
        received_packet.cmd = c;
        checksum+=c;
        state = Length;
    }
    else if(state == Length)
    {
        if(c > MAX_DATA_LENGTH)
        {
            state = Sync;
        }
        received_packet.length = c;
        checksum+=c;
        if(c == 0)
        {
            state = Checksum;
        }
        else
        {
            state = Data;
            data_index = 0;
        }
    }
    else if(state == Data)
    {
        if(data_index >= MAX_DATA_LENGTH)
        {
            state = Sync;
        }
        else
        {
            received_packet.data[data_index] = c;
            data_index++;
            checksum+=c;
            if(data_index >= received_packet.length)
            {
                state = Checksum;
            }
        }
    }
    else if(state == Checksum)
    {
        if(checksum == c)
        {
            ReceivePacket();
        }
        state = Sync;
    }
    else
    {
        state = Sync;
    }
    
}

static void transmit_overflow()
{
    Packet* packet;
    ENTER_CRITICAL();
    packet = queue_enqueue_start();
    if(packet != NULL)
    {
        packet->cmd = CMD_QUEUE_OVERFLOW;
        packet->length = 0;
        queue_enqueue_end();
    }
    EXIT_CRITICAL();
}


static void CommunicationBspInit(void)
{
    /*
    PA9 - USART1-TX(DMA1_CHANNEL4)
    PA10 - USART1-RX
    */
    GPIO_InitTypeDef GpioInit;
    UART_HandleTypeDef UART_Handler;
    DMA_HandleTypeDef DmaHandler;
    
    __GPIOA_CLK_ENABLE();
    //初始化作为通信端口的USART1   TXD:PA9;RXD:PA10
    __USART1_CLK_ENABLE();
    //__HAL_AFIO_REMAP_USART1_ENABLE();
    GpioInit.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GpioInit.Mode = GPIO_MODE_AF_PP;
    GpioInit.Pull = GPIO_NOPULL;
    GpioInit.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GpioInit);

    //初始化UART参数
    UART_Handler.Instance = USART1;
    UART_Handler.Init.BaudRate = 115200;
	UART_Handler.Init.WordLength = UART_WORDLENGTH_8B;
	UART_Handler.Init.StopBits = UART_STOPBITS_1;
	UART_Handler.Init.Parity = UART_PARITY_NONE ;
	UART_Handler.Init.Mode = UART_MODE_TX_RX;
    UART_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    UART_Handler.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&UART_Handler);
    
    USART1->CR3 |= USART_CR3_DMAT;  //开启发送的DMA
    USART1->CR1 |= USART_CR1_RXNEIE; //开启接收的中断

    //初始化作为发送的DMA
    __DMA1_CLK_ENABLE();
    
    DmaHandler.Instance = DMA1_Channel4;
    DmaHandler.Init.Direction = DMA_MEMORY_TO_PERIPH;
    DmaHandler.Init.PeriphInc = DMA_PINC_DISABLE;
    DmaHandler.Init.MemInc = DMA_MINC_ENABLE;
    DmaHandler.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    DmaHandler.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    DmaHandler.Init.Mode = DMA_NORMAL;
    DmaHandler.Init.Priority = DMA_PRIORITY_MEDIUM;
    HAL_DMA_Init(&DmaHandler);
    
    DMA1_Channel4->CPAR = (uint32_t)(&(USART1->DR));
    
    __HAL_DMA_DISABLE_IT(&DmaHandler, DMA_IT_TC);
    __HAL_DMA_DISABLE_IT(&DmaHandler, DMA_IT_HT);  
    __HAL_DMA_DISABLE_IT(&DmaHandler, DMA_IT_TE);
    
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0); //
    HAL_NVIC_EnableIRQ(USART1_IRQn); 
}


void CommunicationInit()
{
    CommunicationBspInit();
    transmit(0,1);//DUMMY TRANSMIT FOR TCIF STATUS
}

void TransmitRun()
{
    Packet* packet;
    
    if(IS_END())
    {
        packet = queue_dequeue_start();
        if(packet != NULL)
        {
            transmit_packet(packet);
            queue_dequeue_end();
            
            if(queue_overflow)
            {
                transmit_overflow();
                queue_overflow = 0;
            }
        }
    }
}

void TransmitNoDataCmd(uint8_t cmd)
{
    Packet* packet;
    ENTER_CRITICAL();
    packet = queue_enqueue_start();
    if(packet != NULL)
    {
        packet->cmd = cmd;
        packet->length = 0;
        queue_enqueue_end();
    }
    EXIT_CRITICAL();
}

void TransmitPacket(uint8_t cmd, uint8_t* data, uint8_t length)
{
    uint8_t i;
    Packet* packet;
    ENTER_CRITICAL();
    packet = queue_enqueue_start();
    if(packet != NULL)
    {
        packet->cmd = cmd;
        packet->length = length;
        for(i=0;i<length;i++)
        {
            packet->data[i] = data[i];
        }
        queue_enqueue_end();
    }
    EXIT_CRITICAL();
}

void TransmitByte(uint8_t cmd, uint8_t data)
{
    Packet* packet;
    ENTER_CRITICAL();
    packet = queue_enqueue_start();
    if(packet != NULL)
    {
        packet->cmd = cmd;
        packet->length = 1;
        packet->data[0] = data;
        queue_enqueue_end();
    }
    EXIT_CRITICAL();
}

void TransmitFloat(uint8_t cmd, float data)
{
    Packet* packet;
    ENTER_CRITICAL();
    packet = queue_enqueue_start();
    if(packet != NULL)
    {
        packet->cmd = cmd;
        packet->length = sizeof(data);
        *((float*)(packet->data)) = data;
        queue_enqueue_end();
    }
    EXIT_CRITICAL();
}
