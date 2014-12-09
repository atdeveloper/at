/*
 * File	: uart.c
 * This file is part of Espressif's AT+ command set program.
 * Copyright (C) 2013 - 2016, Espressif Systems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of version 3 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "ets_sys.h"
#include "osapi.h"
#include "driver/uart.h"
#include "osapi.h"
#include "driver/uart_register.h"
//#include "ssc.h"
#include "at.h"
#include "mem.h"

// UartDev is defined and initialized in rom code.
extern UartDevice    UartDev;
//extern os_event_t    at_recvTaskQueue[at_recvTaskQueueLen];

LOCAL void uart0_rx_intr_handler(void *para);

/******************************************************************************
 * FunctionName : uart_config
 * Description  : Internal used function
 *                UART0 used for data TX/RX, RX buffer size is 0x100, interrupt enabled
 *                UART1 just used for debug output
 * Parameters   : uart_no, use UART0 or UART1 defined ahead
 * Returns      : NONE
*******************************************************************************/
LOCAL void ICACHE_FLASH_ATTR
uart_config(uint8 uart_no)
{
  if (uart_no == UART1)
  {
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_U1TXD_BK);
  }
  else
  {
    /* rcv_buff size if 0x100 */
    ETS_UART_INTR_ATTACH(uart0_rx_intr_handler,  &(UartDev.rcv_buff));
    PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0TXD_U);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_U0RTS);
  }

  uart_div_modify(uart_no, UART_CLK_FREQ / (UartDev.baut_rate));

  WRITE_PERI_REG(UART_CONF0(uart_no), UartDev.exist_parity
                 | UartDev.parity
                 | (UartDev.stop_bits << UART_STOP_BIT_NUM_S)
                 | (UartDev.data_bits << UART_BIT_NUM_S));

  //clear rx and tx fifo,not ready
  SET_PERI_REG_MASK(UART_CONF0(uart_no), UART_RXFIFO_RST | UART_TXFIFO_RST);
  CLEAR_PERI_REG_MASK(UART_CONF0(uart_no), UART_RXFIFO_RST | UART_TXFIFO_RST);

  //set rx fifo trigger
//  WRITE_PERI_REG(UART_CONF1(uart_no),
//                 ((UartDev.rcv_buff.TrigLvl & UART_RXFIFO_FULL_THRHD) << UART_RXFIFO_FULL_THRHD_S) |
//                 ((96 & UART_TXFIFO_EMPTY_THRHD) << UART_TXFIFO_EMPTY_THRHD_S) |
//                 UART_RX_FLOW_EN);
  if (uart_no == UART0)
  {
    //set rx fifo trigger
    WRITE_PERI_REG(UART_CONF1(uart_no),
                   ((0x10 & UART_RXFIFO_FULL_THRHD) << UART_RXFIFO_FULL_THRHD_S) |
                   ((0x10 & UART_RX_FLOW_THRHD) << UART_RX_FLOW_THRHD_S) |
                   UART_RX_FLOW_EN |
                   (0x02 & UART_RX_TOUT_THRHD) << UART_RX_TOUT_THRHD_S |
                   UART_RX_TOUT_EN|
                   
                   ((0x10 & UART_TXFIFO_EMPTY_THRHD)<<UART_TXFIFO_EMPTY_THRHD_S));//wjl 
    SET_PERI_REG_MASK(UART_INT_ENA(uart_no), UART_RXFIFO_TOUT_INT_ENA |
                      UART_FRM_ERR_INT_ENA);
  }
  else
  {
    WRITE_PERI_REG(UART_CONF1(uart_no),
                   ((UartDev.rcv_buff.TrigLvl & UART_RXFIFO_FULL_THRHD) << UART_RXFIFO_FULL_THRHD_S));
  }

  //clear all interrupt
  WRITE_PERI_REG(UART_INT_CLR(uart_no), 0xffff);
  //enable rx_interrupt
  SET_PERI_REG_MASK(UART_INT_ENA(uart_no), UART_RXFIFO_FULL_INT_ENA);
}

/******************************************************************************
 * FunctionName : uart1_tx_one_char
 * Description  : Internal used function
 *                Use uart1 interface to transfer one char
 * Parameters   : uint8 TxChar - character to tx
 * Returns      : OK
*******************************************************************************/
LOCAL STATUS
uart_tx_one_char(uint8 uart, uint8 TxChar)
{
    while (true)
    {
      uint32 fifo_cnt = READ_PERI_REG(UART_STATUS(uart)) & (UART_TXFIFO_CNT<<UART_TXFIFO_CNT_S);
      if ((fifo_cnt >> UART_TXFIFO_CNT_S & UART_TXFIFO_CNT) < 126) {
        break;
      }
    }

    WRITE_PERI_REG(UART_FIFO(uart) , TxChar);
    return OK;
}

/******************************************************************************
 * FunctionName : uart1_write_char
 * Description  : Internal used function
 *                Do some special deal while tx char is '\r' or '\n'
 * Parameters   : char c - character to tx
 * Returns      : NONE
*******************************************************************************/
LOCAL void ICACHE_FLASH_ATTR
uart1_write_char(char c)
{
  if (c == '\n')
  {
    uart_tx_one_char(UART1, '\r');
    uart_tx_one_char(UART1, '\n');
  }
  else if (c == '\r')
  {
  }
  else
  {
    uart_tx_one_char(UART1, c);
  }
}
/******************************************************************************
 * FunctionName : uart0_tx_buffer
 * Description  : use uart0 to transfer buffer
 * Parameters   : uint8 *buf - point to send buffer
 *                uint16 len - buffer len
 * Returns      :
*******************************************************************************/
void ICACHE_FLASH_ATTR
uart0_tx_buffer(uint8 *buf, uint16 len)
{
  uint16 i;

  for (i = 0; i < len; i++)
  {
    uart_tx_one_char(UART0, buf[i]);
  }
}

/******************************************************************************
 * FunctionName : uart0_sendStr
 * Description  : use uart0 to transfer buffer
 * Parameters   : uint8 *buf - point to send buffer
 *                uint16 len - buffer len
 * Returns      :
*******************************************************************************/
void ICACHE_FLASH_ATTR
uart0_sendStr(const char *str)
{
	while(*str)
	{
		uart_tx_one_char(UART0, *str++);
	}
}

/******************************************************************************
 * FunctionName : uart0_rx_intr_handler
 * Description  : Internal used function
 *                UART0 interrupt handler, add self handle code inside
 * Parameters   : void *para - point to ETS_UART_INTR_ATTACH's arg
 * Returns      : NONE
*******************************************************************************/
//extern void at_recvTask(void);

LOCAL void
uart0_rx_intr_handler(void *para)
{
  /* uart0 and uart1 intr combine togther, when interrupt occur, see reg 0x3ff20020, bit2, bit0 represents
    * uart1 and uart0 respectively
    */
//  RcvMsgBuff *pRxBuff = (RcvMsgBuff *)para;
  uint8 RcvChar;
  uint8 uart_no = UART0;//UartDev.buff_uart_no;

//  if (UART_RXFIFO_FULL_INT_ST != (READ_PERI_REG(UART_INT_ST(uart_no)) & UART_RXFIFO_FULL_INT_ST))
//  {
//    return;
//  }
//  if (UART_RXFIFO_FULL_INT_ST == (READ_PERI_REG(UART_INT_ST(uart_no)) & UART_RXFIFO_FULL_INT_ST))
//  {
////    at_recvTask();
//    RcvChar = READ_PERI_REG(UART_FIFO(uart_no)) & 0xFF;
//    system_os_post(at_recvTaskPrio, NULL, RcvChar);
//    WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_RXFIFO_FULL_INT_CLR);
//  }
  if(UART_FRM_ERR_INT_ST == (READ_PERI_REG(UART_INT_ST(uart_no)) & UART_FRM_ERR_INT_ST))
  {
    os_printf("FRM_ERR\r\n");
    WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_FRM_ERR_INT_CLR);
  }

  if(UART_RXFIFO_FULL_INT_ST == (READ_PERI_REG(UART_INT_ST(uart_no)) & UART_RXFIFO_FULL_INT_ST))
  {
//    os_printf("fifo full\r\n");
    ETS_UART_INTR_DISABLE();/////////

    system_os_post(at_recvTaskPrio, 0, 0);

//    WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_RXFIFO_FULL_INT_CLR);
//    while (READ_PERI_REG(UART_STATUS(uart_no)) & (UART_RXFIFO_CNT << UART_RXFIFO_CNT_S))
//    {
////      at_recvTask();
//      RcvChar = READ_PERI_REG(UART_FIFO(uart_no)) & 0xFF;
//      system_os_post(at_recvTaskPrio, NULL, RcvChar);
//    }
  }
  else if(UART_RXFIFO_TOUT_INT_ST == (READ_PERI_REG(UART_INT_ST(uart_no)) & UART_RXFIFO_TOUT_INT_ST))
  {
    ETS_UART_INTR_DISABLE();/////////

//    os_printf("stat:%02X",*(uint8 *)UART_INT_ENA(uart_no));
    system_os_post(at_recvTaskPrio, 0, 0);

//    WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_RXFIFO_TOUT_INT_CLR);
////    os_printf("rx time over\r\n");
//    while (READ_PERI_REG(UART_STATUS(uart_no)) & (UART_RXFIFO_CNT << UART_RXFIFO_CNT_S))
//    {
////      os_printf("process recv\r\n");
////      at_recvTask();
//      RcvChar = READ_PERI_REG(UART_FIFO(uart_no)) & 0xFF;
//      system_os_post(at_recvTaskPrio, NULL, RcvChar);
//    }
  }
  else if(UART_TXFIFO_EMPTY_INT_ST == (READ_PERI_REG(UART_INT_ST(uart_no)) & UART_TXFIFO_EMPTY_INT_ST)){
  //WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_TXFIFO_EMPTY_INT_CLR);
    system_os_post(at_recvTaskPrio, 1, 0);
    //tx_start_uart_buffer(UART0);
    //os_printf("u");
	WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_TXFIFO_EMPTY_INT_CLR);
	CLEAR_PERI_REG_MASK(UART_INT_ENA(UART0), UART_TXFIFO_EMPTY_INT_ENA);


}

//  WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_RXFIFO_FULL_INT_CLR);

//  if (READ_PERI_REG(UART_STATUS(uart_no)) & (UART_RXFIFO_CNT << UART_RXFIFO_CNT_S))
//  {
//    RcvChar = READ_PERI_REG(UART_FIFO(uart_no)) & 0xFF;
//    at_recvTask();
//    *(pRxBuff->pWritePos) = RcvChar;

//    system_os_post(at_recvTaskPrio, NULL, RcvChar);

//    //insert here for get one command line from uart
//    if (RcvChar == '\r')
//    {
//      pRxBuff->BuffState = WRITE_OVER;
//    }
//
//    pRxBuff->pWritePos++;
//
//    if (pRxBuff->pWritePos == (pRxBuff->pRcvMsgBuff + RX_BUFF_SIZE))
//    {
//      // overflow ...we may need more error handle here.
//      pRxBuff->pWritePos = pRxBuff->pRcvMsgBuff ;
//    }
//  }
}

/******************************************************************************
 * FunctionName : uart_init
 * Description  : user interface for init uart
 * Parameters   : UartBautRate uart0_br - uart0 bautrate
 *                UartBautRate uart1_br - uart1 bautrate
 * Returns      : NONE
*******************************************************************************/
void ICACHE_FLASH_ATTR
uart_init(UartBautRate uart0_br, UartBautRate uart1_br)
{
  // rom use 74880 baut_rate, here reinitialize
  UartDev.baut_rate = uart0_br;
  uart_config(UART0);
  UartDev.baut_rate = uart1_br;
  uart_config(UART1);
  ETS_UART_INTR_ENABLE();

  // install uart1 putc callback
  os_install_putc1((void *)uart1_write_char);
}

void ICACHE_FLASH_ATTR
uart_reattach()
{
	uart_init(BIT_RATE_74880, BIT_RATE_74880);
//  ETS_UART_INTR_ATTACH(uart_rx_intr_handler_ssc,  &(UartDev.rcv_buff));
//  ETS_UART_INTR_ENABLE();
}






////////////////////////////////////////////////////////////////
//for uart tx
///////////////////
static struct UartTxBuff* pTxBuffer = NULL;

/******************************************************************************
 * FunctionName : uart_tx_one_char_no_wait
 * Description  : uart tx a single char without waiting for fifo 
 * Parameters   : uint8 uart - uart port
 *                uint8 TxChar - char to tx
 * Returns      : STATUS
*******************************************************************************/
LOCAL STATUS
uart_tx_one_char_no_wait(uint8 uart, uint8 TxChar)
{

    uint8 fifo_cnt = (( READ_PERI_REG(UART_STATUS(uart))>>UART_TXFIFO_CNT_S)& UART_TXFIFO_CNT);
    if (fifo_cnt < 126) {
        WRITE_PERI_REG(UART_FIFO(uart) , TxChar);
    }
    return OK;
}


/******************************************************************************
 * FunctionName : uart1_sendStr_no_wait
 * Description  : uart tx a string without waiting for every char, used for print debug info which can be lost
 * Parameters   : const char *str - string to be sent
 * Returns      : NONE
*******************************************************************************/
void ICACHE_FLASH_ATTR
uart1_sendStr_no_wait(const char *str)
{
    while(*str){
        uart_tx_one_char_no_wait(UART1, *str++);
    }
}


/******************************************************************************
 * FunctionName : tx_buff_enq
 * Description  : tx buffer enqueue: fill a first linked buffer 
 * Parameters   : char *pdata - data point  to be enqueue
 * Returns      : NONE
*******************************************************************************/
void ICACHE_FLASH_ATTR
    tx_buff_enq(char* pdata, uint16 data_len )
{
    if(pTxBuffer == NULL){
        DBG1("\n\rnull, create buffer struct\n\r");
        pTxBuffer = Uart_Tx_Buf_Init(UART_TX_BUF_SIZE);
        if(pTxBuffer!= NULL){
        DBG("pTxBuffer: 0x%08x\n\r",pTxBuffer);
        Uart_Tx_Buf_Fill(pTxBuffer ,  pdata,  data_len,0 );
        }else{
            os_printf("uart tx MALLOC no buf \n\r");
        }
    }else{
        DBG("fill pTxBuffer: 0x%08x\n\r",pTxBuffer);
        // if(pTxBuffer->Space == 0){
        if(pTxBuffer->Space==UART_TX_BUF_SIZE){
            DBG("HEAP:%d\n\r",system_get_free_heap_size());
            DBG("PIN:0x%08x; PTX:0x%08x ;SPACE:%d\n\r",pTxBuffer->pInPos,pTxBuffer->pTxPos,pTxBuffer->Space);
        }
        Uart_Tx_Buf_Fill(pTxBuffer ,  pdata,  data_len,0 );
    }

    SET_PERI_REG_MASK(UART_CONF1(UART0), (0x10 & UART_TXFIFO_EMPTY_THRHD)<<UART_TXFIFO_EMPTY_THRHD_S);
    SET_PERI_REG_MASK(UART_INT_ENA(UART0), UART_TXFIFO_EMPTY_INT_ENA);
}

/******************************************************************************
 * FunctionName : Uart_Tx_Buf_Init
 * Description  : initialization of a new uart tx buffer.
 * Parameters   : uint16 TxBufSize - size of the buffer
 * Returns      : pointer to the UartTxBuff
*******************************************************************************/
struct UartTxBuff* ICACHE_FLASH_ATTR
    Uart_Tx_Buf_Init(uint16 TxBufSize)
{
   
    uint32 heap_size = system_get_free_heap_size();
    if(heap_size <=OS_LEFT_MEM){
        DBG1("reach buf limit\n\r");
        return NULL;
    }else if(heap_size <=TxBufSize ){
        if(TxBufSize<=UART_TX_BUF_SIZE_MIN){
            DBG1("return null\n\r");
            return NULL;
        }else{
            return Uart_Tx_Buf_Init(TxBufSize/2);
        }
    }else{
        DBG("test heap size: %d\n\r",heap_size);
        struct UartTxBuff* pTxBuff = (struct UartTxBuff* )os_malloc(sizeof(struct UartTxBuff));
        pTxBuff->UartTxBuffSize = TxBufSize;
        pTxBuff->pUartTxBuff = (uint8*)os_malloc(pTxBuff->UartTxBuffSize);
        pTxBuff->pInPos = pTxBuff->pUartTxBuff;
        pTxBuff->pTxPos = pTxBuff->pUartTxBuff;
        pTxBuff->Space = pTxBuff->UartTxBuffSize;
        pTxBuff->TxBuffState = OK;
        pTxBuff->nextTxBuff = NULL;
        return pTxBuff;
    }
}


/******************************************************************************
 * FunctionName : Uart_Tx_Buf_Cpy
 * Description  : copy bytes from a data buffer to the uart tx buffer
 * Parameters   : struct UartTxBuff* pCur - tx buffer struct pointer
                          char* pdata - data to be copied
                          uint16 data_len - data length
 * Returns      : NONE
*******************************************************************************/
LOCAL void ICACHE_FLASH_ATTR
    Uart_Tx_Buf_Cpy(struct UartTxBuff* pCur, char* pdata , uint16 data_len)
{
    if(data_len == 0) return ;

    uint16 tail_len = pCur->pUartTxBuff + pCur->UartTxBuffSize - pCur->pInPos ;
    if(tail_len >= data_len){  //do not need to loop back  the queue
        os_memcpy(pCur->pInPos , pdata , data_len );
        pCur->pInPos += ( data_len );/////////////////wwwjl
        pCur->pInPos = (pCur->pUartTxBuff +  (pCur->pInPos - pCur->pUartTxBuff) % pCur->UartTxBuffSize );//////////////////////
        pCur->Space -=data_len;////////////////////
    }else{
        os_memcpy(pCur->pInPos, pdata, tail_len);
        pCur->pInPos += ( tail_len );
        pCur->pInPos = (pCur->pUartTxBuff +  (pCur->pInPos - pCur->pUartTxBuff) % pCur->UartTxBuffSize );
        pCur->Space -=tail_len;
        
        os_memcpy(pCur->pInPos, pdata+tail_len , data_len-tail_len);
        pCur->pInPos += ( data_len-tail_len );
        pCur->pInPos = (pCur->pUartTxBuff +  (pCur->pInPos - pCur->pUartTxBuff) % pCur->UartTxBuffSize );
        pCur->Space -=( data_len-tail_len);
    }
}



/******************************************************************************
 * FunctionName : Uart_Tx_Buf_Fill
 * Description  : fill the last seq of the tx buffer link , if full, create another one
 * Parameters   : struct UartTxBuff* pTxBuff - tx buffer struct pointer
                          char* pdata - data to be copied
                          uint16 data_len - data length
                          uint16 _level - the sequence of the buffer in the link
 * Returns      : NONE
*******************************************************************************/
void ICACHE_FLASH_ATTR
    Uart_Tx_Buf_Fill(struct UartTxBuff* pTxBuff , char* pdata, uint16 data_len ,uint16 _level )
{
    if(data_len == 0) return ;
    uint16 level = _level;
    uint16 ltmp;
    struct UartTxBuff* pCur = pTxBuff; //WHEN SHOULD IT SET BUSY
    //judge whether the buff is busy ??
    DBG("level %d : in buf fill\n\r",level);
    while(pCur->TxBuffState == BUSY){}
    DBG("level %d : after wait busy\n\r",level);
    pCur->TxBuffState = BUSY;
    DBG("level %d : set busy\n\r",level);
    
    while(pCur->nextTxBuff){
        level ++;
        pCur->TxBuffState = OK;
        pCur = pCur->nextTxBuff;
        while(pCur->TxBuffState == BUSY){}
        pCur->TxBuffState = BUSY;
    }
    DBG("level %d :start\n\r",level);
        //not busy then do the job
    if(pCur){
        if(data_len <= (pCur->Space) ){   // data can be put into this
            DBG("level %d : cpy buf\n\r",level);
            Uart_Tx_Buf_Cpy(pCur, pdata , data_len);
        }
        else{  //data overflow
            DBG("level %d : fill space\n\r",level);
            ltmp = pCur->Space;
            DBG("level %d : before cpy : pCur->Space: %d;pCur:0x%08x \n\r",level,pCur->Space,pCur);
            Uart_Tx_Buf_Cpy(pCur, pdata , pCur->Space);
            DBG("level %d : after cpy \n\r",level);
            pCur->nextTxBuff = Uart_Tx_Buf_Init( UART_TX_BUF_SIZE );
            DBG("level %d : fill next: 0x%08x\n\r",level,pCur->nextTxBuff);
            if(pCur->nextTxBuff != NULL){
                DBG("level %d : fill begin: 0x%08x\n\r",level,pCur->nextTxBuff);
                //Uart_Tx_Buf_Fill(pCur->nextTxBuff , pdata+ (pCur->Space),  data_len - pCur->Space ,level+1 );
                Uart_Tx_Buf_Fill(pCur->nextTxBuff , pdata+ ltmp,  data_len - ltmp ,level+1 );
                DBG("level %d : fill end: 0x%08x\n\r",level,pCur->nextTxBuff);
            }else{
                DBG2("level %d:tx no buf \n\r",level);
            }
        }
    }
    pCur->TxBuffState = OK;
    DBG("level %d : set idle\n\r",level);
}

/******************************************************************************
 * FunctionName : disp_tx_buffer_link
 * Description  : display every element of the buffer link , for debug only
 * Parameters   : struct UartTxBuff* pTxBuff - tx buffer struct pointer
                          uint16 _level - the sequence of the buffer in the link
 * Returns      : NONE
*******************************************************************************/
//for debug
void ICACHE_FLASH_ATTR
    disp_tx_buffer_link(struct UartTxBuff* pTxBuff , uint16 level)
{
    static uint32 buf_space = 0;
    //struct UartTxBuff* pCur = pTxBuff;
    DBG2("\n\r=================\n\r");
    DBG2("level %d :pTxBuff->UartTxBuffSize : %d\n\r",level,pTxBuff->UartTxBuffSize );
    DBG2("level %d :pTxBuff->pUartTxBuff : 0x%08x\n\r",level,pTxBuff->pUartTxBuff );
    DBG2("level %d :pTxBuff->pInPos :0x%08x\n\r",level,pTxBuff->pInPos);   
    DBG2("level %d :length from tx: %d\n\r",level,pTxBuff->pInPos-pTxBuff->pTxPos);   
    DBG2("level %d :length from buf: %d\n\r",level,pTxBuff->pInPos-pTxBuff->pUartTxBuff);  
    DBG2("level %d :pTxBuff->pTxPos : 0x%08x\n\r",level,pTxBuff->pTxPos);
    DBG2("level %d :pTxBuff->Space: %d\n\r",level,pTxBuff->Space);
    DBG2("level %d :pTxBuff->TxBuffState: %d\n\r",level,pTxBuff->TxBuffState);
    DBG2("level %d :pTxBuff->nextTxBuff: 0x%08x\n\r",level,pTxBuff->nextTxBuff);
    DBG2("\n\r**************\n\r");
    if(level == 0) buf_space=0;
    buf_space+=(pTxBuff->UartTxBuffSize - pTxBuff->Space);
    DBG2("level %d : buf_space : %d \n\r",level,buf_space);

    if(pTxBuff->nextTxBuff){
        disp_tx_buffer_link(pTxBuff->nextTxBuff,level+1);
    }
    uint32 heap_size = system_get_free_heap_size();
    DBG2("******************\n\r");
    DBG2("test heap size: %d\n\r",heap_size);
    DBG2("******************\n\r");
}


/******************************************************************************
 * FunctionName : tx_fifo_insert
 * Description  : insert byte to the uart tx fifo from the uart tx buffer
 * Parameters   : struct UartTxBuff* pTxBuff - tx buffer struct pointer
                          char* pdata - data to be copied
                          uint8 uart_no - uart port num
 * Returns      : NONE
*******************************************************************************/
void ICACHE_FLASH_ATTR
    tx_fifo_insert(struct UartTxBuff* pTxBuff, uint8 data_len,  uint8 uart_no)
{
    uint8 i;
    for(i = 0; i<data_len;i++){
        WRITE_PERI_REG(UART_FIFO(uart_no) , *(pTxBuff->pTxPos++));	//////////recover it after debug ; wjl
        if(pTxBuff->pTxPos == (pTxBuff->pUartTxBuff + pTxBuff->UartTxBuffSize)){
            pTxBuff->pTxPos = pTxBuff->pUartTxBuff;
        }
    }
    pTxBuff->pTxPos = (pTxBuff->pUartTxBuff +  (pTxBuff->pTxPos - pTxBuff->pUartTxBuff) % pTxBuff->UartTxBuffSize );
    pTxBuff->Space += data_len;
}



/******************************************************************************
 * FunctionName : tx_buf_free
 * Description  : deinit of the tx buffer
 * Parameters   : struct UartTxBuff* pTxBuff - tx buffer struct pointer
 * Returns      : NONE
*******************************************************************************/
void ICACHE_FLASH_ATTR
    tx_buf_free(struct UartTxBuff* pTxBuff)
{
    os_free(pTxBuff->pUartTxBuff);
    os_free(pTxBuff);
}
    

/******************************************************************************
 * FunctionName : tx_start_uart_buffer
 * Description  : get data from the tx buffer and fill the uart tx fifo, co-work with the uart fifo empty interrupt
 * Parameters   : uint8 uart_no - uart port num
 * Returns      : NONE
*******************************************************************************/
void ICACHE_FLASH_ATTR
    tx_start_uart_buffer(uint8 uart_no)
{
    uint8 tx_fifo_len = (READ_PERI_REG(UART_STATUS(uart_no))>>UART_TXFIFO_CNT_S)&UART_TXFIFO_CNT;
    DBG("tx_fifo_len: %d; 0x%02x \n\r",tx_fifo_len,tx_fifo_len);
    uint8 fifo_remain = UART_FIFO_LEN - tx_fifo_len ;
    DBG("fifo_remain: %d \n\r",fifo_remain);
    uint8 len_tmp;
    uint16 tail_ptx_len,head_ptx_len,data_len;
    //struct UartTxBuff* pTxBuff = *get_buff_prt();

    if(pTxBuffer){
        while(pTxBuffer->TxBuffState==BUSY){os_printf("b");}
        pTxBuffer->TxBuffState = BUSY;
        
        data_len = (pTxBuffer->UartTxBuffSize - pTxBuffer->Space);
        if(data_len > fifo_remain){
            len_tmp = fifo_remain;
            DBG("in data_len > fifo_remain:\n\rtest data_len: %d\n\r------\n\r",len_tmp);
            tx_fifo_insert( pTxBuffer,len_tmp,uart_no);
            pTxBuffer->TxBuffState = OK;
            SET_PERI_REG_MASK(UART_INT_ENA(UART0), UART_TXFIFO_EMPTY_INT_ENA);
        }else{
            len_tmp = data_len;
            DBG("in data_len <= fifo_remain\n\rtest data_len: %d\n\r------\n\r",len_tmp);
            tx_fifo_insert( pTxBuffer,len_tmp,uart_no);
            
            if(pTxBuffer->nextTxBuff == NULL){
                DBG(" TxBuff empty\n\r");
                pTxBuffer->TxBuffState = OK;
                CLEAR_PERI_REG_MASK(UART_INT_ENA(UART0), UART_TXFIFO_EMPTY_INT_ENA);
            }else{
                struct UartTxBuff* pBufTmp = pTxBuffer;
                pTxBuffer = pTxBuffer->nextTxBuff;
                tx_buf_free(pBufTmp);
                pBufTmp = NULL;
                tx_start_uart_buffer(uart_no);
            }
        }
    }else{
        DBG1("pTxBuff null \n\r");
    }
}









