#include "serial.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "usart.h"

uint8_t g_nRxBuff[MAX_LEN];
uint8_t g_strCommand[5];
uint8_t g_nRxData[16];

uint8_t g_kp[4];
uint8_t g_ki[4];
uint8_t g_kd[4];
uint8_t g_Setpoint[4];

uint8_t g_Vmax[4];
uint8_t g_Amax[4];
uint8_t g_Position[4];


bool g_bDataAvailable = false;
int Rxindex = 0;

uint8_t RXdata;

uint8_t STX[] = {0x02U};
uint8_t ETX[] = {0x03U};



//cut the string
uint8_t *subString(uint8_t *pBuff, int nPos, int nIndex)
{
    uint8_t *t = &pBuff[nPos];
    pBuff[nPos -1] = '\0';
    for(int i = nIndex; i <(strlen((char*)t) + 1); i++)
    {
        t[i] = '\0';
    }
    return t;
}

//Compare 2 string
bool StrCompare(uint8_t *pBuff, uint8_t *pSample, uint8_t nSize)
{
    for (int i = 0; i < nSize; i++)
    {
        if(pBuff[i] != pSample[i])
        {
            return false;
        }
    }
    return true;
}

//receive data
void SerialInit(void)
{
    HAL_UART_Receive_IT(&huart2, (uint8_t *)g_nRxBuff, MAX_LEN);
}

//receive data
void SerialAcceptReceive(void)
{	HAL_UART_Receive_IT(&huart2, (uint8_t*)g_nRxBuff, MAX_LEN);
}

//send data to GUI
void SerialWriteComm(uint8_t *pStrCmd, uint8_t *pData)
{
    uint8_t *pBuff;
    pBuff = (uint8_t *)malloc(22);
    uint8_t nIndex = 0;

    memcpy(pBuff + nIndex, STX, 1);
    nIndex += 1;
    memcpy(pBuff + nIndex, pStrCmd, 4);
    nIndex += 4;
    memcpy(pBuff + nIndex, pData, 16);
    nIndex += 16;
    memcpy(pBuff + nIndex, ETX, 1);

    HAL_UART_Transmit(&huart2, pBuff, 22 , 1000);

    free(pBuff);

}

//parse data to Command, Option, Data
void SerialParse(uint8_t *pBuff)
{
    if((pBuff[0] == STX[0]) && (pBuff[22] == ETX[0]))
  {
    memcpy(g_strCommand, subString(g_nRxBuff, 1,5), 5);

    memcpy(g_nRxData, subString(g_nRxBuff, 6,16), 16);

    if(StrCompare(g_strCommand, (uint8_t*)"SPID", 4))
    {
    	memcpy(g_kp, subString(g_nRxData, 0, 4), 4);
    	memcpy(g_ki, subString(g_nRxData, 4, 4), 4);
    	memcpy(g_kd, subString(g_nRxData, 8, 4), 4);
    	memcpy(g_Setpoint, subString(g_nRxData, 12, 4), 4);
    }
    else if(StrCompare(g_strCommand, (uint8_t*)"SINF", 4))
    {
    	memcpy(g_Amax, subString(g_nRxData, 0, 4), 4);
    	memcpy(g_Vmax, subString(g_nRxData, 4, 4), 4);
    	memcpy(g_Position, subString(g_nRxData, 8, 4), 4);
    }
  }
}


//interupt uart RX
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart ->Instance == huart2.Instance)
    {
        g_bDataAvailable = true;
        SerialParse(g_nRxBuff);

    }
    HAL_UART_Receive_IT(&huart2,(uint8_t*)g_nRxBuff, MAX_LEN);
}

