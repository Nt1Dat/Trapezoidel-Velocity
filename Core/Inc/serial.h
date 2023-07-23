#ifndef INC_SERIAL_H_
#define INC_SERIAL_H_

#include <stdint.h>
#include <stdbool.h>

#define MAX_LEN 23



typedef enum
{
    NONE = 1,
    SPID,
    VTUN,
    PTUN,
	STOP,
	MOVP,
	MOVV,
	SINF,
}PROCESS_t;


extern void SerialInit(void);
extern uint8_t *subString(uint8_t *pBuff, int Pos, int nIndex);
extern bool StrCompare(uint8_t *pBuff, uint8_t *pSample, uint8_t nSize);
extern void SerialWriteComm(uint8_t *pStrCmd, uint8_t *pData);
extern void SerialParse(uint8_t *pBuff);
extern void SerialAcceptReceive(void);


#endif /* INC_SERIAL_H_ */
