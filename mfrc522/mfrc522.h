
#ifndef MFRC522_H
#define MFRC522_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */
#define MFRC522_EVENT                         0x0008

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the BLE Application
 */
extern void MFRC522_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 MFRC522_ProcessEvent( uint8 task_id, uint16 events );
extern void MFRC522_CheckCard(unsigned char* serial);

extern void uart_printf(const char *fmt, ...);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* MFRC522_H */
