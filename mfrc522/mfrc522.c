
/*********************************************************************
 * INCLUDES
 */
#include <stdio.h>
#include <stdarg.h>

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "hal_uart.h"
#include "hal_types.h"

#include "mfrc522.h"
/*********************************************************************
 * MACROS
 */
#define HAL_IO_SET(port, pin, val)        HAL_IO_SET_PREP(port, pin, val)
#define HAL_IO_SET_PREP(port, pin, val)   st( P##port##_##pin## = val; )
    
#define HAL_CONFIG_IO_OUTPUT(port, pin, val)      HAL_CONFIG_IO_OUTPUT_PREP(port, pin, val)
#define HAL_CONFIG_IO_OUTPUT_PREP(port, pin, val) st( P##port##SEL &= ~BV(pin); \
                                                      P##port##_##pin## = val; \
                                                      P##port##DIR |= BV(pin); )
    
#define HAL_CONFIG_IO_PERIPHERAL(port, pin)      HAL_CONFIG_IO_PERIPHERAL_PREP(port, pin)
#define HAL_CONFIG_IO_PERIPHERAL_PREP(port, pin) st( P##port##SEL |= BV(pin); )

/* SPI lines */
#define HAL_SPI_CS_PORT 1
#define HAL_SPI_CS_PIN  2

#define HAL_SPI_CLK_PORT 1
#define HAL_SPI_CLK_PIN  5

#define HAL_SPI_MOSI_PORT 1
#define HAL_SPI_MOSI_PIN  6

#define HAL_SPI_MISO_PORT 1
#define HAL_SPI_MISO_PIN  7

/* Control lines */
#define HAL_SPI_MODE_PORT 1
#define HAL_SPI_MODE_PIN  7

/* SPI settings */
#define HAL_SPI_CLOCK_POL_LO       0x00
#define HAL_SPI_CLOCK_PHA_0        0x00
#define HAL_SPI_TRANSFER_MSB_FIRST 0x20

/* SPI interface control */
#define SPI_BEGIN()     HAL_IO_SET(HAL_SPI_CS_PORT,  HAL_SPI_CS_PIN,  0); /* chip select */
#define SPI_END()                                                         \
{                                                                         \
  asm("NOP");                                                             \
  asm("NOP");                                                             \
  asm("NOP");                                                             \
  asm("NOP");                                                             \
  HAL_IO_SET(HAL_SPI_CS_PORT,  HAL_SPI_CS_PIN,  1); /* chip select */     \
}
/* clear the received and transmit byte status, write tx data to buffer, wait till transmit done */
#define SPI_TX(x)                   { U1CSR &= ~(BV(1)); U1DBUF = x; while( !(U1CSR & BV(1)) ); }
#define SPI_RX(x)                   { x = U1DBUF; }
/* Control macros */
#define SPI_DO_WRITE()        HAL_IO_SET(HAL_SPI_MODE_PORT,  HAL_SPI_MODE_PIN,  1);
#define SPI_DO_CONTROL()      HAL_IO_SET(HAL_SPI_MODE_PORT,  HAL_SPI_MODE_PIN,  0);

#define MAX_LEN 16
//MF522 Command word
#define PCD_IDLE              0x00               // NO action; Cancel the current command
#define PCD_MEM               0x01               // stores 25 bytes into the internal buffer
#define PCD_GETRAND           0x02               // get random number
#define PCD_CALCCRC           0x03               // CRC Calculate
#define PCD_TRANSMIT          0x04               // Transmit data
#define PCD_RECEIVE           0x08               // Receive Data
#define PCD_TRANSCEIVE        0x0C               // Transmit and receive data,
#define PCD_AUTHENT           0x0E               // Authentication Key
#define PCD_RESETPHASE        0x0F               // Reset

// Mifare_One card command word
#define PICC_REQIDL          0x26               // find the antenna area does not enter hibernation
#define PICC_REQALL          0x52               // find all the cards antenna area
#define PICC_ANTICOLL        0x93               // anti-collision
#define PICC_SElECTTAG       0x93               // election card
#define PICC_AUTHENT1A       0x60               // authentication key A
#define PICC_AUTHENT1B       0x61               // authentication key B
#define PICC_READ            0x30               // Read Block
#define PICC_WRITE           0xA0               // write block
#define PICC_DECREMENT       0xC0               // debit
#define PICC_INCREMENT       0xC1               // recharge
#define PICC_RESTORE         0xC2               // transfer block data to the buffer
#define PICC_TRANSFER        0xB0               // save the data in the buffer
#define PICC_HALT            0x50               // Sleep


//And MF522 The error code is returned when communication
#define MI_OK                 0
#define MI_NOTAGERR           1
#define MI_ERR                2

//------------------MFRC522 Register---------------
//Page 0:Command and Status
#define     Reserved00            0x00
#define     CommandReg            0x01
#define     CommIEnReg            0x02
#define     DivlEnReg             0x03
#define     CommIrqReg            0x04
#define     DivIrqReg             0x05
#define     ErrorReg              0x06
#define     Status1Reg            0x07
#define     Status2Reg            0x08
#define     FIFODataReg           0x09
#define     FIFOLevelReg          0x0A
#define     WaterLevelReg         0x0B
#define     ControlReg            0x0C
#define     BitFramingReg         0x0D
#define     CollReg               0x0E
#define     Reserved01            0x0F
//Page 1:Command
#define     Reserved10            0x10
#define     ModeReg               0x11
#define     TxModeReg             0x12
#define     RxModeReg             0x13
#define     TxControlReg          0x14
#define     TxAutoReg             0x15
#define     TxSelReg              0x16
#define     RxSelReg              0x17
#define     RxThresholdReg        0x18
#define     DemodReg              0x19
#define     Reserved11            0x1A
#define     Reserved12            0x1B
#define     MifareReg             0x1C
#define     Reserved13            0x1D
#define     Reserved14            0x1E
#define     SerialSpeedReg        0x1F
//Page 2:CFG
#define     Reserved20            0x20
#define     CRCResultRegM         0x21
#define     CRCResultRegL         0x22
#define     Reserved21            0x23
#define     ModWidthReg           0x24
#define     Reserved22            0x25
#define     RFCfgReg              0x26
#define     GsNReg                0x27
#define     CWGsPReg              0x28
#define     ModGsPReg             0x29
#define     TModeReg              0x2A
#define     TPrescalerReg         0x2B
#define     TReloadRegH           0x2C
#define     TReloadRegL           0x2D
#define     TCounterValueRegH     0x2E
#define     TCounterValueRegL     0x2F
//Page 3:TestRegister
#define     Reserved30            0x30
#define     TestSel1Reg           0x31
#define     TestSel2Reg           0x32
#define     TestPinEnReg          0x33
#define     TestPinValueReg       0x34
#define     TestBusReg            0x35
#define     AutoTestReg           0x36
#define     VersionReg            0x37
#define     AnalogTestReg         0x38
#define     TestDAC1Reg           0x39
#define     TestDAC2Reg           0x3A
#define     TestADCReg            0x3B
#define     Reserved31            0x3C
#define     Reserved32            0x3D
#define     Reserved33            0x3E
#define     Reserved34            0x3F
//-----------------------------------------------

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 mfrc522_TaskID;   // Task ID for internal task/event processing
static uint8 selfTestStatus = FALSE;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void mfrc522_reset(void);
static void antenna_on(void);
static bool mfrc522_selftest(void);
static void mfrc_hw_init(void);
static void write_mfrc522(char addr, char val);
static char read_mfrc522(char addr);
static void waitUs(uint16 microSecs);
static void uart_init(void);
/*********************************************************************
 * PROFILE CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      MFRC522_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void MFRC522_Init( uint8 task_id )
{
    mfrc522_TaskID = task_id;
    // config uart
    uart_init();

    // Setup a delayed profile startup
    osal_set_event( mfrc522_TaskID, MFRC522_EVENT );

    // setup does initial pin config, performs a soft reset on the card, and sets
    // some sane defaults for the SunFounder Mifare RC522 card
    mfrc_hw_init();

    // now do what you want

    // if you just wanted to do a simple read or write to a register on the chip,
    // use one of the below commands...
    /*
    * char readd = read_mfrc522(TxControlReg);
    * write_mfrc522(TxControlReg, 0x03);
    */

}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 MFRC522_ProcessEvent( uint8 task_id, uint16 events )
{
    VOID task_id; // OSAL required parameter that isn't used in this function

    if ( events & MFRC522_EVENT )
    {
        // MFRC522_CheckCard();

        // osal_start_timerEx( mfrc522_TaskID, MFRC522_EVENT, 2000 );

        // return unprocessed events
        return (events ^ MFRC522_EVENT);
    }

    // Discard unknown events
    return 0;
}

uint8 MFRC522_GetTestStatus(void)
{
    return selfTestStatus;
}
/*********************************************************************
 * PRIVATE FUNCTIONS
 */

void uart_init(void)
{
#ifdef UART_DEBUG
    halUARTCfg_t halUARTCfg;
    halUARTCfg.configured = TRUE;
    halUARTCfg.baudRate = HAL_UART_BR_9600;
    halUARTCfg.flowControl = HAL_UART_FLOW_OFF;
    HalUARTOpen(HAL_UART_PORT_0, &halUARTCfg);
    HalUARTWrite(HAL_UART_PORT_0, "HELLO CC2540!\r\n", 15);
#endif
}

void uart_printf(const char *fmt, ...)
{
#ifdef UART_DEBUG
    char endChar = '\r';
    va_list ap;
    char string[256];
    char* pt = string;

    osal_memset(string, 0, sizeof(string));

    va_start(ap, fmt);
    vsprintf(string, fmt, ap);

    while (*pt) {
        if(*pt == '\n')
            HalUARTWrite(HAL_UART_PORT_0, (uint8*)&endChar, 1);
        HalUARTWrite(HAL_UART_PORT_0, (uint8*)pt++, 1);
    }

    va_end(ap);
#endif
}

static void mfrc_hw_init(void)
{
    volatile char result = 0;
    /* config CC2540 SPI settings*/
    HAL_CONFIG_IO_OUTPUT(HAL_SPI_CS_PORT, HAL_SPI_CS_PIN, 1);

    /* UART/SPI Peripheral configuration */
    uint8 baud_exponent;
    uint8 baud_mantissa;

    /* Set SPI on UART 1 alternative 2 */
    PERCFG |= 0x02;

    /* Configure clk, master out and master in lines */
    HAL_CONFIG_IO_PERIPHERAL(HAL_SPI_CLK_PORT,  HAL_SPI_CLK_PIN);
    HAL_CONFIG_IO_PERIPHERAL(HAL_SPI_MOSI_PORT, HAL_SPI_MOSI_PIN);
    HAL_CONFIG_IO_PERIPHERAL(HAL_SPI_MISO_PORT, HAL_SPI_MISO_PIN);


    /* Set SPI speed to 1 MHz (the values assume system clk of 32MHz)
    * Confirm on board that this results in 1MHz spi clk.
    */
    baud_exponent = 15;
    baud_mantissa =  0;

    /* Configure SPI */
    U1UCR  = 0x80;      /* Flush and goto IDLE state. 8-N-1. */
    U1CSR  = 0x00;      /* SPI mode, master. */
    U1GCR  = HAL_SPI_TRANSFER_MSB_FIRST | HAL_SPI_CLOCK_PHA_0 | HAL_SPI_CLOCK_POL_LO | baud_exponent;
    U1BAUD = baud_mantissa;

    /* config mfrc522 settings */
    selfTestStatus = mfrc522_selftest();
    
    mfrc522_reset();
    
    // Timer: TPrescaler*TreloadVal/6.78MHz = 24ms
    write_mfrc522(TModeReg, 0x8D);      // Tauto=1; f(Timer) = 6.78MHz/TPreScaler
    write_mfrc522(TPrescalerReg, 0xA9); // TModeReg[3..0] + TPrescalerReg
    write_mfrc522(TReloadRegH, 0x03);
    write_mfrc522(TReloadRegL, 0xE8);
    write_mfrc522(TxAutoReg, 0x40);     // force 100% ASK modulation
    write_mfrc522(ModeReg, 0x3D);       // CRC Initial value 0x6363

    // turn antenna on
    antenna_on();
}

/*
 * Function Name：Write_MFRC5200
 * Function Description: To a certain MFRC522 register to write a byte of data
 * Input Parameters：addr - register address; val - the value to be written
 * Return value: None
 */
void write_mfrc522(char addr, char val)
{   
    // set the select line so we can start transferring
    SPI_BEGIN();
    // even though we are calling transfer frame once, we are really sending
    // two 8-bit frames smooshed together-- sending two 8 bit frames back to back
    // results in a spike in the select line which will jack with transactions
    // - top 8 bits are the address. Per the spec, we shift the address left
    //   1 bit, clear the LSb, and clear the MSb to indicate a write
    // - bottom 8 bits are the data bits being sent for that address, we send
    //   them as is
    SPI_TX((addr << 1) & 0x7E);
    SPI_TX(val);
    
    // clear the select line-- we are done here
    SPI_END();

    // waitUs(1);
}

/*
 * Function Name：read_mfrc522
 * Description: From a certain MFRC522 read a byte of data register
 * Input Parameters: addr - register address
 * Returns: a byte of data read from the
 */
char read_mfrc522(char addr)
{
    uint8 result = 0x00;

    // set the select line so we can start transferring
    SPI_BEGIN();

    // even though we are calling transfer frame once, we are really sending
    // two 8-bit frames smooshed together-- sending two 8 bit frames back to back
    // results in a spike in the select line which will jack with transactions
    // - top 8 bits are the address. Per the spec, we shift the address left
    //   1 bit, clear the LSb, and set the MSb to indicate a read
    // - bottom 8 bits are all 0s on a read per 8.1.2.1 Table 6
    SPI_TX(((addr << 1) & 0x7E) | 0x80);
    SPI_TX(0x00);
    SPI_RX(result);
    // clear the select line-- we are done here
    SPI_END();

    //waitUs(15000); // 15 ms
    
    return result;
}

void waitUs(uint16 microSecs)
{
  while(microSecs--)
  {
    /* 32 NOPs == 1 usecs */
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop");
  }
}

/*
 * Function Name：SetBitMask
 * Description: Set RC522 register bit
 * Input parameters: reg - register address; mask - set value
 * Return value: None
 */
void SetBitMask(char reg, char mask)  
{
    char tmp;
    tmp = read_mfrc522(reg);
    write_mfrc522(reg, tmp | mask);  // set bit mask
}


/*
 * Function Name: ClearBitMask
 * Description: clear RC522 register bit
 * Input parameters: reg - register address; mask - clear bit value
 * Return value: None
*/
void ClearBitMask(char reg, char mask)  
{
    char tmp;
    tmp = read_mfrc522(reg);
    write_mfrc522(reg, tmp & (~mask));  // clear bit mask
} 


/*
 * Function Name：AntennaOn
 * Description: Open antennas, each time you start or shut down the natural barrier between the transmitter should be at least 1ms interval
 * Input: None
 * Return value: None
 */
void antenna_on(void)
{
  SetBitMask(TxControlReg, 0x03);
}


/*
  * Function Name: AntennaOff
  * Description: Close antennas, each time you start or shut down the natural barrier between the transmitter should be at least 1ms interval
  * Input: None
  * Return value: None
 */
void AntennaOff(void)
{
  ClearBitMask(TxControlReg, 0x03);
}


/*
 * Function Name: ResetMFRC522
 * Description: Reset RC522
 * Input: None
 * Return value: None
 */
void mfrc522_reset(void)
{
    uint8 result;

    if (selfTestStatus == false) {
        return;
    }
    write_mfrc522(CommandReg, PCD_RESETPHASE);
    // Wait for the PowerDown bit in CommandReg to be cleared (max 3x50ms)
    do {
        result = read_mfrc522(CommandReg);
    } while (result & BV (4));

    uart_printf("FINISH RESET\n");
}

/*
 * Function Name: MFRC522_ToCard
 * Description: RC522 and ISO14443 card communication
 * Input Parameters: command - MF522 command word,
 *           sendData--RC522 sent to the card by the data
 *           sendLen--Length of data sent    
 *           backData--Received the card returns data,
 *           backLen--Return data bit length
 * Return value: the successful return MI_OK
 */
char MFRC522_ToCard(char command, char *sendData, char sendLen, char *backData, uint32 *backLen)
{
  char status = MI_ERR;
  char irqEn = 0x00;
  char waitIRq = 0x00;
  char lastBits;
  char n;
  uint32 i;

  switch (command)
  {
    case PCD_AUTHENT:     // Certification cards close
      {
        irqEn = 0x12;
        waitIRq = 0x10;
        break;
      }
    case PCD_TRANSCEIVE:  // Transmit FIFO data
      {
        irqEn = 0x77;
        waitIRq = 0x30;
        break;
      }
    default:
      break;
  }

  write_mfrc522(CommIEnReg, irqEn|0x80);  // Interrupt request
  ClearBitMask(CommIrqReg, 0x80);         // Clear all interrupt request bit
  SetBitMask(FIFOLevelReg, 0x80);         // FlushBuffer=1, FIFO Initialization

  write_mfrc522(CommandReg, PCD_IDLE);    // NO action; Cancel the current command

  // Writing data to the FIFO
  for (i=0; i<sendLen; i++)
  {
    write_mfrc522(FIFODataReg, sendData[i]);
  }

  // Execute the command
  write_mfrc522(CommandReg, command);
  if (command == PCD_TRANSCEIVE)
  {
    SetBitMask(BitFramingReg, 0x80);      // StartSend=1,transmission of data starts  
  }

  // Waiting to receive data to complete
  // 此數值的修改原作者目標為等待25ms
  i = 50;
  do
  {
    // CommIrqReg[7..0]
    // Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
    n = read_mfrc522(CommIrqReg);
    i--;
  }
  while ((i!=0) && !(n&0x01) && !(n&waitIRq));

  ClearBitMask(BitFramingReg, 0x80);      // StartSend=0

  if (i != 0)
  {
    if(!(read_mfrc522(ErrorReg) & 0x1B))  // BufferOvfl Collerr CRCErr ProtecolErr
    {
      status = MI_OK;
      if (n & irqEn & 0x01)
      {
        status = MI_NOTAGERR;             // ??
      }

      if (command == PCD_TRANSCEIVE)
      {
        n = read_mfrc522(FIFOLevelReg);
        lastBits = read_mfrc522(ControlReg) & 0x07;
        if (lastBits)
        {
          *backLen = (n-1)*8 + lastBits;
        }
        else
        {
          *backLen = n*8;
        }

        if (n == 0)
        {
          n = 1;
        }
        if (n > MAX_LEN)
        {
          n = MAX_LEN;
        }

        // Reading the received data in FIFO
        for (i=0; i<n; i++)
        {
          backData[i] = read_mfrc522(FIFODataReg);
        }
      }
    }
    else {
      uart_printf("OVERFLOW\n");
      status = MI_ERR;
    }
  }
  else {
    uart_printf("TIMEOUT\n");
  }

  return status;
}

/*
 * Function Name：MFRC522_Request
 * Description: Find cards, read the card type number
 * Input parameters: reqMode - find cards way
 *   TagType - Return Card Type
 *    0x4400 = Mifare_UltraLight
 *    0x0400 = Mifare_One(S50)
 *    0x0200 = Mifare_One(S70)
 *    0x0800 = Mifare_Pro(X)
 *    0x4403 = Mifare_DESFire
 * Return value: the successful return MI_OK
 */
char MFRC522_Request(char reqMode, char *TagType)
{
  char status;
  uint32 backBits; // The received data bits

  write_mfrc522(BitFramingReg, 0x07);   // TxLastBists = BitFramingReg[2..0]

  TagType[0] = reqMode;

  status = MFRC522_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);
  if ((status != MI_OK) || (backBits != 0x10)) {
    status = MI_ERR;
  }

  return status;
}

/*
 * Function Name: MFRC522_Anticoll
 * Description: Anti-collision detection, reading selected card serial number card
 * Input parameters: serNum - returns 4 bytes card serial number, the first 5 bytes for the checksum byte
 * Return value: the successful return MI_OK
 */
char MFRC522_Anticoll(char *serNum)
{
  char status;
  char i;
  char serNumCheck=0;
  uint32 unLen;


  //ClearBitMask(Status2Reg, 0x08);     //TempSensclear
  //ClearBitMask(CollReg,0x80);         //ValuesAfterColl
  write_mfrc522(BitFramingReg, 0x00);       //TxLastBists = BitFramingReg[2..0]

  serNum[0] = PICC_ANTICOLL;
  serNum[1] = 0x20;
  status = MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

  if (status == MI_OK)
  {
    //Check card serial number
    for (i=0; i<4; i++)
    {   
      serNumCheck ^= serNum[i];
    }
    if (serNumCheck != serNum[i])
    {   
      status = MI_ERR;    
    }
  }

  //SetBitMask(CollReg, 0x80);      //ValuesAfterColl=1

  return status;
} 


/*
 * Function Name: CalulateCRC
 * Description: CRC calculation with MF522
 * Input parameters: pIndata - To read the CRC data, len - the data length, pOutData - CRC calculation results
 * Return value: None
 */
void CalulateCRC(char *pIndata, char len, char *pOutData)
{
  char i, n;

  write_mfrc522(CommandReg, PCD_IDLE);
  ClearBitMask(DivIrqReg, 0x04);            //CRCIrq = 0
  SetBitMask(FIFOLevelReg, 0x80);           //Clear the FIFO pointer

  //Writing data to the FIFO    
  for (i=0; i<len; i++)
  {   
    write_mfrc522(FIFODataReg, *(pIndata+i));   
  }
  write_mfrc522(CommandReg, PCD_CALCCRC);

  //Wait CRC calculation is complete
  i = 0xFF;
  do 
  {
    n = read_mfrc522(DivIrqReg);
    i--;
  }
  while ((i!=0) && !(n&0x04));          //CRCIrq = 1

  //Read CRC calculation result
  pOutData[0] = read_mfrc522(CRCResultRegL);
  pOutData[1] = read_mfrc522(CRCResultRegM);
}


/*
 * Function Name: MFRC522_SelectTag
 * Description: election card, read the card memory capacity
 * Input parameters: serNum - Incoming card serial number
 * Return value: the successful return of card capacity
 */
char MFRC522_SelectTag(char *serNum)
{
  char i;
  char status;
  char size;
  uint32 recvBits;
  char buffer[9]; 

  //ClearBitMask(Status2Reg, 0x08);         //MFCrypto1On=0

  buffer[0] = PICC_SElECTTAG;
  buffer[1] = 0x70;
  for (i=0; i<5; i++)
  {
    buffer[i+2] = *(serNum+i);
  }
  CalulateCRC(buffer, 7, &buffer[7]);       //??
  status = MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);

  if ((status == MI_OK) && (recvBits == 0x18))
  {   
    size = buffer[0]; 
  }
  else
  {   
    size = 0;    
  }

  return size;
}


/*
 * Function Name: MFRC522_Auth
 * Description: Verify card password
 * Input parameters: authMode - Password Authentication Mode
                 0x60 = A key authentication
                 0x61 = Authentication Key B
             BlockAddr--Block address
             Sectorkey--Sector password
             serNum--Card serial number, 4-byte
 * Return value: the successful return MI_OK
 */
char MFRC522_Auth(char authMode, char BlockAddr, char *Sectorkey, char *serNum)
{
  char status;
  uint32 recvBits;
  char i;
  char buff[12]; 

  //Verify the command block address + sector + password + card serial number
  buff[0] = authMode;
  buff[1] = BlockAddr;
  for (i=0; i<6; i++)
  {    
    buff[i+2] = *(Sectorkey+i);   
  }
  for (i=0; i<4; i++)
  {    
    buff[i+8] = *(serNum+i);   
  }
  status = MFRC522_ToCard(PCD_AUTHENT, buff, 12, buff, &recvBits);

  if ((status != MI_OK) || (!(read_mfrc522(Status2Reg) & 0x08)))
  {   
    status = MI_ERR;   
  }

  return status;
}


/*
 * Function Name: MFRC522_Read
 * Description: Read block data
 * Input parameters: blockAddr - block address; recvData - read block data
 * Return value: the successful return MI_OK
 */
char MFRC522_Read(char blockAddr, char *recvData)
{
  char status;
  uint32 unLen;

  recvData[0] = PICC_READ;
  recvData[1] = blockAddr;
  CalulateCRC(recvData,2, &recvData[2]);
  status = MFRC522_ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);

  if ((status != MI_OK) || (unLen != 0x90))
  {
    status = MI_ERR;
  }

  return status;
}


/*
 * Function Name: MFRC522_Write
 * Description: Write block data
 * Input parameters: blockAddr - block address; writeData - to 16-byte data block write
 * Return value: the successful return MI_OK
 */
char MFRC522_Write(char blockAddr, char *writeData)
{
  char status;
  uint32 recvBits;
  char i;
  char buff[18]; 

  buff[0] = PICC_WRITE;
  buff[1] = blockAddr;
  CalulateCRC(buff, 2, &buff[2]);
  status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);

  if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
  {   
    status = MI_ERR;   
  }

  if (status == MI_OK)
  {
    for (i=0; i<16; i++)        //Data to the FIFO write 16Byte
    {    
      buff[i] = *(writeData+i);   
    }
    CalulateCRC(buff, 16, &buff[16]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);

    if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
    {   
      status = MI_ERR;   
    }
  }

  return status;
}


/*
 * Function Name: MFRC522_Halt
 * Description: Command card into hibernation
 * Input: None
 * Return value: None
 */
void MFRC522_Halt(void)
{
  char status;
  uint32 unLen;
  char buff[4]; 

  buff[0] = PICC_HALT;
  buff[1] = 0;
  CalulateCRC(buff, 2, &buff[2]);

  status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff,&unLen);
}

void MFRC522_Sleep(void)
{
  MFRC522_Halt();
  // enable Soft power-down mode
  write_mfrc522(CommandReg, 0x30);
}

void MFRC522_CheckCard(unsigned char* serial)
{
    char status, checksum1, str[MAX_LEN];
    uint8 i;

    if (selfTestStatus == false) {
        return;
    }
    // Find cards
    status = MFRC522_Request(PICC_REQIDL, str);
    if (status == MI_OK) {
        // uart_printf("CARD:%x\t%x\n", str[0], str[1]);

        // Anti-collision, return card serial number == 4 bytes
        status = MFRC522_Anticoll(str);
        if (status == MI_OK) {
            checksum1 = str[0] ^ str[1] ^ str[2] ^ str[3];
            // uart_printf("SN: %x %x %x %x %x\t CHECKSUM: %x\n", str[0], str[1], str[2], str[3], str[4], checksum1);
            for (i = 0; i < 5; i++) {
                serial[i] = str[i];
            }
        }
    } else {
        // can't find card
        for (i = 0; i < 5; i++) {
            serial[i] = 0;
        }
    }
}

bool mfrc522_selftest(void) 
{
    uint8 i, n, result[64], version, res = 0;
    // Version 2.0 (0x92)
    // NXP Semiconductors; Rev. 3.8 - 17 September 2014; 16.1.1 self-test
    uint8 MFRC522_firmware_referenceV2_0[] = {
        0x00, 0xEB, 0x66, 0xBA, 0x57, 0xBF, 0x23, 0x95,
        0xD0, 0xE3, 0x0D, 0x3D, 0x27, 0x89, 0x5C, 0xDE,
        0x9D, 0x3B, 0xA7, 0x00, 0x21, 0x5B, 0x89, 0x82,
        0x51, 0x3A, 0xEB, 0x02, 0x0C, 0xA5, 0x00, 0x49,
        0x7C, 0x84, 0x4D, 0xB3, 0xCC, 0xD2, 0x1B, 0x81,
        0x5D, 0x48, 0x76, 0xD5, 0x71, 0x61, 0x21, 0xA9,
        0x86, 0x96, 0x83, 0x38, 0xCF, 0x9D, 0x5B, 0x6D,
        0xDC, 0x15, 0xBA, 0x3E, 0x7D, 0x95, 0x3B, 0x2F
    };

    write_mfrc522(CommandReg, PCD_GETRAND);
    i = 10;
    do {
        res = read_mfrc522(CommandReg);
        uart_printf("CMD:%x\n", res);
        waitUs(100);
        i--;
    } while (res & PCD_GETRAND && i > 0);

    if (i == 0) {
        return false;
    }
    
    for (i = 0; i < 10; i++) {
        result[i] = read_mfrc522(FIFODataReg);
        uart_printf("%x ", result[i]);
    }
    uart_printf("\n");
    
    // This follows directly the steps outlined in 16.1.1
    // 1. Perform a soft reset.
    mfrc522_reset();
    
    // 2. Clear the internal buffer by writing 25 bytes of 00h
    write_mfrc522(FIFOLevelReg, 0x80);      // flush the FIFO buffer
    res = read_mfrc522(FIFOLevelReg);
    uart_printf("DATA IN FIFO:%x\n", res & 0x7F);
    for (i = 0; i < 25; i++) {   
        write_mfrc522(FIFODataReg, 0x00);   
    }
    res = read_mfrc522(FIFOLevelReg);
    uart_printf("DATA IN FIFO:%x\n", res & 0x7F);
    
    write_mfrc522(CommandReg, PCD_MEM);     // transfer to internal buffer

    i = 10;
    do {
        res = read_mfrc522(CommandReg);
        uart_printf("CMD:%x\n", res);
        waitUs(100);
        i--;
    } while (res & PCD_MEM && i > 0);

    if (i == 0) {
        return false;
    }
    
    // 3. Enable self-test
    write_mfrc522(AutoTestReg, 0x09);
    
    // 4. Write 00h to FIFO buffer
    write_mfrc522(FIFODataReg, 0x00);
    
    // 5. Start self-test by issuing the CalcCRC command
    write_mfrc522(CommandReg, PCD_CALCCRC);
    
    // 6. Wait for self-test to complete
    for (i = 0; i < 0xFF; i++) {
        // The datasheet does not specify exact completion condition except
        // that FIFO buffer should contain 64 bytes.
        // While selftest is initiated by CalcCRC command
        // it behaves differently from normal CRC computation,
        // so one can't reliably use DivIrqReg to check for completion.
        // It is reported that some devices does not trigger CRCIRq flag
        // during selftest.
        n = read_mfrc522(FIFOLevelReg);
        uart_printf("n:%x\n", n);
        if (n >= 64) {
            break;
        }
    }
    write_mfrc522(CommandReg, PCD_IDLE);        // Stop calculating CRC for new content in the FIFO.
    
    // 7. Read out resulting 64 bytes from the FIFO buffer.
    for (i = 0; i < 64; i++) {
        result[i] = read_mfrc522(FIFODataReg);
    }
    // Auto self-test done
    // Reset AutoTestReg register to be 0 again. Required for normal operation.
    write_mfrc522(AutoTestReg, 0x00);
    
    // Determine firmware version (see section 9.3.4.8 in spec)
    version = read_mfrc522(VersionReg);
    uart_printf("VER:%X\n", version);
    
    // Verify that the results match up to our expectations
    for (i = 0; i < 64; i++) {
        if (result[i] != MFRC522_firmware_referenceV2_0[i]) {
            uart_printf("DIFF[%d]\:%x %x\n", i, result[i], MFRC522_firmware_referenceV2_0[i]);
            return false;
        }
    }

    i = 10;
    do {
        res = read_mfrc522(VersionReg);
        uart_printf("VER RES:%x\n", res);
        waitUs(100);
        i--;
    } while (res != 0x92 && i > 0);

    if (i == 0) {
        return false;
    }
    
    // Test passed; all is good.
    uart_printf("SELF TEST PASS\n");
    return true;
}

/*********************************************************************
*********************************************************************/
