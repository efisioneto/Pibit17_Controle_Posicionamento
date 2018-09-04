# Pibit17_Controle_Posicionamento

PROGRAMA QUE CONTROLA A LUMINOSIDADE DO LED PORINTERMÉDIO DA TÉCNICA DE PWM (PULSE WIDTHMODULATION) - ANEXO A

int x=0;
int z=5;
int v=9;
int i;

void setup() {
 // put your setup code here, to run once:
 Serial.begin(9600);
 pinMode(9,OUTPUT);
}
void loop() {
 // put your main code here, to run repeatedly:
    while(x<=255){
    analogWrite(v,x);
    x=x+z;
    delay(50);
}
x=255;
    while(x>=0){
    analogWrite(v,x);
    x=x-z;
    delay(50);
}
x=0;
}

PROGRAMA COM O USO DO LCD E DO MÓDULO NRF24L01JUNTAMENTE COM UMA PLATAFORMA ARDUINO - ANEXO B1

//Programa transmissor
//Este programa será responsável por transmitir uma mensagem de um computador por intermédio de
do monitor serial para um outro computador(receptor)
//- CONNECTIONS: nRF24L01 Modules See:
//http://arduino-info.wikispaces.com/Nrf24L01-2.4GHz-HowTo
//Uses the RF24 Library by TMRH2o here:
//https://github.com/TMRh20/RF24
//1 - GND
//2 - VCC 3.3V !!! NOT 5V
//3 - CE to Arduino pin 7
//4 - CSN to Arduino pin 8
//5 - SCK to Arduino pin 13
//6 - MOSI to Arduino pin 11
//7 - MISO to Arduino pin 12
//8 - UNUSED
//V1.02 02/06/2016
//Questions: terry@yourduino.com *///
#include <SPI.h>
#include "RF24.h" // Download and Install (See above)
// (Create an instance of a radio, specifying the CE and CS pins. )
RF24 myRadio (7, 8); // "myRadio" is the identifier you will use in following methods
/*-----( Declare Variables )-----*/
byte addresses[][6] = {"1Node"}; // Create address for 1 pipe.
char dataTransmitted; // Data that will be Transmitted from the transmitter
char b;
String hello;
int a = 0;
uint8_t buffer[32];//pode ser outra array, como bufer.
int i;
void setup() /****** SETUP: RUNS ONCE ******/
{
// Use the serial Monitor (Symbol on far right). Set speed to 115200 (Bottom Right)
Serial.begin(115200);
delay(1000);
Serial.println(F("RF24/Simple Transmit data Test"));
Serial.println(F("Questions: terry@yourduino.com"));
//dataTransmitted = 100; // Arbitrary known data to transmit. Change it to test...
myRadio.begin(); // Start up the physical nRF24L01 Radio
myRadio.setChannel(108); // Above most Wifi Channels
// Set the PA Level low to prevent power supply related issues since this is a
// getting_started sketch, and the likelihood of close proximity of the devices.
RF24_PA_MAX is default.
myRadio.setPALevel(RF24_PA_MIN);
// myRadio.setPALevel(RF24_PA_MAX); // Uncomment for more power
myRadio.openWritingPipe( addresses[0]); // Use the first entry in array 'addresses' (Only 1
right now)
delay(1000);
}
void loop() /****** LOOP: RUNS CONSTANTLY ******/
{
if(Serial.available()>0){
//buffer=Serial.read();
// read the incoming data as string
hello = Serial.readString();
hello.getBytes(buffer, sizeof(buffer));
myRadio.write(buffer,sizeof(buffer));
//myRadio.write( &dataTransmitted, sizeof(char) );
//dataTransmitted = Serial.read(); // Transmit the data
for(i=0;i<=31;i++){
buffer[i]= NULL;
}
delay(1000);
Serial.print(F("Data Transmitted = "));
Serial.println(hello);
delay(500);
}
}

PROGRAMA COM O USO DO LCD E DO MÓDULO NRF24L01JUNTAMENTE COM UMA PLATAFORMA ARDUINO - ANEXO B2
 
 // Programa receptor
//Este programa será responsável por receber uma mensagem de um computador por intermédio do
monitor serial e do LCD.
#include <LiquidCrystal.h>
LiquidCrystal LCD(10,9,5,4,3,2);
#include <SPI.h>
#include "RF24.h"
#define MAXSIZE 20
// (Create an instance of a radio, specifying the CE and CS pins. )
RF24 myRadio (7, 8); // "myRadio" is the identifier you will use in following methods
/*-----( Declare Variables )-----*/
byte addresses[ ][6] = {"1Node"}; // Create address for 1 pipe.
int i;
uint8_t buffer [32];
void setup() /****** SETUP: RUNS ONCE ******/
{
LCD.begin(16,2);
// Use the serial Monitor (Symbol on far right). Set speed to 115200 (Bottom Right)
Serial.begin(115200);
delay(1000);
Serial.println(F("RF24/Simple Receive data Test"));
Serial.println(F("Questions: terry@yourduino.com"));
myRadio.begin(); // Start up the physical nRF24L01 Radio
myRadio.setChannel(108); // Above most Wifi Channels
// Set the PA Level low to prevent power supply related issues since this is a
// getting_started sketch, and the likelihood of close proximity of the devices.
RF24_PA_MAX is default.
myRadio.setPALevel(RF24_PA_MIN);
// myRadio.setPALevel(RF24_PA_MAX); // Uncomment for more power
myRadio.openReadingPipe(1, addresses[0]); // Use the first entry in array 'addresses' (Only 1
right now)
myRadio.startListening();
}//--(end setup )---
void loop() /****** LOOP: RUNS CONSTANTLY ******/
{
if ( myRadio.available()>0) // Check for incoming data from transmitter
{
while (myRadio.available()) // While there is data ready
{
// myRadio.read(&dataReceived, sizeof(dataReceived) ); // Get the data payload (You
must have defined that already!)
Serial.println("Data received = ");
myRadio.read(buffer,sizeof(buffer));
for (i=0;i<=31;i++){
Serial.write(buffer[i]);
LCD.write(buffer[i]);}
delay(5000);
LCD.clear();
Serial.println(" ");
}
}
}

CÓDIGO DE TRANSMISSÃO - ANEXO C

/* ex_01a_simple_tx ,main.c */

#include "deca_device_api.h"
#include "deca_regs.h"
#include "sleep.h"
#include "LCD.h"
#include "port.h"

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* The frame sent in this example is an 802.15.4e standard blink. It is a 12-byte frame composed of the following fields:
 *     - byte 0: frame type (0xC5 for a blink).
 *     - byte 1: sequence number, incremented for each new frame.
 *     - byte 2 -> 9: device ID, see NOTE 1 below.
 *     - byte 10/11: frame check-sum, automatically set by DW1000.  */
static uint8 tx_msg[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E', 0, 0};
/* Index to access to sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 1

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 1000

/**
 * Application entry point.
 */
int main(void)
{
    /* Start with board specific hardware init. */
    peripherals_init();

    /* Reset and initialise DW1000. See NOTE 2 below.
     * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
     * performance. */
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    spi_set_rate_low();
    if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR)
    {
        while (1)
        { };
    }
    spi_set_rate_high();

    /* Configure DW1000. See NOTE 3 below. */
    dwt_configure(&config);

    /* Loop forever sending frames periodically. */
    while(1)
    {
        /* Write frame data to DW1000 and prepare transmission. See NOTE 4 below.*/
        dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(tx_msg), 0, 0); /* Zero offset in TX buffer, no ranging. */

        /* Start transmission. */
        dwt_starttx(DWT_START_TX_IMMEDIATE);

        /* Poll DW1000 until TX frame sent event set. See NOTE 5 below.
         * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
         * function to access it.*/
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
        { };

        /* Clear TX frame sent event. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

        /* Execute a delay between transmissions. */
        sleep_ms(TX_DELAY_MS);

        /* Increment the blink frame sequence number (modulo 256). */
        tx_msg[BLINK_FRAME_SN_IDX]++;
    }


CÓDIGO DE RECEPÇÃO - ANEXO D

#include "deca_device_api.h"
#include "deca_regs.h"
#include "port.h"

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Buffer to store received frame. See NOTE 1 below. */
#define FRAME_LEN_MAX 127
static uint8 rx_buffer[FRAME_LEN_MAX];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
static uint16 frame_len = 0;

/**
 * Application entry point.
 */
int main(void)
{
    /* Start with board specific hardware init. */
    peripherals_init();

    /* Reset and initialise DW1000. See NOTE 2 below.
     * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
     * performance. */
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    spi_set_rate_low();
    if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR)
    {
        while (1)
        { };
    }
    spi_set_rate_high();

    /* Configure DW1000. */
    dwt_configure(&config);

    /* Loop forever receiving frames. */
    while (1)
    {
        int i;

        /* TESTING BREAKPOINT LOCATION #1 */

        /* Clear local RX buffer to avoid having leftovers from previous receptions  This is not necessary but is included here to aid reading
         * the RX buffer.
         * This is a good place to put a breakpoint. Here (after first time through the loop) the local status register will be set for last event
         * and if a good receive has happened the data buffer will have the data in it, and frame_len will be set to the length of the RX frame. */
        for (i = 0 ; i < FRAME_LEN_MAX; i++ )
        {
            rx_buffer[i] = 0;
        }

        /* Activate reception immediately. See NOTE 3 below. */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        /* Poll until a frame is properly received or an error/timeout occurs. See NOTE 4 below.
         * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
         * function to access it. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
        { };

        if (status_reg & SYS_STATUS_RXFCG)
        {
            /* A frame has been received, copy it to our local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
            if (frame_len <= FRAME_LEN_MAX)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            /* Clear good RX frame event in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
        }
        else
        {
            /* Clear RX error events in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }
    }

 CÓDIGO DE INICIAÇÃO DA TRANSMISSÃO PARA SS-TWR - ANEXO E
 
 /* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"
#include "port.h"
#include <stdio.h>
#include <string.h>
#include "deca_device_api.h"
#include "deca_regs.h"
#include "sleep.h"

SPI_HandleTypeDef hspi1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

extern void dw_main(void);

/* Example application name and version to display on LCD screen. */
#define APP_NAME "SS TWR INIT v1.3"

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

/* Default communication configuration. We use here EVK1000's mode 4. See NOTE 1 below. */
static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (129 + 8 - 8)    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

/* Frames used in the ranging process. See NOTE 3 below. */
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4

#define FREQ_OFFSET_MULTIPLIER          (998.4e6/2.0/1024.0/131072.0)
#define HERTZ_TO_PPM_MULTIPLIER_CHAN_2     (-1.0e6/3993.6e6)

/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 20
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 140
/* Receive response timeout. See NOTE 5 below. */
//#define RESP_RX_TIMEOUT_UUS 210
#define RESP_RX_TIMEOUT_UUS 4000

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

/* String used to display measured distance on LCD screen (16 characters maximum). */
char dist_str[16] = {0};

/* Declaration of static functions. */
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts);

int main(void)
{
	 HAL_Init();

  	 SystemClock_Config();

  	 MX_GPIO_Init();
  	 MX_SPI1_Init();
  	 //MX_USB_DEVICE_Init();

  	 setup_DW1000RSTnIRQ(0);
     //port_DisableEXT_IRQ();
     //dw_main();
     /* Start with board specific hardware init. */
     peripherals_init();

     /* Reset and initialise DW1000.
      * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
      * performance. */
     reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
     port_set_dw1000_slowrate();
     if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
     {
         while (1)
         { };
     }
     port_set_dw1000_fastrate();

     /* Configure DW1000. See NOTE 6 below. */
     dwt_configure(&config);

     /* Apply default antenna delay value. See NOTE 2 below. */
     dwt_setrxantennadelay(RX_ANT_DLY);
     dwt_settxantennadelay(TX_ANT_DLY);

     /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
      * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
     dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
     dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

     /* Loop forever initiating ranging exchanges. */
     while (1)
     {
         /* Write frame data to DW1000 and prepare transmission. See NOTE 7 below. */
         tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
         dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
         dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
         dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

         /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
          * set by dwt_setrxaftertxdelay() has elapsed. */
         dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

         /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
         while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
         { };

         /* Increment frame sequence number after transmission of the poll message (modulo 256). */
         frame_seq_nb++;

         if (status_reg & SYS_STATUS_RXFCG)
         {
             uint32 frame_len;

             /* Clear good RX frame event in the DW1000 status register. */
             dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

             /* A frame has been received, read it into the local buffer. */
             frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
             if (frame_len <= RX_BUF_LEN)
             {
                 dwt_readrxdata(rx_buffer, frame_len, 0);
             }

             /* Check that the frame is the expected response from the companion "SS TWR responder" example.
              * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
             rx_buffer[ALL_MSG_SN_IDX] = 0;
             if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
             {
                 uint32 poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
                 int32 rtd_init, rtd_resp;
                 float clockOffsetRatio ;

                 /* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
                 poll_tx_ts = dwt_readtxtimestamplo32();
                 resp_rx_ts = dwt_readrxtimestamplo32();

                 /* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
                 //clockOffsetRatio = dwt_readcarrierintegrator() * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_2 / 1.0e6) ;
                 clockOffsetRatio = 0;


                 /* Get timestamps embedded in response message. */
                 resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
                 resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

                 /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
                 rtd_init = resp_rx_ts - poll_tx_ts;
                 rtd_resp = resp_tx_ts - poll_rx_ts;

                 tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
                 distance = tof * SPEED_OF_LIGHT;
             }
         }
         else
         {
             /* Clear RX error/timeout events in the DW1000 status register. */
             dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

             /* Reset RX to properly reinitialise LDE operation. */
             dwt_rxreset();
         }

         /* Execute a delay between ranging exchanges. */
         Sleep(RNG_DELAY_MS);
     }
}
/*!
 * @fn resp_msg_get_ts()
 *
 * @brief Read a given timestamp value from the response message. In the timestamp fields of the response message, the
 *        least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to get
 *         ts  timestamp value
 *
 * @return none
 */
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < RESP_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 5, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

	  GPIO_InitTypeDef GPIO_InitStruct;

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

	  /*Configure GPIO pin : PC13 */
	  GPIO_InitStruct.Pin = GPIO_PIN_13;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pin : PA0 */
	  GPIO_InitStruct.Pin = GPIO_PIN_0;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pin : PA4 */
	  GPIO_InitStruct.Pin = GPIO_PIN_4;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pin : PB0 */
	  GPIO_InitStruct.Pin = GPIO_PIN_0;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /*Configure GPIO pin : PB5 */
	  GPIO_InitStruct.Pin = GPIO_PIN_5;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /* EXTI interrupt init*/
	  HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);
	  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
	  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

CÓDIGO DE RESPOSTA À INICIAÇÃO DA TRANSMISSÃO PARA SS-TWR -ANEXO F

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"
#include "port.h"
#include <string.h>
#include "deca_device_api.h"
#include "deca_regs.h"
//#include "LCD.h"



SPI_HandleTypeDef hspi1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

extern void dw_main(void);


/* Example application name and version to display on LCD screen. */
#define APP_NAME "SS TWR RESP v1.2"

/* Default communication configuration. We use here EVK1000's mode 4. See NOTE 1 below. */
static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (129 + 8 - 8)    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

/* Frames used in the ranging process. See NOTE 3 below. */
static uint8 rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8 tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 12
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
//#define POLL_RX_TO_RESP_TX_DLY_UUS 330
#define POLL_RX_TO_RESP_TX_DLY_UUS 500

/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;

/* Declaration of static functions. */
static uint64 get_rx_timestamp_u64(void);
static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts);

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();

  setup_DW1000RSTnIRQ(0);
  //port_DisableEXT_IRQ();
  //dw_main();
  /* Start with board specific hardware init. */
  peripherals_init();

  /* Display application name on LCD. */
  //  LCD_display_str(APP_NAME);

  /* Reset and initialise DW1000.
  * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
  * performance. */
  reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
  port_set_dw1000_slowrate();
  if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
  {
	  //LCD_display_str("INIT FAILED");
      while (1)
      { };
  }
  port_set_dw1000_fastrate();

  /* Configure DW1000. See NOTE 5 below. */
  dwt_configure(&config);

  /* Apply default antenna delay value. See NOTE 2 below. */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  /* Loop forever responding to ranging requests. */
  while (1)
  {
      /* Activate reception immediately. */
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
       /* Poll for reception of a frame or error/timeout. See NOTE 6 below. */
      while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
      { };
      if (status_reg & SYS_STATUS_RXFCG)
      {
          uint32 frame_len;
          /* Clear good RX frame event in the DW1000 status register. */
          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
          /* A frame has been received, read it into the local buffer. */
          frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
          if (frame_len <= RX_BUFFER_LEN)
          {
              dwt_readrxdata(rx_buffer, frame_len, 0);
          }
          /* Check that the frame is a poll sent by "SS TWR initiator" example.
           * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
           rx_buffer[ALL_MSG_SN_IDX] = 0;
           if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
           {
        	   uint32 resp_tx_time;
               int ret;

               /* Retrieve poll reception timestamp. */
               poll_rx_ts = get_rx_timestamp_u64();

                /* Compute final message transmission time. See NOTE 7 below. */
                resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(resp_tx_time);

                /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
                resp_tx_ts = (((uint64)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

                /* Write all timestamps in the final message. See NOTE 8 below. */
                resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
                resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

                /* Write and send the response message. See NOTE 9 below. */
                tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
                dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
                ret = dwt_starttx(DWT_START_TX_DELAYED);

                /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 10 below. */
                if (ret == DWT_SUCCESS)
                {
                    /* Poll DW1000 until TX frame sent event set. See NOTE 6 below. */
                    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
                    { };

                    /* Clear TXFRS event. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

                    /* Increment frame sequence number after transmission of the poll message (modulo 256). */
                    frame_seq_nb++;
                  }
           }
      }
      else
      {
    	  /* Clear RX error events in the DW1000 status register. */
          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

          /* Reset RX to properly reinitialise LDE operation. */
          dwt_rxreset();
      }
  }
}

/*!
 * @fn final_msg_set_ts()
*
* @brief Get the RX time-stamp in a 64-bit variable.
*        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
*
* @param  none
*
* @return  64-bit value of the read time-stamp.
*/
static uint64 get_rx_timestamp_u64(void)
{
   uint8 ts_tab[5];
   uint64 ts = 0;
   int i;
   dwt_readrxtimestamp(ts_tab);
   for (i = 4; i >= 0; i--)
   {
       ts <<= 8;
       ts |= ts_tab[i];
   }
   return ts;
}

/*!
* @fn final_msg_set_ts()
*
* @brief Fill a given timestamp field in the response message with the given value. In the timestamp fields of the
*        response message, the least significant byte is at the lower address.
*
* @param  ts_field  pointer on the first byte of the timestamp field to fill
*         ts  timestamp value
*
* @return none
*/
static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts)
{
   int i;
   for (i = 0; i < RESP_MSG_TS_LEN; i++)
   {
       ts_field[i] = (ts >> (i * 8)) & 0xFF;
   }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 5, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

	  GPIO_InitTypeDef GPIO_InitStruct;

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

	  /*Configure GPIO pin : PC13 */
	  GPIO_InitStruct.Pin = GPIO_PIN_13;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pin : PA0 */
	  GPIO_InitStruct.Pin = GPIO_PIN_0;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pin : PA4 */
	  GPIO_InitStruct.Pin = GPIO_PIN_4;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pin : PB0 */
	  GPIO_InitStruct.Pin = GPIO_PIN_0;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /*Configure GPIO pin : PB5 */
	  GPIO_InitStruct.Pin = GPIO_PIN_5;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /* EXTI interrupt init*/
	  HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);
	  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
	  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

CÓDIGO  CORRESPONDENTE  AO  ALGORITMO  PARA  CÁLCULO  DAPOSIÇÃO DO QUADRIRROTOR - ANEXO G

#include <stdio.h>
#include <math.h>
float *PosicaoQuadrirrotor(float d12,float d13, float d14, float d15,float d23 ,float d24, float d25,float d34,float d35, float d45){
int i=0;
float P2[3],P4[3],P5[3],P1[3],P3[3];
float *v;
float z3=0;
// P1 é o sink node, origem do sistema de coordenadas
// P3 é o alvo
for(i=0;i<3;i++){
    P3[i]=0;
    P2[i]=0;
    P4[i]=0;
    P5[i]=0;
    P1[i]=0;
}
//DETERMINADO P2
P2[0]=d12;
//Determinando P5
P5[0]=(pow(d15,2) - pow(d25,2)+pow(P2[0],2))/(2*P2[0]);
P5[1]=sqrt(pow(d15,2)-pow(P5[0],2));
//DETERMINANDO P4
P4[0]=(pow(P2[0],2)-pow(d24,2)+pow(d14,2))/(2*P2[0]);
P4[1]=(pow(P5[0],2)+pow(P5[1],2)-pow(d45,2)+pow(d14,2)-2*(P4[0]*P5[0]))/(2*P5[0]);
P4[2]=sqrt(pow(d14,2)-pow(P4[0],2)-pow(P4[1],2));
//DETERMINIANDO P3
P3[0]=(pow(P2[0],2)-pow(d23,2)+pow(d13,2))/(2*P2[0]);
P3[1]=(pow(P5[0],2)+pow(P5[1],2)-pow(d35,2)+pow(d13,2)-2*(P3[0]*P5[0]))/(2*P5[1]);
P3[2]=sqrt(pow(d13,2)-pow(P3[1],2)-pow(P3[0],2));
v=P3;
return v;
}


