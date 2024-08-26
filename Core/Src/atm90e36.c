/*
 * atm90e36.c
 *
 *  Created on: Aug 8, 2024
 *      Author: bsaig
 */
#include"atm90e36.h"
#include"stdio.h"
#include "stm32h5xx_hal_spi.h"
extern SPI_HandleTypeDef hspi1;
uint16_t flag = 0;
//uint16_t volts = 0;
//uint16_t current = 0;
//uint16_t power = 0;



typedef struct {
    uint16_t cmd;       // Command to be sent
    uint32_t addr;      // Address for the transaction
    size_t length;      // Length of the data to be transmitted in bits
    size_t rxlength;    // Length of the data to be received in bits
    uint8_t *tx_buffer; // Pointer to the transmit buffer
    uint8_t *rx_buffer; // Pointer to the receive buffer
} spi_transaction_t;


void readEnrgyMeterValues(void)
{
	flag = 1;

	uint16_t temp     =       CommEnergyIC(&hspi1, ATM90E36A_READ, Temp, 0);//UrmsA
	uint16_t volts    =      CommEnergyIC(&hspi1, ATM90E36A_READ, UrmsA, 0);
	uint16_t current  =    CommEnergyIC(&hspi1, ATM90E36A_READ, IrmsA, 0);
//	uint16_t  power    =      CommEnergyIC(&hspi1, ATM90E36A_READ, PmeanAF, 0);

	 printf("temp : %d\r\n",temp);
//	 printf("Voltage: %f \n",(volts * 0.01 * 0.989649057 + 0.315538775));
	 printf("volts %d\r\n",volts/100);
	 printf("current : %d\r\n",current);
//	 printf("power : %d\r\n",power);




}



uint16_t CommEnergyIC(SPI_HandleTypeDef *hspi, uint16_t RW, uint16_t address, uint16_t val)
{


//	 HAL_SPI_Transmit(&hspi2, &transaction, sizeof(transaction), HAL_MAX_DELAY);
	  uint16_t readCommand = address | RW;
	    uint8_t tx_data[4] = {(readCommand >> 8) & 0xFF, readCommand & 0xFF, (val >> 8) & 0xFF, val & 0xFF};
	    uint8_t rx_data[4] = {0, 0, 0, 0};


//	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);


	    HAL_Delay(10);

//	    spi_transaction_t transaction = {
//	        .cmd = 0,
//	        .addr = 0,
//	        .length = 32,   // 2 bytes (16 bits)
//	        .rxlength = 32, // 2 bytes (16 bits)
//	        .tx_buffer = tx_data,
//	        .rx_buffer = rx_data,
//	    };


//	   HAL_SPI_Transmit(&hspi1, &transaction, sizeof(transaction), HAL_MAX_DELAY);

	    HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 4, HAL_MAX_DELAY);


	    HAL_Delay(10);


//	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);


         return ((rx_data[2] << 8) | rx_data[3]);



}






void ATM90E36A_Init(void ) {

//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    // Example initialization sequence - adjust according to your needs and the datasheet
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, SoftReset, 0x789A); // Perform soft reset//lastdata
//	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, lastdata, 0x0000);

	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, FuncEn0, 0x0000);   // Voltage sag
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, FuncEn1, 0x0000);   // Voltage sag
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, SagTh, 0x0001);     // Voltage sag threshold

	    // Set metering config values (CONFIG)
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, ConfigStart, 0x5678); // Metering calibration startup
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, PLconstH, 0x0861);    // PL Constant MSB (default)
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, PLconstL, 0xC468);    // PL Constant LSB (default)
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, MMode0, 0x1087);      // Mode Config (60 Hz, 3P4W)
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, MMode1, 0x1500);      // 0x5555 (x2) // 0x0000 (1x)
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, PStartTh, 0x0000);    // Active Startup Power Threshold
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, QStartTh, 0x0000);    // Reactive Startup Power Threshold
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, SStartTh, 0x0000);    // Apparent Startup Power Threshold
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, PPhaseTh, 0x0000);    // Active Phase Threshold
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, QPhaseTh, 0x0000);    // Reactive Phase Threshold
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, SPhaseTh, 0x0000);    // Apparent  Phase Threshold
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, CSZero, 0x4741);      // Checksum 0

	    // Set metering calibration values (CALIBRATION)
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, CalStart, 0x5678); // Metering calibration startup
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, GainA, 0x0000);    // Line calibration gain
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, PhiA, 0x0000);     // Line calibration angle
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, GainB, 0x0000);    // Line calibration gain
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, PhiB, 0x0000);     // Line calibration angle
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, GainC, 0x0000);    // Line calibration gain
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, PhiC, 0x0000);     // Line calibration angle
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, PoffsetA, 0x0000); // A line active power offset
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, QoffsetA, 0x0000); // A line reactive power offset
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, PoffsetB, 0x0000); // B line active power offset
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, QoffsetB, 0x0000); // B line reactive power offset
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, PoffsetC, 0x0000); // C line active power offset
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, QoffsetC, 0x0000); // C line reactive power offset
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, CSOne, 0x0000);    // Checksum 1

	    // Set metering calibration values (HARMONIC)
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, HarmStart, 0x5678); // Metering calibration startup
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, POffsetAF, 0x0000); // A Fund. active power offset
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, POffsetBF, 0x0000); // B Fund. active power offset
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, POffsetCF, 0x0000); // C Fund. active power offset
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, PGainAF, 0x0000);   // A Fund. active power gain
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, PGainBF, 0x0000);   // B Fund. active power gain
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, PGainCF, 0x0000);   // C Fund. active power gain
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, CSTwo, 0x0000);     // Checksum 2

	    // Set measurement calibration values (ADJUST)
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, AdjStart, 0x5678); // Measurement calibration
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, UgainA, 0x0002);   // A SVoltage rms gain
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, IgainA, 0xFD7F);   // A line current gain
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, UoffsetA, 0x0000); // A Voltage offset
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, IoffsetA, 0x0000); // A line current offset
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, UgainB, 0x0002);   // B Voltage rms gain
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, IgainB, 0xFD7F);   // B line current gain
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, UoffsetB, 0x0000); // B Voltage offset
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, IoffsetB, 0x0000); // B line current offset
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, UgainC, 0x0002);   // C Voltage rms gain
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, IgainC, 0xFD7F);   // C line current gain
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, UoffsetC, 0x0000); // C Voltage offset
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, IoffsetC, 0x0000); // C line current offset
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, IgainN, 0xFD7F);   // C line current gain
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, CSThree, 0x02F6);  // Checksum 3

	    // Done with the configuration
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, ConfigStart, 0x5678);
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, CalStart, 0x5678);  // 0x6886 //0x5678 //8765);
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, HarmStart, 0x5678); // 0x6886 //0x5678 //8765);
	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, AdjStart, 0x5678);  // 0x6886 //0x5678 //8765);

	    CommEnergyIC(&hspi1, ATM90E36A_WRITE, SoftReset, 0x789A); // Perform soft reset
//	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);


}



