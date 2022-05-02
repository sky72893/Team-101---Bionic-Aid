/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.7
        Device            :  PIC18F66J65
        Driver Version    :  2.00
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/i2c1_master.h"
#include "mcc_generated_files/examples/i2c1_master_example.h" //includes 12C example header file
#include "mcc_generated_files/eusart1.h"
#include <xc.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define SELF_TEST               0x6D
#define STATUS                  0x1B
#define SELF_CONG_1             0x7B
#define PMU_STATUS              0x03
#define PMU_TRIGGER             0x6C
#define CMD                     0x7E
#define I2C_SLAVE_ADDR          0x68
#define BMX160_GYR_X1           0x0C
#define BMX160_GYR_X2           0x0D
#define BMX160_GYR_Y1           0x0E
#define BMX160_GYR_Y2           0x0F
#define BMX160_GYR_Z1           0x10
#define BMX160_GYR_Z2           0x11
#define GYR_RANGE               0x43
#define GYR_CONF                0x42

/*void Custom_ISR(void) //Start of ISR2 (Interrupt Service Routine)
{     
  EUSART1_Receive_ISR();  //Call the default 
  
        if(EUSART1_is_rx_ready()) //If it received data and can be read
            {
                rxData = EUSART1_Read(); //a. Read byte from buffer and save into variable
                
                if(EUSART1_is_tx_ready()) //if it can send data
                {
                    EUSART1_Write(rxData); //send the data
                }
            }
}*/

uint16_t convertedValue; //defines variable of value from ADC 
int calculatedValue;
uint8_t rxData; //data to be read from eusart
uint8_t dataGyro;
uint16_t xval1Gyro;
uint16_t yval1Gyro;
uint16_t zval1Gyro;
uint16_t xval2Gyro;
uint16_t yval2Gyro;
uint16_t zval2Gyro;
uint16_t xval_newGyro;
uint16_t yval_newGyro;
uint16_t zval_newGyro;
i2c1_address_t id = 0x28;//write
uint8_t data[7]; // holds output data
uint8_t cmd[3] = {0xAA, 0x00, 0x00}; // command to be sent
uint8_t press_counts = 0; // digital pressure reading [counts]
uint8_t pressure = 0; // pressure reading [bar, psi, kPa, etc.]
uint8_t outputmax = 15099494; // output at maximum pressure [counts]
uint8_t outputmin = 1677722; // output at minimum pressure [counts]
uint8_t pmax = 1; // maximum value of pressure range [bar, psi, kPa, etc.]
uint8_t pmin = 0; // minimum value of pressure range [bar, psi, kPa, etc.]
uint8_t percentage = 0; // holds percentage of full scale data
char output[50];

  float   rxDataConvert = 0;

  int rx_Convert = 1; //receive
  int tx_Convert = 0; //transmit
  


void main(void){
    // Initialize the device
    SYSTEM_Initialize();
    ADC_Initialize(); // initialize for ADC thermistor 
    I2C1_Initialize(); // initialize for I2C of sensors (gyro and pressure)
    EUSART1_Initialize(); //initialize EUSART to send to ESP32
    //EUSART1_SetRxInterruptHandler(Custom_ISR); // sets handler to send to esp
    
    int newVariable; // sets variable for idk yet
    
    adc_result_t convertedValue; 

    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global and Peripheral Interrupts
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    
    //RB5 = Red
    //RB4 = Green
    //RF7 = Blue
    //RB3 = White

    while (1)
    {
        IO_RB3_SetHigh(); // Lets us know the board is on and powered
        ADC_SelectChannel(channel_AN0); //tell the adc to read signal from that port
        ADC_StartConversion(); //start the conversion of data
        convertedValue = ADC_GetConversionResult(); //set the converted value to the value that is converted
        calculatedValue = convertedValue * 0.0757; //Multiply decimal value by bit per temperature range
        
        PWM5_Initialize();
        PWM4_Initialize();
        int maxspd3 = 1023;
        int maxspd5 = 1023;
        int minspd3 = 0;
        int minspd5 = 0;
        int tstep = 100; 
        
        if (calculatedValue >= 80)
        {
            IO_RB4_SetHigh(); // Turns red to know it has exceeded temperature
        }
        else
        {
            IO_RB4_SetHigh(); // Turns red to know if it has not exceeded temperature
        }
        
       I2C1_Write1ByteRegister(I2C_SLAVE_ADDR, SELF_TEST, 0x1B); //0x1B
       I2C1_Write1ByteRegister(I2C_SLAVE_ADDR, STATUS, 0b0100000); //0b0100000
       dataGyro = I2C1_Read1ByteRegister(I2C_SLAVE_ADDR, STATUS); 
               
       I2C1_Write1ByteRegister(I2C_SLAVE_ADDR, GYR_CONF, 0b1000); //0b1100 
       I2C1_Write1ByteRegister(I2C_SLAVE_ADDR, GYR_RANGE, 0b100); //0b010
       I2C1_Write1ByteRegister(I2C_SLAVE_ADDR, SELF_CONG_1, 0x03); //0x03
       I2C1_Write1ByteRegister(I2C_SLAVE_ADDR, PMU_STATUS, 0b01); //0b01
       I2C1_Write1ByteRegister(I2C_SLAVE_ADDR, PMU_TRIGGER, 0b10); //0b10
       I2C1_Write1ByteRegister(I2C_SLAVE_ADDR, CMD, 0x15); //0x15
               
       xval1Gyro = I2C1_Read2ByteRegister(I2C_SLAVE_ADDR, BMX160_GYR_X1) / 720;
       xval2Gyro = I2C1_Read2ByteRegister(I2C_SLAVE_ADDR, BMX160_GYR_X2) / 720;
       yval1Gyro = I2C1_Read2ByteRegister(I2C_SLAVE_ADDR, BMX160_GYR_Y1) / 720;
       yval2Gyro = I2C1_Read2ByteRegister(I2C_SLAVE_ADDR, BMX160_GYR_Y2) / 720;
       zval1Gyro = I2C1_Read2ByteRegister(I2C_SLAVE_ADDR, BMX160_GYR_Z1) / 720;     
       zval2Gyro = I2C1_Read2ByteRegister(I2C_SLAVE_ADDR, BMX160_GYR_Z2) / 720;    
       
       xval_newGyro = xval1Gyro | (xval2Gyro<<8)/720; 
       yval_newGyro = yval1Gyro | (yval2Gyro<<8)/720; 
       zval_newGyro = zval1Gyro | (zval2Gyro<<8)/720; 
       
       if (xval1Gyro > 70 && yval1Gyro < 35)
       {
           IO_RF7_SetHigh();
       }
       else
       {
           IO_RF7_SetLow();
           __delay_ms(50);
           IO_RF7_SetHigh();
           __delay_ms(50);
       }
       
       //motors
       
        PWM5_LoadDutyValue(0);
        PWM4_LoadDutyValue(0);
        PWM5_LoadDutyValue(maxspd5);
        __delay_ms(500);
        PWM5_LoadDutyValue(minspd5);
        __delay_ms(500);
        
        
        I2C1_WriteNBytes(id,cmd,7);
        __delay_ms(100);
        I2C1_ReadNBytes(id, data);   
        IO_RB5_SetHigh(); //test led to make sure its running the program
        press_counts = data[3] + data[2] * 256 + data[1] * 65536; // calculate digital pressure counts
        percentage = (press_counts / 16777215) * 100; // calculate pressure as percentage of full scale
        pressure = ((press_counts - outputmin) * (pmax - pmin)) / (outputmax - outputmin) + pmin;
        printf("Digital Pressure Count  = %3.3f ;", press_counts);//outputs to putty the pressure should be really high number
        __delay_ms(100);
        printf("Percentage of Pressure = %2.2f ;", percentage);//
        __delay_ms(100);
        printf("Pressure = %2.8f ;", pressure); //should be the pressure in psi should be pretty low
        __delay_ms(2000);

        /*
        PWM5_LoadDutyValue(0);
        for (int i = minspd4; i<maxspd4; i++)
            {
                PWM4_LoadDutyValue(i);
                delay_ms(tstep);
            }
        for (int i = maxspd4; i>minspd4; i--)
            {
                PWM4_LoadDutyValue(i);
                delay_ms(tstep);
            } 
        PWM4_LoadDutyValue(0);
        for (int i = minspd5; i<maxspd5; i++)
            {
                PWM5_LoadDutyValue(i);
                delay_ms(tstep);
            }
        for (int i = maxspd5; i>minspd5; i--)
            {
                PWM5_LoadDutyValue(i);
                delay_ms(tstep);
            } 
        */
    }  
}

void txGyro(void)
{
    if(tx_Convert == 1){ // if tx is ready
        while(!EUSART1_is_tx_ready()); // wait until tx is ready
        printf("Read X Data: X1:%d | X2:%d | Read X: %d ;", xval1Gyro, xval2Gyro, xval_newGyro);
        sprintf(output, "Read Y Data: Y1:%d | Y2:%d | Read Y: %d ;", xval1Gyro, xval2Gyro, xval_newGyro);
        printf("Read Y Data: Y1:%d | Y2:%d | Read Y: %d ;", yval1Gyro, yval2Gyro, yval_newGyro);
        sprintf(output, "Read Y Data: Y1:%d | Y2:%d | Read Y: %d ;", yval1Gyro, yval2Gyro, yval_newGyro);
        printf("Read Z Data: Z1:%d | Z2:%d | Read Z: %d ;", zval1Gyro, zval2Gyro, zval_newGyro);
        sprintf(output, "Read Y Data: Y1:%d | Y2:%d | Read Y: %d ;", zval1Gyro, zval2Gyro, zval_newGyro);
        printf("Read Data: %s ;", dataGyro);
        sprintf(output,"Read Y Data: Y1:%d | Y2:%d | Read Y: %d ;", dataGyro);
        EUSART1_Write(output);
        __delay_ms(3000);
        while(!EUSART1_is_tx_done());
 }
}
