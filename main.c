#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/examples/i2c2_master_example.h"
#include <stdio.h>
#include <math.h>

#define Address 0x36 
#define Angle1 0x0E
#define Angle2 0x0F
#define Ticks 4096
#define Radius 10.08 //radius in cm 

#define MCP3426_ADDRESS 0b1101000 

//############################# PRINT #########################################
volatile uint8_t rxData;

void Custom_EUSART1_Receive_ISR(void)
{
    // Call the default EUSART1 RX ISR to properly transfer bytes out of the EUSART2 character buffer
    EUSART1_Receive_ISR();

    // Check if the EUSART1 receiver has received data and is ready to be read
    if (EUSART1_is_rx_ready())
    {
        // Read a byte from the EUSART1 buffer and save it into a variable of the proper data type
        rxData = EUSART1_Read();

        // Loop until the EUSART1 transmitter is ready to accept and transmit a data byte
        if(EUSART1_is_tx_ready())
        {
            EUSART1_Write(rxData); //  Write the byte you just read to EUSART1(Echo it back to the terminal)
        }
       
        __delay_ms(10);

    }
}

uint16_t timer_ms = 1;
uint16_t timer_s = 1;
float time = 0;

uint16_t Position[2];
uint16_t Time[2];

uint16_t HallRead(void) {
    return (uint16_t) (I2C2_Read1ByteRegister(Address, Angle1) << 8 | I2C2_Read1ByteRegister(Address, Angle2));
}

void timer_callback(void) {
    timer_ms = timer_ms + 1;
    if (timer_ms >= 1000) {
        timer_ms = timer_ms - 1000;
        timer_s = timer_s + 1;
    }
}

//########################### ADC ->(Voltage Sensor aka Moisture Sensor########
bool check_adc_reading(void) // Goes back and forth between liking boolean
{
    uint8_t data[2];  // Buffer array to store the conversion result
    uint8_t config_reg[1] = {0x10};  // Configuration register value for MCP3426
    
    I2C2_WriteNBytes(MCP3426_ADDRESS, config_reg, 1); 
    __delay_ms(15); // Wait for conversion to complete
    
    I2C2_ReadNBytes(MCP3426_ADDRESS, data, 2);  // Read the data from the MCP3426
    int16_t result = (data[0] << 8) | data[1];  // Combine the two bytes to form a 16-bit signed integer (large range)
    __delay_ms(15);    // Wait for the conversion to complete

    printf("ADC Result: %d\n", result);
    if(result < 35){
       return true;
    }
    else{
        return false;
    }
}

//############################ BI-MOTOR #######################################
void spin_motor_cw(void)
{
    uint8_t dir1 = 0b11001111;
    uint8_t receive;

    SPI1_Open(SPI1_DEFAULT);
    SS_pin_SetLow();
    //printf("Send: %u\r\n",dir1);

    receive = SPI1_ExchangeByte(dir1);
    //printf("Receive: %u\r\n",receive);
    printf("CW \r\n");
    SS_pin_SetHigh();
    SPI1_Close();

    __delay_ms(10);
}

void spin_motor_ccw(void)
{
    uint8_t dir2 = 0b11001101;
    uint8_t receive;

    SPI1_Open(SPI1_DEFAULT);
    SS_pin_SetLow();
    printf("CCW \r\n");

    receive = SPI1_ExchangeByte(dir2);
    //printf("Receive: %u\r\n",receive);
    SS_pin_SetHigh();
    SPI1_Close();
    
    __delay_ms(10);
}


void main(void) {

    SYSTEM_Initialize();
    //    TMR2_StartTimer();
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();
    TMR2_SetInterruptHandler(timer_callback);
    
    Position[1] = HallRead();
    Time[1] = 0;

    while (1) {
        Position[0] = Position[1];
        Time[0] = Time[1];

        Position[1] = HallRead();
        Time[1] = (uint16_t) (time);


        double Rotational_Velocity = (Position[1] - Position[0]) / (Time[1] - Time[0]);
        double Linear_Velocity = (2.0 * M_PI * Radius * Rotational_Velocity) / (Ticks * pow(10, 3)); //need help

        time = (float) timer_s + (((float) timer_ms) / 1000);

        if (check_adc_reading())
        {
            if (EUSART1_is_tx_ready()) {
                printf("Position = %f \n  ", Rotational_Velocity);
                
                if (Rotational_Velocity < 500){
                    spin_motor_ccw();
                    LED_2_SetHigh();
                    
                }
                else if ( Rotational_Velocity > 500){
                    spin_motor_cw();
                    LED_3_SetHigh();
                }
                
                
            }
        }
    }
}
