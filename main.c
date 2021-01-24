#include "i2c.h"

int main(void)
{
    // Local Variables -----------------------------------------------------------------------------------------------
    uint8_t aux[6], j = 0, txData[10], rxData[10];
    volatile uint16_t shtTemp = 0, shtHum = 0;
    uint16_t address = 0x0000, addr = 0x0000;
    float mlxTemp = 0.0;
    flag = 0;

    // Stop WATCHDOG --------------------------------------------------------------------------------------------------
    WDTCTL = WDTPW | WDTHOLD;

    // Configure LCD --------------------------------------------------------------------------------------------------
    Init_LCD();

    // Buttons
    // Configure button S1 (P1.2) interrupt
    P1IES &= ~BIT2;
    P1DIR &= ~BIT2;
    P1REN |=  BIT2;
    P1OUT |=  BIT2;
    P1IFG &= ~BIT2;
    P1IE  |=  BIT2;

    // Configure button S2 (P2.6) interrupt
    P2IES &= ~BIT6;
    P2DIR &= ~BIT6;
    P2REN |=  BIT6;
    P2OUT |=  BIT6;
    P2IFG &= ~BIT6;
    P2IE  |=  BIT6;

    // Desabilita el modo de alta impedancia habilitando la configuración establecida previamente.
    PM5CTL0 &= ~LOCKLPM5;

    __bis_SR_register(GIE);

    while(1)
    {
        if(flag == 1)
        {
            // MLX90614 --------------------------------------------------------------------------------------------------------
            I2C_initPort(MLX90614_I2C_ADDRESS);             // Inicializa el puerto.
            mlxTemp = MLX90614_getTemp(MLX90614_TOBJ1, aux);// Obtiene la temperatura objeto.
            txData[j++] = aux[1] >> 8;                      // Byte alto
            txData[j++] = aux[0];                           // Byte bajo.
            MLX90614_showTemp(mlxTemp);                     // Muestra el valor de la temperatura.
        }
        if(flag == 2)
        {
            // SHT3 -------------------------------------------------------------------------------------------------------------
            I2C_initPort(SHT3_I2C_ADDRESS);                 // Inicializa el puerto.
            SHT3_getData(aux);                              // Obtiene los valores en crudo de la temperatura y humedad.
            txData[j++] = aux[0];                           // Temperatura byte alto.
            txData[j++] = aux[1];                           // Temperatura byte bajo.
            txData[j++] = aux[3];                           // Humedad byte alto.
            txData[j++] = aux[4];                           // Humedad byte bajo.
            shtTemp = SHT3_calculateTemp(aux);              // Muestra el valor de la temperatura.
            shtHum  = SHT3_calculateHum(aux);               // Muestra el valor de la humedad.
            SHT3_showTempHum(shtTemp, shtHum);
        }
        if(flag == 3)
        {
            // 24LC512 ----------------------------------------------------------------------------------------------------------
            if(M24LC512_memoryCheck())
            {
                I2C_initPort(M24LC512_I2C_ADDRESS);             // Inicializa el puerto.
                addr = address;
                M24LC512_pageWrite(&address, txData, j);        // Almacena los valores en crudo en la memoria.
                M24LC512_sequentialRead(addr, rxData, j);       // Realiza una lectura de los valores almacenados en la memoria.
                j = 0;
                clearLCD();
                displayScrollText("SAVED IT ON MEMORY");
                __bis_SR_register(GIE);
                flag = 0;
            }else
            {
                j = 0;
                flag = 0;
                clearLCD();
                displayScrollText("MEMORY FAILURE");
                __bis_SR_register(GIE);
            }
        }

    }
}

