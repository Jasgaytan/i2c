/*
 * i2c.c
 *
 *  Created on: 30 oct. 2019
 *      Author: Matías López - Jesús López
 */
#include "i2c.h"
//**********************************************************************************************************************************************************
uint16_t countS;
uint8_t *myArray;
//**********************************************************************************************************************************************************
void I2C_initPort(uint16_t i2cAddress)
{
    // Configuración de los pines I2C (Port 5 Pin 2 -> SDA y Port 5 Pin 3 -> SCL)
    I2C_PORT_SEL |= I2C_PIN_SDA | I2C_PIN_SCL;              // I2C pins

    // Configure USCI_B0 for I2 mode - Sending
    UCB0CTLW0 |= UCSWRST;
    UCB0CTLW0 |= UCMST | UCMODE_3 | UCSYNC | UCSSEL__SMCLK; // I2C mode, master, sync, sending, SMCLK

    UCB0BRW = I2C_SCL_CLOCK_DIV;                            // SMCLK / 10 = 100 KHz; Bit clock prescaler. Modify only when UCSWRST = 1.

    UCB0I2CSA = i2cAddress;                                 // Dirección del esclavo.

    UCB0CTLW0 &= ~UCSWRST;                                  // clear reset register

    if (UCB0STAT & UCBBUSY)                                 // test if bus to be free otherwise a manual Clock on is
    {                                                       // generated
        I2C_PORT_SEL &= ~I2C_PIN_SCL;                       // Select Port function for SCL
        I2C_PORT_OUT &= ~I2C_PIN_SCL;
        I2C_PORT_DIR |=  I2C_PIN_SCL;                       // drive SCL low
        I2C_PORT_SEL |=  I2C_PIN_SDA + I2C_PIN_SCL;         // select module function for the used I2C pins
    };
}
//**********************************************************************************************************************************************************
static void M24LC512_initWrite(void)
{
  UCB0CTLW0 |= UCTR;                        // UCTR=1 => Transmit Mode (R/W bit = 0)
  UCB0IFG &= ~(UCTXIFG0 | UCSTPIFG);
  UCB0IE &= ~UCRXIE0;                       // disable Receive ready interrupt
  UCB0IE |= (UCTXIE0 | UCSTPIE);            // enable Transmit ready interrupt
}
//**********************************************************************************************************************************************************
static void M24LC512_initRead(void)
{
  UCB0CTLW0 &= ~UCTR;                       // UCTR=0 => Receive Mode (R/W bit = 1)
  UCB0IFG &= ~(UCRXIFG0 | UCSTPIFG);
  UCB0IE &= ~(UCTXIE0 | UCSTPIE);           // disable Transmit ready interrupt
  UCB0IE |= UCRXIE0;                        // enable Receive ready interrupt
}
//**********************************************************************************************************************************************************
void M24LC512_byteWrite(const uint16_t Address, const uint8_t Data)
{
    uint8_t adr_hi;
    uint8_t adr_lo;

    adr_hi = Address >> 8;                      // calculate high byte
    adr_lo = Address & 0xFF;                    // and low byte of address

    M24LC512_initWrite();

    UCB0CTLW0 |= UCTXSTT;                       // start condition generation
    __bis_SR_register(LPM3_bits + GIE);

    UCB0TXBUF = adr_hi;                         // Load TX buffer
    __bis_SR_register(LPM3_bits + GIE);

    UCB0TXBUF = adr_lo;                         // Load TX buffer
    __bis_SR_register(LPM3_bits + GIE);

    UCB0TXBUF = Data;                           // Load TX buffer
    __bis_SR_register(LPM3_bits + GIE);

    UCB0CTLW0 |= UCTXSTP;                       // I2C stop condition
    __bis_SR_register(LPM3_bits + GIE);

    UCB0IE &= ~(UCTXIE0 | UCSTPIE);             // disable Transmit ready interrupt
}
//**********************************************************************************************************************************************************
void M24LC512_pageWrite(uint16_t* StartAddress, uint8_t *Data, const uint16_t Size)
{
    volatile uint16_t i = 0;
    volatile uint16_t index = 0;
    volatile uint16_t counterI2cBuffer = 0;
    uint8_t adr_hi;
    uint8_t adr_lo;
    uint16_t currentAddress = *StartAddress;
    volatile uint16_t currentSize = Size;
    volatile uint16_t status = 0;
    uint8_t moreDataToRead = 1;
    uint16_t auxSize;
    uint32_t auxAddress = 128;
    uint32_t tempAddress;

    tempAddress = currentAddress;
    while(tempAddress > 128)
    {
        tempAddress = tempAddress - 128;
        auxAddress += 128;
    }
    tempAddress = auxAddress;

    // Execute until no more data in Data buffer
    while(moreDataToRead)
    {
        adr_hi = currentAddress >> 8;                               // calculate high byte
        adr_lo = currentAddress & 0x00FF;                           // and low byte of address

        // Chop data down to 64-byte packets to be transmitted at a time
        // Maintain pointer of current startaddress
        if(currentSize > M24LC512_MAXPAGEWRITE)
        {
            index = counterI2cBuffer;
            auxAddress = currentAddress;
            auxSize = currentSize;
            counterI2cBuffer = counterI2cBuffer + M24LC512_MAXPAGEWRITE;
            currentSize = currentSize - M24LC512_MAXPAGEWRITE;
            currentAddress = currentAddress + M24LC512_MAXPAGEWRITE;
        }
        else
        {
            moreDataToRead = 0;
            index = counterI2cBuffer;
            auxAddress = currentAddress;
            auxSize = currentSize;
            counterI2cBuffer = counterI2cBuffer + currentSize;
            currentAddress = currentAddress + currentSize;
        }
        status = __get_SR_register();
        if((status & C) || (status & (C | Z)))
        {
            countS++; // Contador de sobreescritura de la memoria.
            counterI2cBuffer = index + (0xFFFF - auxAddress) + 1;
            currentSize = auxAddress - (0xFFFF - auxSize) - 1;
            currentAddress = 0x0000;
            moreDataToRead = 1;
            goto jump;
        }
        if(currentAddress > tempAddress)
        {
            counterI2cBuffer = index + (tempAddress - auxAddress);
            currentSize = auxAddress - (tempAddress - auxSize);
            currentAddress = tempAddress;
            moreDataToRead = 1;
        }

        jump:
        if(tempAddress == 65536)
            tempAddress = 0;
        tempAddress += 128;

        M24LC512_initWrite();

        UCB0CTLW0 |= UCTXSTT;                                       // start condition generation => I2C communication is started
        __bis_SR_register(LPM3_bits + GIE);                         // Enter LPM0 w/ interrupts

        UCB0TXBUF = adr_hi;                                         // Load TX buffer
        __bis_SR_register(LPM3_bits + GIE);

        UCB0TXBUF = adr_lo;                                         // Load TX buffer
        __bis_SR_register(LPM3_bits + GIE);

        for(i = counterI2cBuffer ; i > index ; i--)
        {
            UCB0TXBUF = Data[(index + counterI2cBuffer) - i];       // Load TX buffer
            __bis_SR_register(LPM3_bits + GIE);
        }

        UCB0CTLW0 |= UCTXSTP;                                       // I2C stop condition
        __bis_SR_register(LPM3_bits + GIE);                         // Ensure stop condition got sent

        M24LC512_ackPolling();                                      // Ensure data is written in EEPROM
    }

    UCB0IE &= ~(UCTXIE0 | UCSTPIE);                                 // disable Transmit ready interrupt
    *StartAddress = currentAddress;
}
//**********************************************************************************************************************************************************
unsigned char M24LC512_currentRead(void)
{
    volatile uint8_t aux, temp;

    // Read Data byte
    M24LC512_initRead();

    UCB0CTLW0 |= UCTXSTP;

    UCB0CTLW0 |= UCTXSTT;                       // I2C start condition
    __bis_SR_register(LPM3_bits + GIE);         // Enter LPM0 w/ interrupts

    temp = UCB0RXBUF;

    UCB0IE |= UCSTPIE;
    __bis_SR_register(LPM3_bits + GIE);         // Enter LPM0 w/ interrupts

    UCB0IE &= ~(UCRXIE0 | UCSTPIE);

    return temp;
}
//**********************************************************************************************************************************************************
uint8_t M24LC512_randomRead(const uint16_t Address)
{
    uint8_t adr_hi;
    uint8_t adr_lo;
    volatile uint8_t temp = 0, aux = 0;

    adr_hi = Address >> 8;                      // calculate high byte
    adr_lo = Address & 0x00FF;                  // and low byte of address

    // Write Address first
    M24LC512_initWrite();

    UCB0CTLW0 |= UCTXSTT;                       // start condition generation
    __bis_SR_register(LPM3_bits + GIE);

    UCB0TXBUF = adr_hi;                         // Load TX buffer
    __bis_SR_register(LPM3_bits + GIE);

    UCB0TXBUF = adr_lo;    // Load TX buffer
    __bis_SR_register(LPM3_bits + GIE);

    UCB0CTLW0 |= UCTXSTP;                       // I2C stop condition
    __bis_SR_register(LPM3_bits + GIE);         // Enter LPM0 w/ interrupts

    // Read Data byte
    M24LC512_initRead();

    UCB0CTLW0 |= UCTXSTP;

    UCB0CTLW0 |= UCTXSTT;                       // I2C start condition
    __bis_SR_register(LPM3_bits + GIE);         // Enter LPM0 w/ interrupts

    temp = UCB0RXBUF;

    UCB0IE |= UCSTPIE;
    __bis_SR_register(LPM3_bits + GIE);         // Enter LPM0 w/ interrupts

    UCB0IE &= ~(UCRXIE0 | UCSTPIE);

    return temp;
}
//**********************************************************************************************************************************************************
void M24LC512_sequentialRead(uint16_t Address , uint8_t *Data , uint16_t Size)
{
    uint8_t adr_hi;
    uint8_t adr_lo;
    uint16_t counterSize;

    adr_hi = Address >> 8;                      // calculate high byte
    adr_lo = Address & 0x00FF;                  // and low byte of address

    // Write Address first
    M24LC512_initWrite();

    UCB0CTLW0 |= UCTXSTT;                       // start condition generation
    __bis_SR_register(LPM3_bits + GIE);         // => I2C communication is started
                                                // Enter LPM0 w/ interrupts
    UCB0TXBUF = adr_hi;                         // Load TX buffer
    __bis_SR_register(LPM3_bits + GIE);

    UCB0TXBUF = adr_lo;                         // Load TX buffer
    __bis_SR_register(LPM3_bits + GIE);

    UCB0CTLW0 |= UCTXSTP;                       // I2C stop condition
    __bis_SR_register(LPM3_bits + GIE);         // Enter LPM0 w/ interrupts

    // Read Data byte
    M24LC512_initRead();

    UCB0CTLW0 |= UCTXSTT;                       // I2C start condition

    for(counterSize = (Size-2) ; counterSize > 0  ; counterSize--)
    {
        __bis_SR_register(LPM3_bits + GIE);     // Enter LPM0 w/ interrupts
        Data[(Size-2) - counterSize] = UCB0RXBUF;
    }

    __bis_SR_register(LPM3_bits + GIE);         // Enter LPM0 w/ interrupts
    UCB0CTLW0 |= UCTXSTP;                       // I2C stop condition

    Data[Size-2] = UCB0RXBUF;
    __bis_SR_register(LPM3_bits + GIE);         // Enter LPM0 w/ interrupts

    Data[Size-1] = UCB0RXBUF;

    UCB0IE |= UCSTPIE;
    __bis_SR_register(LPM3_bits + GIE);         // Enter LPM0 w/ interrupts

    UCB0IE &= ~(UCRXIE0 | UCSTPIE);
}
//**********************************************************************************************************************************************************
void M24LC512_ackPolling(void)
{
    do
    {
        UCB0IE |= UCTXIE0 | UCSTPIE;
        UCB0IFG = 0x00;                         // clear I2C interrupt flags
        UCB0CTLW0 |= UCTR;                      // I2CTRX=1 => Transmit Mode (R/W bit = 0)
        UCB0CTLW0 &= ~UCTXSTT;
        UCB0CTLW0 |= UCTXSTT;                   // start condition is generated

        while(UCB0CTLW0 & UCTXSTT)                  // wait till I2CSTT bit was cleared
        {
            if(!(UCNACKIFG & UCB0IFG))          // Break out if ACK received
                break;
        }

        UCB0CTLW0 |= UCTXSTP;                   // stop condition is generated after. Wait till stop bit is reset
        __bis_SR_register(LPM3_bits + GIE);     // Enter LPM0 w/ interrupts

        UCB0IE &= ~(UCTXIE0 | UCSTPIE);

        __delay_cycles(1000);                   // delay

    } while(UCNACKIFG & UCB0IFG);

    UCB0IFG &= ~(UCTXIFG0 | UCSTPIFG);
}
//**********************************************************************************************************************************************************//**********************************************************************************************************************************************************
void M24LC512_setinitValueHeader(void)
{
    uint16_t i = 0x0000;

    // Habilita las escrituras en la memoria FRAM.
    SYSCFG0 &= ~DFWP;

    // Se coloca en cero el contador de sobreescritura, el flag de perdida de datos y la fecha y hora de la ultima medicion medicion.
    for(i = 21 ; i > 7 ; i--)
    {
        myArray[i] = 0x00;
    }

    // Deshabilita las escrituras en la memoria FRAM.
    SYSCFG0 |= DFWP;
}
//**********************************************************************************************************************************************************
void M24LC512_updateHeader(const uint16_t currentAddress, const uint16_t size)
{
    uint16_t address;
    uint16_t count;

    // Habilita las escrituras en la memoria FRAM.
    SYSCFG0 &= ~DFWP;

    if((myArray[15] != 2) && (myArray[15] != 3))
    {
        // Obtengo el puntero que tiene la direccion a partir de la cual se debe enviar los datos cuando se soliciten por comando - Ptro de la comunicación.
        address = ((((uint16_t)(myArray[7])) << 8) | myArray[8]);

        // Obtengo el contador de sobreescrituras.
        count = ((((uint16_t)(myArray[13])) << 8) | myArray[14]);

        // Actualizo la dirección de bytes escritos en memoria - currentAddress - Puntero de la memoria
        myArray[9] = ((uint8_t)(currentAddress >> 8));  // Parte alta
        myArray[10] = (uint8_t)currentAddress;          // Parte baja

        // Actualizo el contador de sobreescrituras cuando ocurra una sobreescritura en la memoria.
        if(countS > count)
        {
            // Actualizo la cantidad de sobreescrituras (aprox. cada 7 dias) - count
            myArray[13] = ((uint8_t)(countS >> 8));     // Parte alta
            myArray[14] = (uint8_t)countS;              // Parte baja
        }

        // Verifica que si se han perdido datos.
        if((address <= currentAddress) && (countS > count))
        {
            myArray[15] = 1;
        }
    }

    switch(myArray[15])
    {
        case 1:
            // En caso de que se pierdan datos se va actualizando la dirección desde donde se debe enviar lo datos por RF.
            myArray[7] = ((uint8_t)(currentAddress >> 8)); // Parte alta.
            myArray[8] = ((uint8_t)currentAddress);        // Parte baja.
            break;

        case 2:
            // Cuando hubo una transmision exitosa el puntero de la comunicacion se actualiza al puntero de la memoria para enviar nuevos datos.
            myArray[7] = myArray[9];
            myArray[8] = myArray[10];

            // Actualizo a cero el flag.
            myArray[15] = 0;           // Como ya se indico que los si los datos se perdieron o no se pone a cero para poder indicar nuevamente cuando ocurra una nueva perdidad de datos.
            break;

        case 3:
            // Actualizo la direccion de inicio donde se guardan los datos en la memoria - startAddress - Puntero de la comunicacion
            myArray[7] = 0x00;  // Parte alta
            myArray[8] = 0x00;  // Parte baja

            // Actualizo la dirección de bytes escritos en memoria al inicio - currentAddress - Puntero de la memoria
            myArray[9] = 0x00;  // Parte alta
            myArray[10] = 0x00; // Parte baja

            // Se pone a cero el contador de sobreescrituras - count
            myArray[13] = 0x00;  // Parte alta
            myArray[14] = 0x00;  // Parte baja

            // Se pone a cero el flag de perdidas de datos.
            myArray[15] = 0;
            break;
    }

    // Actualizo la cantidad de bytes escritos en memoria - size
    myArray[11] = ((uint8_t)(size >> 8));  // Parte alta
    myArray[12] = ((uint8_t)size);         // Parte baja

    // Deshabilita las escrituras en la memoria FRAM.
    SYSCFG0 |= DFWP;
}
//**********************************************************************************************************************************************************
bool M24LC512_memoryCheck(void)
{
    uint8_t contDo = 3;
    static uint8_t temp = 0;
    uint8_t contWhile = 3;

    do
    {
        UCB0IE |= UCTXIE0 | UCSTPIE | UCSTTIE;
        UCB0IFG = 0x00;                        // clear I2C interrupt flags
        UCB0CTLW0 |= UCTR;                     // I2CTRX=1 => Transmit Mode (R/W bit = 0)
        UCB0CTLW0 &= ~UCTXSTT;
        UCB0CTLW0 |= UCTXSTT;                  // start condition is generated

        while((UCTXSTT & UCB0CTLW0))           // wait till I2CSTT bit was cleared
        {
            if((!(UCNACKIFG & UCB0IFG)) || (contWhile > 0))           // Break out if ACK received
                break;
            contWhile--;
        }

        --contDo;
        contWhile = 3;

        __delay_cycles(1000);

        if(contDo == 0) break;

    }while(UCNACKIFG & UCB0IFG);

    if(contDo != 0 && !(UCNACKIFG & UCB0IFG) && !(UCTXSTT & UCB0CTLW0) && (UCTXIFG0 & UCB0IFG))
    {
        // Es necesario hacer una transmision completa para verificar la conexion si no no envia el stop lo que ocasiona problemas en posteriores accesos de memoria.
        __bis_SR_register(LPM3_bits + GIE);         // for start

        UCB0TXBUF = 0x00;                           // Load TX buffer
        __bis_SR_register(LPM3_bits + GIE);

        UCB0TXBUF = 0x00;                           // Load TX buffer
        __bis_SR_register(LPM3_bits + GIE);

        UCB0CTLW0 |= UCTXSTP;                       // I2C stop condition
        __bis_SR_register(LPM3_bits + GIE);         // Enter LPM0 w/ interrupts

        UCB0IE &= ~(UCTXIE0 | UCSTPIE | UCSTTIE);
        UCB0IFG &= ~(UCTXIFG0 | UCSTTIFG | UCSTPIFG);

        if(myArray[15] == 4)    // Verifica si anteriormente no habia estado repondiendo.
        {
            SYSCFG0 &= ~DFWP;   // Habilita las escrituras en la memoria FRAM.
            myArray[15] = temp; // En caso de que, anteriormente, no hubiese respondido y luego al intentarlo de nuevo (o reiniciarlo) vuelve a responder y retome el valor que tenia.
            SYSCFG0 |= DFWP;    // Deshabilita las escrituras en la memoria FRAM.
        }else
        {
            temp = myArray[15]; // Se va almacenando el ultimo valor para luego si deja de responder y vuelve a responder retoma el valor de antes.
        }

        return true;

    }else if((contDo == 0 && (UCNACKIFG & UCB0IFG) && !(UCTXSTT & UCB0CTLW0) && (UCTXIFG0 & UCB0IFG)) ||
             (contDo != 0 && !(UCNACKIFG & UCB0IFG) && (UCTXSTT & UCB0CTLW0) && !(UCTXIFG0 & UCB0IFG)))
    {
        UCB0IE &= ~(UCTXIE0 | UCSTPIE | UCSTTIE);
        UCB0IFG &= ~(UCTXIFG0 | UCSTTIFG | UCSTPIFG);
        UCB0CTLW0 &= ~UCTXSTT;

        SYSCFG0 &= ~DFWP;
        myArray[15] = 4;
        SYSCFG0 |= DFWP;

        return false;
    }
    return 0;
}
//***************************************************************************************************************
float MLX90614_getTemp(uint8_t command, uint8_t *temp)
{
    float aux = 0.0;
    //uint8_t g_mlxValBytes[3];                 // Recieved value byte storage

    // Send object temperature read command
    M24LC512_initWrite();                       // Change to transmitter.

    UCB0CTLW0 |= UCTXSTT;
    __bis_SR_register(LPM3_bits + GIE);

    UCB0TXBUF = command;                        // Send temperature command
    __bis_SR_register(LPM3_bits + GIE);

    // Receive Bytes
    M24LC512_initRead();                        // Change to receive

    UCB0CTLW0 |= UCTXSTT;                       // Send restart
    __bis_SR_register(LPM3_bits + GIE);         // Wait for restart

    temp[0] = UCB0RXBUF;
    __bis_SR_register(LPM3_bits + GIE);         // Wait for RX interrupt flag

    UCB0CTLW0 |= UCTXSTP;

    temp[1] = UCB0RXBUF;                        // 1st byte.
    __bis_SR_register(LPM3_bits + GIE);

    temp[2] = UCB0RXBUF;                        // 2nd byte.

    UCB0IE |= UCSTPIE;
    __bis_SR_register(LPM3_bits + GIE);         // Enter LPM0 w/ interrupts

    UCB0IE &= ~(UCRXIE0 | UCSTPIE);

    //calculate Temperature
    uint16_t tempVals = ( ((uint16_t) temp[1]) << 8 ) | ( (uint16_t) temp[0] );
    aux = ((float) tempVals) * 0.02 - 273.15;

    return (aux);
}
//***************************************************************************************************************
void MLX90614_sleepMode(void)
{
    UCB0CTLW0 |= UCTR;

    UCB0CTLW0 |= UCTXSTT;
    __bis_SR_register(LPM3_bits + GIE);

    // Send object temperature sleep mode command
    UCB0TXBUF = MLX90614_SLEEP;
    __bis_SR_register(LPM3_bits + GIE);

    I2C_SCL_LO
}
//***************************************************************************************************************
void MLX90614_exitSleepMode(void)
{
    //SCL pin high and then PWM/SDA pin low for at least tDDQ > 33ms
    I2C_SCL_HI
    I2C_SDA_LO

    delay_ms(40);

    UCB0CTLW0 |= UCTXSTP;
    while(UCB0CTLW0 & UCTXSTP);
}
//***************************************************************************************************************
void MLX90614_showTemp(float g_Temp)
{
    //Show object temperature
    volatile uint16_t aux;
    showChar('T',pos1);
    aux=((int)g_Temp)/10;
    showChar(aux+48,pos2);
    aux=((int)g_Temp)%10;
    showChar(aux+48,pos3);

    // Decimal point
    LCDMEM[pos3+1] |= 0x01;
    volatile float mantisa = g_Temp - (uint16_t)g_Temp;
    volatile uint16_t dosDecimales = mantisa * 100;
    aux=((int)dosDecimales)/10;
    showChar(aux+48,pos4);
    aux=((int)dosDecimales)%10;
    showChar(aux+48,pos5);

    // Degree symbol
    LCDMEM[pos5+1] |= 0x04;
    showChar('C',pos6);
}
//***************************************************************************************************************
void SHT3_getData(uint8_t* buffer){

    uint8_t i;

    // Send object temperature read command
    M24LC512_initWrite();                   // and low byte of address               // Change to transmitter.

    UCB0CTLW0 |= UCTXSTT;
    __bis_SR_register(LPM3_bits + GIE);

    UCB0TXBUF = SHT3_MEASUREMENT_MSB;       // Send temperature command
    __bis_SR_register(LPM3_bits + GIE);

    UCB0TXBUF = SHT3_MEASUREMENT_LSB;       // Send temperature command
    __bis_SR_register(LPM3_bits + GIE);

    // Receive Bytes
    M24LC512_initRead();                    // Change to receive

    UCB0CTLW0 |= UCTXSTT;                   // Send restart

    // Receive Bytes
    for(i = 3; i > 0 ; i--)
    {
        __bis_SR_register(LPM3_bits + GIE);
        buffer[3 - i] = UCB0RXBUF;
    }

    __bis_SR_register(LPM3_bits + GIE);         // Enter LPM0 w/ interrupts
    UCB0CTLW0 |= UCTXSTP;                       // I2C stop condition

    buffer[4] = UCB0RXBUF;
    __bis_SR_register(LPM3_bits + GIE);         // Enter LPM0 w/ interrupts

    buffer[5] = UCB0RXBUF;

    UCB0IE |= UCSTPIE;
    __bis_SR_register(LPM3_bits + GIE);

    UCB0IE &= ~(UCRXIE0 | UCSTPIE);
}
//***************************************************************************************************************
uint16_t SHT3_calculateTemp(uint8_t* buffer)
{
    uint16_t temp = ((((uint16_t) buffer[0]) << 8 ) | ( (uint16_t) buffer[1]));
    uint32_t stemp = temp;
    stemp *= 175;
    stemp /= 65535;
    stemp = -45 + stemp;
    //temp = (uint16_t) stemp;

    return ((uint16_t) stemp);
}
//***************************************************************************************************************
uint16_t SHT3_calculateHum(uint8_t* buffer)
{
    uint16_t hum = ((((uint16_t) buffer[3]) << 8 ) | ( (uint16_t) buffer[4]));
    uint32_t shum = hum;
    shum *= 100;
    shum /= 65535;
    //hum = (uint16_t) shum;

    return ((uint16_t) shum);
}
//***************************************************************************************************************
void SHT3_showTempHum(uint16_t g_Temp,uint16_t g_Hum)
{
    uint16_t aux;
    showChar('T',pos1);
    aux=(g_Temp)/10;
    showChar(aux+48,pos2);
    aux=(g_Temp)%10;
    showChar(aux+48,pos3);

    showChar('H',pos4);
    aux=(g_Hum)/10;
    showChar(aux+48,pos5);
    aux=(g_Hum)%10;
    showChar(aux+48,pos6);
}
//********************************************************************************************************************************************************************
// I2C interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_B0_VECTOR
__interrupt void USCIB0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B0_VECTOR))) USCIB0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG))
  {
    case USCI_NONE:          break;         // Vector 0: No interrupts
    case USCI_I2C_UCALIFG:   break;         // Vector 2: ALIFG
    case USCI_I2C_UCNACKIFG: break;         // Vector 4: NACKIFG
    case USCI_I2C_UCSTTIFG:                 // Vector 6: STTIFG
    case USCI_I2C_UCSTPIFG:                 // Vector 8: STPIFG

        __bic_SR_register_on_exit(LPM3_bits + GIE);
        break;

    case USCI_I2C_UCRXIFG3:  break;         // Vector 10: RXIFG3
    case USCI_I2C_UCTXIFG3:  break;         // Vector 14: TXIFG3
    case USCI_I2C_UCRXIFG2:  break;         // Vector 16: RXIFG2
    case USCI_I2C_UCTXIFG2:  break;         // Vector 18: TXIFG2
    case USCI_I2C_UCRXIFG1:  break;         // Vector 20: RXIFG1
    case USCI_I2C_UCTXIFG1:  break;         // Vector 22: TXIFG1
    case USCI_I2C_UCRXIFG0:                 // Vector 24: RXIFG0
    case USCI_I2C_UCTXIFG0:                 // Vector 26: TXIFG0

        __bic_SR_register_on_exit(LPM3_bits + GIE);
        break;

    case USCI_I2C_UCBCNTIFG: break;         // Vector 28: BCNTIFG
    case USCI_I2C_UCCLTOIFG: break;         // Vector 30: clock low timeout
    case USCI_I2C_UCBIT9IFG: break;         // Vector 32: 9th bit
    default: break;
  }
}
