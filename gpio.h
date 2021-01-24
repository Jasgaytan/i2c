/*
 * port_config.h
 *
 *  Created on: 23 jul. 2020
 *      Author: Jesús
 */
//*****************************************************************************
//
// gpio.h - Driver para manejar los pines de uC.
//
//*****************************************************************************

#ifndef GPIO_H_
#define GPIO_H_
//*****************************************************************************
//                              Include
//*****************************************************************************
#include "driverlib.h"

//*****************************************************************************
//                              Variables Globales
//*****************************************************************************
extern uint8_t flag;
extern volatile unsigned int holdCount;

//*****************************************************************************
//                              Configuración de Pines
//*****************************************************************************
//*****************************************************************************
//! @name Configuración de pines:
//! \brief Definiciones de Pins dispuesta de manera tal que no es necesario
//!        modificar nada aca, solamente en la función que utiliza estos defines.
//! @{
//*****************************************************************************

//*****************************************************************************
//! \details Coloca un pin como salida.
//*****************************************************************************
#define GPIO_setPinAsOutput(baseAddress, selectedPin)               HWREG16(baseAddress + OFS_PADIR) |= selectedPin;

//*****************************************************************************
//! \details Coloca un pin de salida en alto.
//*****************************************************************************
#define GPIO_setHighOnPin(baseAddress, selectedPin)                 HWREG16(baseAddress + OFS_PADIR) |=  selectedPin;   \
                                                                    HWREG16(baseAddress + OFS_PAREN) &= ~selectedPin;   \
                                                                    HWREG16(baseAddress + OFS_PAOUT) |=  selectedPin;
//*****************************************************************************
//! \details Coloca un pin de salida en bajo.
//*****************************************************************************
#define GPIO_setLowOnPin(baseAddress, selectedPin)                  HWREG16(baseAddress + OFS_PADIR) |=  selectedPin;   \
                                                                    HWREG16(baseAddress + OFS_PAREN) &= ~selectedPin;   \
                                                                    HWREG16(baseAddress + OFS_PAOUT) &= ~selectedPin;
//*****************************************************************************
//! \details Coloca un pin como entrada.
//*****************************************************************************
#define GPIO_setPinAsInput(baseAddress, selectedPin)                HWREG16(baseAddress + OFS_PADIR) &= ~selectedPin;

//*****************************************************************************
//! \details Obtiene el estado de un pin de entrada.
//*****************************************************************************
#define GPIO_getInputPinValue(baseAddress, selectedPin)             HWREG16(baseAddress + OFS_PAIN) & (selectedPin)

//*****************************************************************************
//! \details Establece una resistencia de Pullup en un pin de entrada.
//*****************************************************************************
#define GPIO_setPinWithPullDownResistor(baseAddress, selectedPin)   HWREG16(baseAddress + OFS_PADIR) &= ~selectedPin;   \
                                                                    HWREG16(baseAddress + OFS_PAREN) |= selectedPin;    \
                                                                    HWREG16(baseAddress + OFS_PAOUT) &= ~selectedPin;
//*****************************************************************************
//! \details Establece una resistencia de Pulldown en un pin de entrada.
//*****************************************************************************
#define GPIO_setPinWithPullUpResistor(baseAddress, selectedPin)     HWREG16(baseAddress + OFS_PADIR) &= ~selectedPin;   \
                                                                    HWREG16(baseAddress + OFS_PAREN) |= selectedPin;    \
                                                                    HWREG16(baseAddress + OFS_PAOUT) |= selectedPin;
//*****************************************************************************
//! \details Establece la función primaria de un determinado pin.
//*****************************************************************************
#define GPIO_setPinPrimaryFunction(baseAddress, selectedPin)        HWREG16(baseAddress + OFS_PASEL0) |= selectedPin;  \
                                                                    HWREG16(baseAddress + OFS_PASEL1) &= ~selectedPin;
//*****************************************************************************
//! \details Establece la función secundaria de un determinado pin.
//*****************************************************************************
#define GPIO_setPinSecondFunction(baseAddress, selectedPin)         HWREG16(baseAddress + OFS_PASEL0) &= ~selectedPin;  \
                                                                    HWREG16(baseAddress + OFS_PASEL1) |= selectedPin;
//*****************************************************************************
//! \details Establece la función terciaria de un determinado pin.
//*****************************************************************************
#define GPIO_setPinTertiaryFunction(baseAddress, selectedPin)       HWREG16(baseAddress + OFS_PASEL0) |= selectedPin;  \
                                                                    HWREG16(baseAddress + OFS_PASEL1) |= selectedPin;

//*****************************************************************************
//! \details El pin de modo configuración es colocado como entrada.
//*****************************************************************************
#define GPIO_configurationMode                                      { P2DIR &= ~BIT7; P2REN &= ~BIT7;}

//*****************************************************************************
//! \details Estado del pin de modo configuración.
//*****************************************************************************
#define GPIO_configurationPin                                       P2IN & BIT7

//*****************************************************************************
//! @}
//*****************************************************************************

//*****************************************************************************
//                              Funciones Prototipos
//*****************************************************************************
//*****************************************************************************
//! \brief Configure los pines seleccionados por el usuario para realizar la
//!        distintas operaciónes que lo requieran.
//!
//! \details \b Descripción \n
//!          Función que recibe dos punteros como argumentos que se utilizarán
//!          para realizar la configuración de los pines según la elección del
//!          usuario. Para realizar la configuración se utiliza un \c switch(),
//!          y si se tratase de un puerto impar solamente se desplaza el pin
//!          hacia la izquierda de acuerdo a la elección del usuario y se pasa
//!          la dirección base del puerto que corresponda, por ejemplo, para
//!          los puertos 1 y 2 que corresponden al \b PA se le asigna la
//!          dirección base \b 0x0200. Si el puerto elegido es par, además de
//!          lo realizado para un puerto impar, se realiza un desplazamiento al
//!          registro parte alta que es el que corresponde para los puertos
//!          pares.
//!
//! \param *selectedPort Puntero que define el puerto seleccionado.
//! \param *selectedPin Puntero que define el pin elegido en el puerto deseado.
//!
//! \return \c void.
//*****************************************************************************
void GPIO_configPins(uint16_t*, uint16_t*);

//*****************************************************************************
//! \brief Función para encender un sensor.
//!
//! \details \b Descripción \n
//!          Realiza el encendido de un sensor a través de una llave electrónica
//!          controlada por un pin, el cual es indicado en uno de los
//!          parámetros recibidos, además del puerto correspondiente.
//!
//! \param vccPort Puerto de donde se obtiene el pin de alimentación.
//! \param vccPin Pin de alimentación.
//!
//! \return \c void.
//*****************************************************************************
void GPIO_powerOnSensor(const uint8_t vccPort, const uint8_t vccPin);

//*****************************************************************************
//! \brief Función para apagar un sensor.
//!
//! \details \b Descripción \n
//!          Realiza el apagado de un sensor a través de una llave electrónica
//!          controlada por un pin, el cual es indicado en uno de los
//!          parámetros recibidos, además del puerto correspondiente.
//!
//! \param vccPort Puerto de donde se obtiene el pin de alimentación.
//! \param vccPin Pin de alimentación.
//!
//! \return \c void.
//*****************************************************************************
void GPIO_powerOffSensor(const uint8_t vccPort, const uint8_t vccPin);

#endif /* GPIO_H_ */
