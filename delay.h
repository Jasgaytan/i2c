/**
  * @file     delay.h
  * @brief    Driver para delays tiempos.
  * @date     Created on: 30 oct. 2019
  * @authors  Matías López - Jesús López
  * @version  1.0
  */
//*****************************************************************************
//
// delay.h - Driver para delays de tiempos.
//
//*****************************************************************************

#ifndef DELAY_H_
#define DELAY_H_

//*****************************************************************************
//                              Include
//*****************************************************************************
#include "driverlib.h"

//*****************************************************************************
//                              Definiciones
//*****************************************************************************
//*****************************************************************************
//! @name Configuración delays:
//! \brief Defines para configurar la velocidad de la CPU. Esta forma es mas
//!        rapida y precisa que las funciones de mas abajo ya que estas son
//!        macros y las funciones deben configurar el timer.
//! @{
//*****************************************************************************
//*****************************************************************************
//! \details Depende de la velocidad que tenga la CPU.
//*****************************************************************************
#define CYCLES_PER_US 8L // depends on the CPU speed

//*****************************************************************************
//! \details Depende del valor asignado a CYCLES_PER US multiplicado por 1000
//!          para que este en terminos de milisegundos.
//*****************************************************************************
#define CYCLES_PER_MS (CYCLES_PER_US * 1000L)

//*****************************************************************************
//! \details Mediante esta constante se ingresa el valor del delay necesario
//!          en x y se obtiene un delay de tiempo en terminos de micro segundos.
//*****************************************************************************
#define DELAY_US(x) __delay_cycles((x * CYCLES_PER_US))

//*****************************************************************************
//! \details Mediante esta constante se ingresa el valor del delay necesario
//!          en x y se obtiene un delay de tiempo en terminos de mili segundos.
//*****************************************************************************
#define DELAY_MS(x) __delay_cycles((x * CYCLES_PER_MS))

//*****************************************************************************
//! @}
//*****************************************************************************

//*****************************************************************************
//                              Functiones Prototipos
//*****************************************************************************
//*****************************************************************************
//! \brief Retardo de tiempo en microsegundos.
//!
//! \details \b Descripción \n
//!          Primero se setea el bit \b TACLR en el registro \b TA0CTL que
//!          resetea el \b TAR. Luego se carga el \b TA0CCR0 con el valor
//!          hasta donde se desea que cuente. Posteriormemte se habilitan las
//!          interrupciones del Capture/Compare 0, luego se divide la señal
//!          de reloj por 4 y por último se configura el <b>Timer A</b>
//!          seleccionando como fuente de reloj \b SMCLK dividiendola por
//!          2 mediante los bits \b ID_1 y en modo up que contara hasta
//!          llegar al valor seteado en elregistro \b TA0CCR0. Por último,
//!          entra al modo bajo consumo \b LMP3.
//!
//! \note Se debe configurar el timer de este modo para evitar un comportamiento
//!       impredecible (Esto se indica en el datasheet del dispositivo).
//!       Tambien puede observarse que, previo a la configuración del timer, se
//!       deshabilita la interrupción que es utilizada por el RF para que
//!       no interrumpa y permita que finalice el conteo del timer.
//!
//! \param us Valor de tiempo en microsegundos que es requerido por el usuario.
//!
//! \return \c void.
//!
//! \attention Modifica los bits de los registros \b TA1CTL, \b TA1CCTL0,
//!            \b TA0CCR0 y \SR.
//*****************************************************************************
void delay_us(const uint16_t);

//*****************************************************************************
//! \brief Retardo de tiempo en milisegundos.
//!
//! \details \b Descripción \n
//!          Primero se setea el bit \b TACLR en el registro \b TA0CTL que
//!          resetea el \b TAR. Luego se carga el \b TA0CCR0 con el valor
//!          hasta donde se desea que cuente. Posteriormemte se habilitan las
//!          interrupciones del Capture/Compare 0, luego se divide la señal
//!          de reloj por 4 y por último se configura el <b>Timer A</b>
//!          seleccionando como fuente de reloj \b ACLK dividiendola por
//!          8 mediante los bits \b ID_3 y en modo up que contara hasta
//!          llegar al valor seteado en elregistro \b TA0CCR0. Por último,
//!          entra al modo bajo consumo \b LMP3.
//!
//! \note Se debe configurar el timer de este modo para evitar un comportamiento
//!       impredecible (Esto se indica en el datasheet del dispositivo).
//!       Tambien puede observarse que, previo a la configuración del timer, se
//!       deshabilita la interrupción que es utilizada por el RF para que
//!       no interrumpa y permita que finalice el conteo del timer.
//!
//! \param us Valor de tiempo en microsegundos que es requerido por el usuario.
//!
//! \return \c void.
//!
//! \attention Modifica los bits de los registros \b TA1CTL, \b TA1CCTL0,
//!            \b TA0CCR0 y \SR.
//*****************************************************************************
void delay_ms(const uint16_t);

#endif /* DELAY_H_ */
