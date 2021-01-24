/**
  * @file     i2c.h
  * @brief    Driver para realizar una comunicacion I2C.
  * @date     Created on: 30 oct. 2019
  * @authors  Matías López - Jesús López
  * @version  1.0
  */
//*****************************************************************************
//
// i2c.h - Driver para realizar una comunicacion I2C.
//
//*****************************************************************************

#ifndef I2C_H_
#define I2C_H_

//*****************************************************************************
//                                      Includes
//*****************************************************************************
#include "driverlib.h"
#include "hal_LCD.h"
#include "gpio.h"
#include "delay.h"

//*****************************************************************************
//                                      Defines
//*****************************************************************************
//*****************************************************************************
//! @name Cabecera en Memoria FRAM:
//! \details Dirección de la FRAM donde se van a guardan los datos de
//!          configuración.
//! @{
//*****************************************************************************
#define FRAM_START 0x1800

//*****************************************************************************
//! \details Puntero que indica la configuracion utilizada, es decir, los
//!          sensores que se estan utilizando.
//*****************************************************************************
extern uint8_t *myArray;

//*****************************************************************************
//! \details Contador que indica que hubo una sobreescritura en la memoria,
//!          que se transmitira, mediante el comando EM, para permitir realizar
//!          un control de los datos.
//*****************************************************************************
extern uint16_t countS;

//*****************************************************************************
//! @}
//*****************************************************************************

//*****************************************************************************
//                                      Comandos
//*****************************************************************************
//*****************************************************************************
//! @name Comandos:
// \brief Comandos que utilizan los diferentes dispositivos I2C.
// @{
//*****************************************************************************

//*****************************************************************************
//! @name SHT3:
//! \brief Comandos del sensor de sht3.
//! @{
//*****************************************************************************
#define SHT3_I2C_ADDRESS 0x44
#define SHT3_MEASUREMENT_MSB 0x24
#define SHT3_MEASUREMENT_LSB 0x16
//*****************************************************************************
//! @}
//*****************************************************************************

//*****************************************************************************
//! @name MLX90614:
//! \brief Comandos del sensor de mlx90614.
//! @{
//*****************************************************************************
#define MLX90614_I2C_ADDRESS 0x5A
// RAM
#define MLX90614_RAWIR1 0x04
#define MLX90614_RAWIR2 0x05
#define MLX90614_TA 0x06
#define MLX90614_TOBJ1 0x07
#define MLX90614_TOBJ2 0x08
// EEPROM - Section 8.4.5 of data sheet explains why 0x20 is added to these addresses
#define MLX90614_TOMAX 0x20
#define MLX90614_TOMIN 0x21
#define MLX90614_PWMCTRL 0x22
#define MLX90614_TARANGE 0x23
#define MLX90614_EMISS 0x24
#define MLX90614_CONFIG 0x25
#define MLX90614_ADDR 0x0E
#define MLX90614_ID1 0x3C
#define MLX90614_ID2 0x3D
#define MLX90614_ID3 0x3E
#define MLX90614_ID4 0x3F
// Command to into Sleep Mode
#define MLX90614_SLEEP 0xFF     //Enter SLEEP mode
//*****************************************************************************
//! @}
//*****************************************************************************

//*****************************************************************************
//! @name Memoria 24LC512:
//! \brief Comandos a utilizar con la memoria I2C 24LC512.
//! @{
//*****************************************************************************
//*****************************************************************************
//! \details Comando que indica la dirección de la memoria I2C que el \b MCU
//!          deberá enviar por el pin \b SDA para poder comunicarse.
//*****************************************************************************
#define M24LC512_I2C_ADDRESS    0x50

//*****************************************************************************
//! \details Máxima cantidad de bytes que se pueden escribir por página.
//*****************************************************************************
#define M24LC512_MAXPAGEWRITE   128

//*****************************************************************************
//! \details Dirección de inicio de escritura en la memoria externa.
//*****************************************************************************
#define M24LC512_STARTADDRESS   0x0000

//*****************************************************************************
//! @}
//*****************************************************************************

//*****************************************************************************
//! @}
//*****************************************************************************

//*****************************************************************************
//                                      I2C
//*****************************************************************************
//*****************************************************************************
//! @name Configuración I2C:
//! \brief Defines para configurar el Puerto I2C.
//! @{
//*****************************************************************************
#define I2C_PORT_SEL      P5SEL0
#define I2C_PORT_OUT      P5OUT
#define I2C_PORT_REN      P5REN
#define I2C_PORT_DIR      P5DIR
#define I2C_PIN_SDA       BIT2         // UCB0SDA pin
#define I2C_PIN_SCL       BIT3         // UCB0SCL pin
#define I2C_SCL_CLOCK_DIV 0x0A         // SCL clock divider
// To exit from Sleep Mode (Wake up request)
#define I2C_SDA_LO { I2C_PORT_DIR |= I2C_PIN_SDA; I2C_PORT_OUT &= ~I2C_PIN_SDA; }
#define I2C_SDA_HI { I2C_PORT_DIR |= I2C_PIN_SDA; I2C_PORT_OUT |=  I2C_PIN_SDA; }
#define I2C_SCL_HI { I2C_PORT_DIR |= I2C_PIN_SCL; I2C_PORT_OUT |=  I2C_PIN_SCL; }
#define I2C_SCL_LO { I2C_PORT_DIR |= I2C_PIN_SCL; I2C_PORT_OUT &= ~I2C_PIN_SCL; }
//*****************************************************************************
//! @}
//*****************************************************************************

//*****************************************************************************
//                              Funciones prototipos
//*****************************************************************************
//*****************************************************************************
//! \brief Configura e inicializa la comunicación I2C.
//!
//! \details \b Descripción \n
//!          Función que se encarga de configurar la comunicación I2C. Primero
//!          en el registro \b P5SEL0 se selecciona la función primaria del
//!          módulo seteando un 1 en los pines \b P5.2 y \b P5.3
//!          correspondientes a los pines SDA y SCL respectivamente, como se
//!          indica la tabla <em> "Table 6-20. Port P5.0, P5.1, P5.2, and P5.3
//!          Pin Functions" </em> en la sección <em>"6.9.13 Input/Output
//!          Schematics"</em> del datasheet <em>"MSP430FR413x mixed-signal
//!          microcontrollers"</em>. Luego para configurar la comunicación se
//!          setea el bit \b UCSWRST y así evitar comportamientos impredecibles.
//!          Ahora se configura la comunicación utilizando el registro
//!          \b UCB0CTLW0 en donde se configura en el modo I2C, en el modo
//!          maestro, se setea el bit \b UCSYNC ya que el fabricante indica que
//!          en el eUSI_B se debe poner en 1. También se configura como
//!          transmisor y se selecciona como fuente de reloj el \b SMCLK y
//!          luego se coloca un divisor igual a 0xA en el registro \b UCB0BRW
//!          que es el divisor de la fuente de reloj que hará que la frecuencia
//!          sea de 100KHz. Posteriormente en el registro \b UCB0I2CSA se le
//!          asigna la dirección del esclavo con el que se quiere comunicar.
//!          Mediante un \c if() se verifica si el bus está ocupado, si resulta
//!          verdadero desactiva la señal de reloj y la genera manualmente.
//!
//! /param I2C_ADDRESS Direccion del dispositivo con el que se quiere
//!                    establecer una comunicacion
//!
//! \return \c void.
//!
//! \attention Modifica los bits de los registros \b P5SEL, \b UCB0CTLW0,
//!            \b UCB0BRW, \b UCB0I2CSA, \b P5DIR, \b P5OUT, \b UCB0IE y
//!            \b UCB0IFG.
//*****************************************************************************
void I2C_initPort(uint16_t i2cAddress);

//*****************************************************************************
//! \brief Configura el MCU para realizar una escritura.
//!
//! \details \b Descripción \n
//!          Función que se encarga de configurar el \b MCU en el modo
//!          transmisión para realizar una escritura en el dispositivo
//!          direccionado, luego se limpia la bandera \b UCTXIFG0 por si
//!          existiese alguna interrupción pendiente, se deshabilita la
//!          interrupción \b UCRXIE0 de recepción pendiente y por último se
//!          habilitan las interrupciones \b UCTXIE0, \b UCSTTIE y \b UCSTPIE
//!          utilizadas en la trasmisión de datos.
//!
//! \return \c void.
//!
//! \attention Modifica los bits de los registros \b UCB0CTLW0, \b UCB0IFG y
//!            \b UCB0IE.
//*****************************************************************************
static void M24LC512_initWrite(void);

//*****************************************************************************
//! \brief Configura el MCU para realizar una lectura.
//!
//! \details \b Descripción \n
//!          Función que se encarga de configurar el \b MCU en el modo
//!          recepción para realizar una lectura del dispositivo
//!          direccionado, luego se limpia la bandera \b UCRXIFG0 por si
//!          existiese alguna interrupción pendiente, se deshabilita la
//!          interrupción \b UCTXIE0 de una transmisión pendiente, el bit
//!          \b UCSTTIE que genera una condición de start y el bit \b UCSTPIE
//!          que genera una condición de stop, y por último se habilitan las
//!          interrupción \b UCRXIE0, utilizada en la recepción de datos.
//!
//! \return \c void.
//!
//! \attention Modifica los bits de los registros \b UCB0CTLW0, \b UCB0IFG y
//!            \b UCB0IE.
//*****************************************************************************
static void M24LC512_initRead(void);

//*****************************************************************************
//! \brief Realiza la escritura de un solo byte en la memoria EEPROM.
//!
//! \details \b Descripción \n
//!          Esta función se encarga de escribir en una dirección de memoria un
//!          byte de datos. Para llevar a cabo esto primero se separa
//!          la dirección en bytes de parte alta y parte baja ya que es una
//!          dirección compuesta por 2 bytes. Después se coloca como transmisor
//!          para luego empezar la comunicación enviando un start, que trasmite,
//!          en conjunto, la dirección del dispositivo con el que se esta
//!          estableciendo la comunicación y, posteriormente, se transmite la
//!          dirección de memoria que se esta intentando acceder para escribir
//!          en la misma, empezando por la parte alta y luego la parte baja.
//!          Una vez establecida la comunicación con el esclavo se envía el
//!          dato que se desea escribir en la memoria. Por último, se
//!          transmite una condición de Stop para finalizar la comunicación.
//!          Se puede notar que una vez se realiza una transmisión de alguna
//!          parte de la trama se entra a un modo bajo consumo y se habilitan
//!          las interrupciones generales, de modo que cuando se active alguna
//!          interrupción de pedido de atención vaya a parar a esa interrupcion,
//!          lo que indica que ya se transmitió el byte en cuestión. Entonces
//!          en la interrupción se sale del bajo consumo, se deshabilitan las
//!          interrupciones globales para luego continuar con la normal
//!          ejecución del programa.
//!
//! \param address Dirección del dispositivo con el que el micro quiere
//!        establecer una comunicación.
//! \param data Información que el usuario desea escribir en la memoria.
//!
//! \return \c void.
//!
//! \attention Modifica los bits de los registros \b UCB0CTLW0, \b UCB0IFG,
//!            \b UCB0TXBUF, \b SR y \b UCB0IE.
//*****************************************************************************
void M24LC512_byteWrite(const uint16_t Address , const uint8_t Data);

//*****************************************************************************
//! \brief Función que realiza una escritura en la memoria por página,
//!        compuestas por 128 bytes.
//!
//! \details \b Descripción \n
//!          Función que se encarga de escribir una cantidad x de datos
//!          indicados por el usuario y con una cantidad máxima de 128 bytes
//!          por página.  Si el maestro debe transmitir más de 128 bytes antes
//!          de generar una condición de stop, el puntero de direcciones interno
//!          de la memoria, que apunta a una dirección de memoria a la que se
//!          encuentra actualmente accediendo, comenzará de nuevo al superarse
//!          la cantidad de bytes por página y los datos recibidos anteriormente
//!          se sobrescribirán, por tal motivo se utiliza un \c while() en
//!          conjunto con \c if-else para diferenciar si se quiere escribir una
//!          página completa de la memoria o una cantidad menor y así evitar
//!          perdida de datos.
//!          En conjunto con la condición de start del maestro, se transmite
//!          el código de control (cuatro bits), la Selección de Chip (tres
//!          bits) y el bit R/W en "0" en el bus (que indica una escritura).
//!          Esto indica al receptor esclavo direccionado que el byte alto de
//!          la dirección seguirá después de que haya generado un \b ACK
//!          durante el noveno ciclo de reloj. Por lo tanto, el siguiente byte
//!          transmitido por el maestro es el byte de orden superior de la
//!          palabra dirección y se escribirá en la dirección puntero de la
//!          \b 24LC512. El siguiente byte es el byte de dirección menos
//!          significativo. Después de recibir otra señal de reconocimiento
//!          \b ACK del \b 24LC512, el dispositivo maestro transmitirá la
//!          palabra de datos que se escribirá en la ubicación de memoria
//!          direccionada. El \b 24LC512 reconoce nuevamente y el maestro
//!          transmite hasta 127 bytes adicionales, que se almacenan
//!          temporalmente en el búfer de página en el chip y se escribirán en
//!          la memoria después de que el maestro haya transmitido una condición
//!          de stop. Después de recibir cada palabra, los siete bits
//!          inferiores del puntero de dirección de la memoria se incrementan
//!          internamente en uno.
//!          Luego de ese ciclo de escritura se realiza un \a M24LC512_ackPolling()
//!          para esperar a que la memoria esté disponible, ya que mientras esta
//!          escribiendo los datos indicados por el usuario, esta no responderá a los
//!          pedidos del \b MCU.
//!          Al igual que con la operación de escritura de bytes, una vez que
//!          se recibe la condición Stop, comenzará un ciclo de escritura
//!          interno. Si se intenta escribir en la matriz con el pin WP en
//!          alto, el dispositivo reconocerá el comando, pero no escribirá,
//!          ocurrirá el ciclo, no se escribirán datos y el dispositivo
//!          aceptará inmediatamente un nuevo comando.
//!
//! \param Address Dirección de la memoria desde donde el micro escribirá los
//!        bytes de datos.
//! \param *Data Puntero donde se almacenarán los datos, que se quieren escribir
//!        en la memoria.
//! \param Size Cantidad de datos a escribir en la memoria.
//!
//! \return \c void.
//!
//! \attention Modifica los bits de los registros \b UCB0CTLW0, \b UCB0IFG,
//!            \b UCB0TXBUF ,\b SR y \b UCB0IE.
//*****************************************************************************
void M24LC512_pageWrite(uint16_t* StartAddress , uint8_t *Data,
                        const uint16_t Size);

//*****************************************************************************
//! \brief Realiza una lectura de datos en la dirección actual en que se
//!        encuentre la memoria.
//!
//! \details \b Descripción \n
//!          El \ b 24LC512 contiene un contador de direcciones que mantiene
//!          la dirección de la última palabra accedida, incrementada
//!          internamente en "1". Por lo tanto, si el acceso de lectura
//!          anterior era a la dirección 'n' (n es cualquier dirección válida),
//!          la siguiente operación de lectura de dirección actual accedería a
//!          los datos de la dirección n + 1.
//!          Al recibir el byte de control con \b R/W bit establecido en
//!          '1', el \b 24LC512 emite un acknowledge(\b ACK) y transmite la
//!          palabra de datos de 8 bits. El maestro emite un \b NACK que indica
//!          el recibimiento de la transferencia, genera una condición de stop
//!          y el \b 24LC512 interrumpe la transmisión.
//!
//! \note Se envía la condición de stop antes de almacenar el dato
//!       sino la memoria continuaría enviando datos como si se trátese de
//!       una lectura secuencial.
//!
//! \return \c UCB0RXBUF Valor que se encuentra almacenado en la memoria.
//!
//! \attention Modifica los bits de los registros \b UCB0CTLW0, \b UCB0IFG,
//!            \b UCB0RXBUF, \b SR y \b UCB0IE.
//*****************************************************************************
unsigned char M24LC512_currentRead(void);

//*****************************************************************************
//! \brief Operación de lectura aleatoria. Los datos se leen desde la EEPROM.
//!        La dirección EEPROM se define con el parámetro \b Address.
//!
//! \details \b Descripción \n
//!          Las operaciones de lectura aleatoria permiten al maestro acceder
//!          a cualquier ubicación de memoria de manera aleatoria. Para
//!          realizar este tipo de operación de lectura, primero se debe
//!          establecer la dirección a la cual se quiere acceder. Esto se hace
//!          enviando la dirección de la palabra al \b 24LC512 como parte de
//!          una operación de escritura (bit R/W establecido en "0"). Después
//!          de que se envía la dirección y recibir el \b ACK, el maestro
//!          genera una condición de start. Esto finaliza la operación de
//!          escritura, pero no antes de establecer el puntero de dirección
//!          interno de la memoria en la dirección indicada. Luego, el maestro
//!          emite el byte de control nuevamente, pero con el bit R/W
//!          establecido en uno. El \b 24LC512 emitirá un \b ACK y transmitirá
//!          el byte de datos. El maestro emite un \b NACK, y genera una
//!          condición de detención que hace que el \b 24LC512 interrumpa
//!          transmisión. Después de un comando de lectura aleatorio, el
//!          contador de dirección interno apuntará a la ubicación de la
//!          dirección después de la que se acaba de leer.
//!
//! \note Se envía la condición de stop antes de almacenar el dato
//!       sino la memoria continuaría enviando datos como si se trátese de
//!       una lectura secuencial.
//!
//! \param address Dirección de la memoria de donde el micro quiere obtener un
//!        byte de datos.
//!
//! \return \c UCB0RXBUF Valor que se encuentra almacenado en la memoria.
//!
//! \attention Modifica los bits de los registros \b UCB0CTLW0, \b UCB0IFG,
//!            \b UCB0TXBUF ,\b UCB0RXBUF, \b SR y \b UCB0IE.
//*****************************************************************************
uint8_t M24LC512_randomRead(const uint16_t Address);

//*****************************************************************************
//! \brief Operación de lectura secuencial. Los datos se leen desde la EEPROM
//!        en forma secuencial desde el parámetro \b address  como punto de
//!        partida. Especifique el tamaño que se leerá y completará en un búfer
//!        de datos.
//!
//! \details \b Descripción \n
//!          Las operaciones de lectura secuencial permiten al maestro acceder
//!          a cualquier ubicación de memoria y a partir de ahí leer la cantidad
//!          de datos que se requieran. Para realizar este tipo de operación de
//!          lectura, primero se debe establecer la dirección a la cual se
//!          quiere acceder. Esto se hace enviando la dirección de la palabra
//!          al \b 24LC512 como parte de una operación de escritura (bit R/W
//!          establecido en "0"). Después de que se envía la dirección y
//!          recibir el \b ACK, el maestro genera una condición de stop.
//!          Esto finaliza la operación de escritura, pero no antes de
//!          establecer el puntero de dirección interno de la memoria en la
//!          dirección indicada. Luego, el maestro emite el byte de control
//!          nuevamente, pero con el bit R/W establecido en uno.
//!          El \b 24LC512 emitirá un \b ACK y transmitiráel byte de datos.
//!          Después de que el \b 24LC512 transmite el primer
//!          byte de datos, el maestro emite un \b ACK. Este reconocimiento
//!          dirige al \b 24LC512 a transmitir la siguiente palabra de 8 bits
//!          dirigida secuencialmente. Después del byte final transmitido al
//!          maestro, el maestro emite un \b NACK, y generará una condición de
//!          stop. Para proveer lecturas secuenciales, el \b 24LC512
//!          contiene un puntero de dirección interno que se incrementa en uno
//!          al finalizar cada operación. Este puntero de dirección permite
//!          que todo el contenido de la memoria se lea en serie durante una
//!          operación. El puntero de dirección interno pasará automáticamente
//!          de la dirección FFFF a la dirección 0000 si el maestro reconoce
//!          el byte recibido de la dirección de matriz FFFF.
//!
//! \note Se envía la condición de stop antes de almacenar el dato
//!       sino la memoria enviara un datos de mas que genera problemas en
//!       la lectura.
//!
//! \param Address Dirección inicial de la memoria a partir de la cual el
//!        microcontrolador quiere obtener una cierta cantidad de datos.
//! \param *Data Puntero donde se almacenan los datos leidos en forma
//!        secuencial en la memoria desde una dirección incial indicada.
//! \param Size Cantidad de datos a leer en la memoria.
//!
//! \return \c void
//!
//! \attention Modifica los bits de los registros \b UCB0CTLW0, \b UCB0IFG,
//!            \b UCB0TXBUF ,\b UCB0RXBUF, \b SR y \b UCB0IE.
//*****************************************************************************
void M24LC512_sequentialRead(uint16_t Address, uint8_t *Data,
                             uint16_t Size);

//*****************************************************************************
//! \brief Acknowledge Polling. La EEPROM no reconocera al MCU si hay un ciclo
//!        de escritura en curso. Se puede usar para determinar cuándo se
//!        completa un ciclo de escritura.
//!
//! \details \b Descripción \n
//!          Como el dispositivo no reconocerá ningún comando durante un ciclo
//!          de escritura, esta función se puede usar para determinar cuándo se
//!          completa el ciclo (esta función se puede usar para maximizar el
//!          rendimiento del bus).
//!          Una vez que el maestro ha emitido la condición de stop para un
//!          comando de escritura, lo cual se realiza en otras funciones de
//!          escritura, el dispositivo inicia el ciclo de escritura temporizado
//!          internamente. El \b ACK polling puede iniciarse de inmediato. Esto
//!          implica que el maestro envíe una condición de start, seguido del
//!          byte de control para un comando de Escritura (R / W = 0).
//!          Si el dispositivo todavía está ocupado con el ciclo de escritura,
//!          no se devolverá ningún ACK. Si no se devuelve ningún \b ACK, el
//!          bit de inicio y el byte de control deben volver a enviarse. Si el
//!          ciclo se completa, el dispositivo devolverá el \b ACK y luego se
//!          termina la comunicación enviando una condición de stop y el
//!          maestro podrá continuar con el siguiente comando de  lectura o
//!          escritura.
//!
//! \return \c void.
//!
//! \attention Modifica los bits de los registros \b UCB0CTLW0, \b UCB0IFG,
//!            \b SR, \b TA1CTL, \b TA1CCTL0 y\b TA1CCR0.
//*****************************************************************************
void M24LC512_ackPolling(void);

//*****************************************************************************
//! \brief Establece los valores iniciales de la cabecera en la memoria.
//!
//! \details \b Descripción \n
//!          Se establecen las direcciones y valores iniciales de la cabecera
//!          en la memoria externa.
//!
//! \return \c void.
//*****************************************************************************
void M24LC512_setinitValueHeader(void);

//*****************************************************************************
//! \brief Actualiza la cabecera en memoria.
//!
//! \details \b Descripción \n
//!          Se encarga de ir actualizando la cabecera en memoria cada vez
//!          que ocurre una escritura en la misma o cuando se reciba una
//!          comando que asi lo requiera.
//!
//! \param currentAddress Direccion actual que apunta a la proxima dirección
//!                       de la memoria a escribir.
//! \param size Indica la cantidad de bytes escritos en la memoria.
//!
//! \return \c void.
//*****************************************************************************
void M24LC512_updateHeader(const uint16_t currentAddress, const uint16_t size);

//*****************************************************************************
//! \brief Realiza un chequeo del estado de la memoria.
//!
//! \details \b Descripción \n
//!          Función que comprueba si la memoria funciona correctamente
//!          enviando un start una cierta cantidad de veces y verifica si este
//!          se transimite correctamente, es decir, verifica si recibe un ACK
//!          o si falla la comunicacion. Si el start es reconocido se realiza
//!          una transmision cualquiera para no generar problemas en
//!          posteriores escrituras ya que al transmitir solo el stop genera
//!          problemas en posteriores accesos a la memoria.
//!
//! \return Indica si hubo exito, o fallo, la comunicacion con la memoria.
//!
//! \attention Modifica los bits de los registros \b UCB0CTLW0, \b UCB0IFG,
//!            \b SR, \b TA1CTL, \b TA1CCTL0 y\b TA1CCR0.
//*****************************************************************************
bool M24LC512_memoryCheck(void);

float MLX90614_getTemp      (uint8_t MLX90614_ADDRESSTEMP, uint8_t *temp);
void  MLX90614_sleepMode    (void);
void  MLX90614_delay_ms     (uint16_t);
void  MLX90614_exitSleepMode(void);
void  MLX90614_showTemp     (float);

void     SHT3_getData       (uint8_t*);
uint16_t SHT3_calculateTemp (uint8_t*);
uint16_t SHT3_calculateHum  (uint8_t*);
void     SHT3_showTempHum   (uint16_t g_Temp,uint16_t g_Hum);



#endif /* I2C_H_ */
