/******************************************************************************
 * Модуль:      ads1256.c
 * Автор:       Celeron (c) 2018 
 * Назначение:  Драйвер внешнего 24-битного АЦП "ADS1256" с SPI интерфейсом.
 ******************************************************************************/
 
#include "stm32f1xx_hal.h"          // подключаем HAL API
#include <stdio.h>                  // printf...

#include "delays.h"                 // Функции простой блокирующей задержки (для точной выдержки)
#include "ads1256_defs.h"           // Псевдонимы Управляющих Регистров и Команд микросхемы ADS1256
#include "ads1256.h"                // прототипы локальных функций и типов



//============================================================================
// Подключаем АЦП (аппаратная часть)
//============================================================================


// Подключаем SPI интерфейс, на котором висит внешнее АЦП
#define SPI_ADC_HANDLE   hspi2

// Структуры данных для управления SPI/АЦП (External variables)
extern SPI_HandleTypeDef SPI_ADC_HANDLE;


// Настройка SPI: 
//  CPOL = Low; CPHA=2 Edge.
//
// Википедия:  https://ru.wikipedia.org/wiki/Serial_Peripheral_Interface
//  CPOL = 0 — сигнал синхронизации начинается с низкого уровня;
//  CPHA = 1 — выборка данных производится по заднему фронту сигнала синхронизации.
//
// ADS1256 Datasheet:  http://www.ti.com/lit/ds/symlink/ads1256.pdf
//  ADS1256 Datasheet / Figure 1. Serial Interface Timing (page 6) показывает требуемый режим интерфейса.
//  см. T4 T5 - это тайминги смешения DIN импульса относительно ВТОРОГО ПЕРЕХОДА.
//  Причём, ответ DOUT поступает в той же фазе (по второму фронту) - это стандарт SPI, надо знать! 
//  А тайминги T7 T8 специфицируют "перелёты" (запрещённые состояния сигнала). 
//  Пояснение: Тайминги T7 T8 указываются относительно "первого перехода", потому что на первом переходе происходит переключение сигнала. Но Сигнал затем удерживается весь следующий период (посередине которого проходит "второй переход синхры" - и всё четко читается с MISO=DOUT).
//
// STM32CubeMX визуальный конфигуратор
//  Вкладка "Configuration" / в секции "Connectivity" / жмем кнопку соответствующего "SPIx" интерфейса
//  Clock Polarity (CPOL) = Low
//  Clock Phase    (CPHA) = 2 Edge



//-------------------------------------
// Порт DRDY (Data Ready - готовность результата)
//-------------------------------------

// Контроль порта DRDY (готовность семпла к выборке) (а также, свободность АЦП принимать и реагировать на команды - не занятость в "калибровке" и "конвертации", и др. внутренних задачах)
#define  ADS1256_DRDY_PIN         GPIO_PIN_13
#define  ADS1256_DRDY_PORT        GPIOD

#if defined(ADS1256_DRDY_PORT) && defined (ADS1256_DRDY_PIN)
  // Обратная связь по асинхронной шине (очень быстро и полнофункционально, поддерживаются все режимы конвертации; 
  //  однако требует выделение дополнительного Порта и аппаратной шины DRDY для подключения АЦП)
  #define  ADS1256_DRDY_BUSY()    (HAL_GPIO_ReadPin(ADS1256_DRDY_PORT, ADS1256_DRDY_PIN) == GPIO_PIN_SET )
  #define  ADS1256_DRDY_READY()   (HAL_GPIO_ReadPin(ADS1256_DRDY_PORT, ADS1256_DRDY_PIN) == GPIO_PIN_RESET )

#else
  // Обратная связь по циклическому запросу значения бита DRDY, из внутреннего регистра микросхемы, через SPI 
  //  (очень медленно, причем синхронизация отсутствует - поэтому нельзя использовать режим "Read data continously"!)
  #define  ADS1256_DRDY_BUSY()    (ADS1256_API_GetDataReadyBit() == ADS1256_STATUS_DRDY_BUSY)
  #define  ADS1256_DRDY_READY()   (ADS1256_API_GetDataReadyBit() == ADS1256_STATUS_DRDY_READY)

  // Как вариант: вообще отключить контроль готовности (настоятельно не рекомендуется!) - работает, если делать только одиночные замеры и с большими таймаутами...
  //#define  ADS1256_DRDY_BUSY()    0 
  //#define  ADS1256_DRDY_READY()   1
                                    // Примечание: 0 или 1 - здесь это не состояние порта или бита, здесь уже логическое значение!
                                    // Прописаны такие значения-заглушки, чтобы статус был "всегда разрешено".
#endif



//-------------------------------------
// Порт CS (Chip Select - разрешена передача)
//-------------------------------------

// Важно: активация сигнала CS также сбрасывает SPI интерфейс внутри ведомого устройства (Модуля ADS1256), что восстанавливает его способность принимать очередной Пакет.  
//  (Повторю: спад сигнала /CS делает сброс-reset, но не всей микросхеме ADS1256, а только подсистеме интерфейса SPI внутри микросхемы, которая является "ведомым устройством" - т.о. содержимое "регистров конфигурации АЦП" сохраняются, но обнуляются "счетчики тактовых импульсов", отмеряющие байты Посылки, и сбрасываются "биты состояния" флагового автомата интерфейса SPI...)
//  Datasheet (page 26): "CS must remain low for the duration of the serial communication. When CS is taken high, the serial interface is reset and DOUT enters a high impedance state. CS may be permanently tied low..."
//
// Сигнал CS - стробирует начало Пакета данных! 
//  Рекомендуется: всегда использовать этот сигнал параллельно с Шинами SPI интерфейса. И каждую ВЫСОКОУРОВНЕВУЮ операцию связи "запрос-ответ" покрывать ОТДЕЛЬНЫМ но ЕДИНЫМ импульсом CS.


// Контроль порта CS (включение интерфейса SPI в удаленном ADS1256 модуле - способность АЦП слушать команды и выдавать ответы)
#define  ADS1256_CS_PORT          GPIOB, GPIO_PIN_12

#ifdef   ADS1256_CS_PORT
  // Управление шиной CS осуществляется вручную, через Порт ввода-вывода...
  #define ADS1256_CS_ON()      HAL_GPIO_WritePin (ADS1256_CS_PORT, GPIO_PIN_RESET)
  #define ADS1256_CS_OFF()     HAL_GPIO_WritePin (ADS1256_CS_PORT, GPIO_PIN_SET)
  //#define ADS1256_CS_TOGGLE()  HAL_GPIO_TogglePin(ADS1256_CS_PORT)

#else
  // Если программный контроль CS-порта на Мастере не используется:
  //  т.е. либо Контроля нет вообще и /CS на Ведомых подтянут жестко к Земле; 
  //  либо на Мастере используется "аппаратный SPI.NSS" сигнал...
  #define ADS1256_CS_ON()
  #define ADS1256_CS_OFF()
  //#define ADS1256_CS_TOGGLE()
  
#endif



//-------------------------------------
// Настройка Частоты и Таймингов 
//-------------------------------------


// Внутренняя частота кварца, тактирующего микросхему АЦП, Гц
//  (Стандартный кварц F_CLKIN = 7.68MHz)
#define  ADS1256_F_CLKIN  7680000


// Частота семплирования АЦП (количество замеров в секунду), SPS
// Максимально допустимая для данной аппаратной конфигурации (ограничение сверху)
//  Замечу: функциями API можно, оперативно, устанавливать и меньшие скорости семплирования из стандартного ряда (для большего усреднения сигнала).
//
//  Диапазон допустимых значений параметра F_DATA = [2.5 .. 30000] SPS  (при F_CLKIN=7.68MHz)
//  Важно: значение должно точно равнятся одному из стандартного ряда = 30000, 15000, 7500, 3750, 2000, 1000, 500, 100, 60, 50, 30, 25, 15, 10, 5, 2.5
#define  ADS1256_F_DATA   15000  /*предельный максимум для текущей аппаратной конфигурации, но я не буду его использовать - возьму с запасом, в 2 раза медленнее*/
//#define  ADS1256_F_DATA   7500  /*лучше использовать это*/


//-------------------------------------
// Предварительно, ввести сюда тайминги настройки частоты Шины SPI из STM32Cube:
#define  STM32_MCU_SYSCLK             72000000                /*частота ядра*/
#define  STM32_MCU_APBCLK             ((STM32_MCU_SYSCLK)/2)  /*частота периферийной шины (половинная)*/
#define  STM32_SPI_BAUDRATEPRESCALER  64                      /*предделитель APB формирует частоту SPI*/

// Итого, Частота интерфейса SPI, Гц
#define  ADS1256_F_SCLK               ((STM32_MCU_APBCLK)/(STM32_SPI_BAUDRATEPRESCALER))


//-------------------------------------
// Проверка: Диапазон допустимых значений Частоты интерфейса SPI по datasheet:
//  Период SPI должен быть в пределе T_SCLK = [ 4*(1/F_CLKIN) .. 10*(1/F_DATA) ], т.е. от 4х тактов внутренней частоты до 10х семплов.
//  Диапазон допустимых значений частоты F_SCLK = F_DATA/10 .. F_CLKIN/4
#define  ADS1256_F_SCLK_MAX   ((ADS1256_F_CLKIN)/4)   /*максимальная частота внешнего интерфейса SPI, при котором микросхема АЦП еще способна обрабатывать поступающие входные данные, равна 4х кратной от собственной внутренней частоты*/
//#define  ADS1256_F_SCLK_MIN   ((ADS1256_F_DATA)/10)   /*самая медленная допустимая скорость SPI, чтобы интерфейс еще обслуживался и не детектировался timeout*/

// Подход: Нижняя граница требуемой скорости SPI определяется условием - чтобы успевать выгребать поток результатов АЦП в непрерывном режиме RDATAC, при данной частоте семплирования...
//#define  ADS1256_F_SCLK_MIN   ((ADS1256_F_DATA)*24*2)   /*тупо и ненадежно: пусть, надо выгрести 24-битный семпл + удваиваем время, т.е. отфонарно надеемся, что все таймауты суммарно равны времени непосредственной выборки*/

// А теперь, алгоритм "по уму" (со всеми безопасными таймаутами):
//    OneSampleCycle = 50*T_CLKIN(deadtime) + 24bit*T_SCLK(fetch new sample) + 50*T_CLKIN(deadtime) + 16*T_CLKIN(DRDY update period) = 1/F_DATA(samples per sec)
//    116/F_CLKIN + 24/F_SCLK = 1/F_DATA
//    F_SCLK = 24*F_CLKIN*F_DATA/(F_CLKIN-116*F_DATA)
#define  ADS1256_F_SCLK_MIN   (24*(ADS1256_F_CLKIN)*(ADS1256_F_DATA)/((ADS1256_F_CLKIN) - 116*(ADS1256_F_DATA)))

#if ((ADS1256_F_SCLK) <= (ADS1256_F_SCLK_MIN)) || ((ADS1256_F_SCLK) >= (ADS1256_F_SCLK_MAX))
#error "Ошибка: значение ADS1256_F_SCLK вне границ допустимого!"
#endif


//-------------------------------------
// Таймауты между Командами в протоколе АЦП:

// Вспомогательный макрос: преобразует единицы измерения времени из "тактов внутреннего таймера" в "мкс"
#define _TCLKIN_TO_US(T_CLKIN)   ((T_CLKIN) * 1000000 / (ADS1256_F_CLKIN) +1)

// Delay from last SCLK edge for DIN to first SCLK rising edge for DOUT (RDATA, RDATAC, RREG Commands)
#define ADS1256_DELAY_T6_CLKIN   (50*2)                                   /* здесь, минимум = 50 T_CLKIN, но возьмем удвоенную задержку */
#define ADS1256_DELAY_T6_US      _TCLKIN_TO_US(ADS1256_DELAY_T6_CLKIN)    /* в мкс */

// Final SCLK falling edge of command to first SCLK rising edge of next command (задержка между Командами)
//  После запуска предыдущей команды, АЦП нужно время на обработку, чтобы приготовиться к принятию следующей команды
#define ADS1256_DELAY_T11_CLKIN  (24*2)                                   /* некоторым командам достаточно 4 T_CLKIN, но большинству нужно 24 T_CLKIN... также, удвоим для верности */
#define ADS1256_DELAY_T11_US     _TCLKIN_TO_US(ADS1256_DELAY_T11_CLKIN)   /* в мкс */

// Таймаут для синхронных операций ввода-вывода (передается в HAL-функции), мс
#define HAL_IO_TIMEOUT   50



//============================================================================
// Отладка АЦП
//============================================================================


// Отладочный вывод в Консоль
#ifdef USE_FULL_ASSERT

  // Note: Recipe from  http://qaru.site/questions/21384/is-there-a-printf-converter-to-print-in-binary-format
  #define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
  #define BYTE_TO_BINARY(byte)  \
    (byte & 0x80 ? '1' : '0'), \
    (byte & 0x40 ? '1' : '0'), \
    (byte & 0x20 ? '1' : '0'), \
    (byte & 0x10 ? '1' : '0'), \
    (byte & 0x08 ? '1' : '0'), \
    (byte & 0x04 ? '1' : '0'), \
    (byte & 0x02 ? '1' : '0'), \
    (byte & 0x01 ? '1' : '0') 


  // Поля данных
  static volatile uint8_t  ADS1256_LogOn   = 0;   // Флаг "логи включены?"
  static    char *volatile ADS1256_LogFile = 0;   // Последняя "метка включения лога"
  static volatile uint32_t ADS1256_LogLine = 0;


  // Методы
  void ADS1256_Log_Print(uint8_t* caption, uint32_t value)
  {
    if(ADS1256_LogOn)
    {
      printf( "ADS1256 driver (LOG ON = file \"%s\" on line %d):\t\"%s\" = 0x%02X = 0b%c%c%c%c %c%c%c%c\r\n", 
              ADS1256_LogFile, 
              ADS1256_LogLine,
              caption, 
              value, 
              BYTE_TO_BINARY(value));
    }
    return;
  }
  
  
  // Forward declarations
  uint8_t ADS1256_Command_ReadFromRegister(uint8_t regaddr);


  //-------------------------------------
  // Запрашивает из Устройства и Распечатывает в отладочную консоль Текущие ЗНАЧЕНИЯ ВСЕХ РЕГИСТРОВ
  void ADS1256_Log_DumpRegisters(uint8_t* caption)
  {
    uint8_t value;
    printf("\r\n\r\n%s\r\n", caption);
    printf("--- ADS1256 driver / DUMP REGISTERS (start) ---\r\n");
    
    value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_STATUS);
    printf("ADS1256_REGISTER_STATUS (0x00) = 0x%02X = 0b%c%c%c%c %c%c%c%c\r\n", value, BYTE_TO_BINARY(value));

    value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_MUX);
    printf("ADS1256_REGISTER_MUX    (0x01) = 0x%02X = 0b%c%c%c%c %c%c%c%c\r\n", value, BYTE_TO_BINARY(value));

    value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_ADCON);
    printf("ADS1256_REGISTER_ADCON  (0x02) = 0x%02X = 0b%c%c%c%c %c%c%c%c\r\n", value, BYTE_TO_BINARY(value));

    value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_DRATE);
    printf("ADS1256_REGISTER_DRATE  (0x03) = 0x%02X = 0b%c%c%c%c %c%c%c%c\r\n", value, BYTE_TO_BINARY(value));

    value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_IO);
    printf("ADS1256_REGISTER_IO     (0X04) = 0x%02X = 0b%c%c%c%c %c%c%c%c\r\n", value, BYTE_TO_BINARY(value));

    value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_OFC0);
    printf("ADS1256_REGISTER_OFC0   (0x05) = 0x%02X = 0b%c%c%c%c %c%c%c%c\r\n", value, BYTE_TO_BINARY(value));

    value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_OFC1);
    printf("ADS1256_REGISTER_OFC1   (0x06) = 0x%02X = 0b%c%c%c%c %c%c%c%c\r\n", value, BYTE_TO_BINARY(value));

    value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_OFC2);
    printf("ADS1256_REGISTER_OFC2   (0x07) = 0x%02X = 0b%c%c%c%c %c%c%c%c\r\n", value, BYTE_TO_BINARY(value));

    value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_FSC0);
    printf("ADS1256_REGISTER_FSC0   (0x08) = 0x%02X = 0b%c%c%c%c %c%c%c%c\r\n", value, BYTE_TO_BINARY(value));
    
    value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_FSC1);
    printf("ADS1256_REGISTER_FSC1   (0x09) = 0x%02X = 0b%c%c%c%c %c%c%c%c\r\n", value, BYTE_TO_BINARY(value));
    
    value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_FSC2);
    printf("ADS1256_REGISTER_FSC2   (0x0A) = 0x%02X = 0b%c%c%c%c %c%c%c%c\r\n", value, BYTE_TO_BINARY(value));

    printf("--- ADS1256 driver / DUMP REGISTERS (end) ---\r\n");
  }  


  //-------------------------------------
  // Интерфейс
  #define ADS1256_LOG_ON()  {ADS1256_LogOn   = 1;        \
                             ADS1256_LogFile = __FILE__; \
                             ADS1256_LogLine = __LINE__; }

  #define ADS1256_LOG_OFF() {ADS1256_LogOn   = 0;}

  #define ADS1256_LOG_PRINT(CAPTION, VALUE)  { ADS1256_Log_Print((CAPTION),(VALUE)); }
  
  #define ADS1256_LOG_DUMPREGISTERS(CAPTION) { ADS1256_Log_DumpRegisters((#CAPTION)); }

#else

  // Интерфейс
  #define ADS1256_LOG_ON()
  #define ADS1256_LOG_OFF()
  #define ADS1256_LOG_PRINT(CAPTION, VALUE)
  #define ADS1256_LOG_DUMPREGISTERS(CAPTION)

#endif



 

//============================================================================
// Низкоуровневые команды АЦП  (синхронные операции)
//============================================================================

// Внимание: нельзя использовать "низкоуровневые команды" в API модуля и в "прикладном коде" - поскольку они не обрабатывают сигнал CS!
//  Разрешить и использовать отдельные команды вы можете только на свой страх и риск, и только если точно знаете что делаете...
//
// Исключения:
//  Например, вы можете использовать метод ADS1256_Command_EnsureDRDY(), если сигнал DRDY сигнал заходит на отдельный порт GPIO и не использует от SPI.
//  Также, вы технически можете использовать "низкоуровневые методы", если вы вообще не используете сигнал CS в аппаратной конфигурации, а соответствующие Порты (/CS) на "ведомых устройствах" (Slave) жестко подтянули к шине Земли (/CS=GND -> CS=='1').
//  Datasheet (page 26): "CS must remain low for the duration of the serial communication. When CS is taken high, the serial interface is reset and DOUT enters a high impedance state. CS may be permanently tied low..."



//-------------------------------------
// Конфигурация регистров  
//-------------------------------------

//-------------------------------------
// RESET: Reset Registers to Default Values
//
// Description: Returns all registers except the CLK0 and CLK1 bits in the ADCON register to their default values.
//  This command will also stop the Read Continuous mode: in this case, issue the RESET command after DRDY goes low.
//
void ADS1256_Command_Reset(void)
{
  // Синхронная задержка до готовности АЦП
  while(ADS1256_DRDY_BUSY());

  // Буфер
  uint8_t TxBuffer = ADS1256_COMMAND_RESET;
  // Команда
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));

  // Обязательная "задержка между командами", пока АЦП восстановится после только что принятой команды
  delay_us(ADS1256_DELAY_T11_US);
  
  // Datasheet: "After a reset, the ADS1255/6 performs self-calibration... "
  // Синхронная задержка до готовности АЦП  (пока завершится автокалибровка)
  while(ADS1256_DRDY_BUSY());
}



//-------------------------------------
// Чтение данных из указанного Конфигурационного Регистра АЦП
// RREG: Read from Registers
uint8_t ADS1256_Command_ReadFromRegister(uint8_t regaddr)
{
  // Синхронная задержка до готовности АЦП
  //while(ADS1256_DRDY_BUSY());     //здесь, не требуется ограничивать доступ, т.к. "чтение регистров" не изменяет режим АЦП...
  
  // Буферы
  uint8_t TxBuffer;                                                                         // Buffer dlya otpravlyaemih dannih
  uint8_t RxBuffer = 0;                                                                     // Buffer dlya prinimaemih dannih

  // Запрос
  TxBuffer = ADS1256_COMMAND_RREG | (regaddr & 0xF);                                        // Ukazivaem adres registra s kotorogo nachinaetsa chtenie
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));  // Peredaem adress nachalnogo registra

  TxBuffer = 0;                                                                             // Ukazivaem kolichestvo registrov iz kotorih budut schitivatsa dannie (zapisivaetsa 1 + TO KOLICHESTVO KOTOROE UKAZIVAEM, V DANOM SLUCHAE POLUCHAETSA 1+0=1)
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));  // Peredaem kolichestvo registrov

  // Обязательная задержка, пока микросхема АЦП подготовит ответ
  delay_us(ADS1256_DELAY_T6_US);
  
  // Ответ
  assert_param(HAL_OK == HAL_SPI_Receive(&SPI_ADC_HANDLE, &RxBuffer, 1, HAL_IO_TIMEOUT));   // Schitivaem dannie iz registra

  // Обязательная "задержка между командами", пока АЦП восстановится после только что принятой команды
  delay_us(ADS1256_DELAY_T11_US);
  
  return RxBuffer;                                                                          // Vozvrashaem peremennoy znachenie vibrannogo registra
}



//-------------------------------------
// Запись данных в указанный Конфигурационный Регистр АЦП
// WREG: Write to Register
void ADS1256_Command_WriteToRegister(uint8_t regaddr, uint8_t value)
{
  // Синхронная задержка до готовности АЦП
  while(ADS1256_DRDY_BUSY());
  
  // Буферы
  uint8_t TxBuffer;                                                                         // Buffer dlya otpravlyaemih dannih

  // Команда
  TxBuffer = ADS1256_COMMAND_WREG | (regaddr & 0xF);                                        // Ukazivaem adres registra s kotorogo nachinaetsa chtenie
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));  // Peredaem adress nachalnogo registra

  TxBuffer = 0;                                                                             // Ukazivaem kolichestvo registrov iz kotorih budut schitivatsa dannie (zapisivaetsa 1 + TO KOLICHESTVO KOTOROE UKAZIVAEM, V DANOM SLUCHAE POLUCHAETSA 1+0=1)
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));  // Peredaem kolichestvo registrov

  // Данные
  TxBuffer = value;                                                                         // Ukazivaem kolichestvo registrov iz kotorih budut schitivatsa dannie (zapisivaetsa 1 + TO KOLICHESTVO KOTOROE UKAZIVAEM, V DANOM SLUCHAE POLUCHAETSA 1+0=1)
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));  // Peredaem kolichestvo registrov

  // Обязательная "задержка между командами", пока АЦП восстановится после только что принятой команды
  delay_us(ADS1256_DELAY_T11_US);
}




//-------------------------------------
// Калибровка
//-------------------------------------

//-------------------------------------
// SELFCAL: Self Offset and Gain Calibration
//
// Description: Performs a self offset and self gain calibration. 
//  The Offset Calibration Register (OFC) and Full-Scale Calibration Register (FSC) are updated after this operation. 
//  DRDY goes high at the beginning of the calibration. It goes low after the calibration completes and settled data is ready. 
//  Do not send additional commands after issuing this command until DRDY goes low indicating that the calibration is complete.
//
// Note:  For the best performance, it is strongly recommended to perform an additional self-calibration 
//  by issuing the SELFCAL command after the power supplies and voltage reference have had time to settle to their final values.
//
void ADS1256_Command_SelfCalibration(void)
{
  // Синхронная задержка до готовности АЦП
  while(ADS1256_DRDY_BUSY());
  
  // Буфер
  uint8_t TxBuffer = ADS1256_COMMAND_SELFCAL;
  // Команда
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));

  // Обязательная "задержка между командами", пока АЦП восстановится после только что принятой команды
  delay_us(ADS1256_DELAY_T11_US);

  // Синхронная задержка до готовности АЦП  (пока завершится автокалибровка)
  while(ADS1256_DRDY_BUSY());
}



//-------------------------------------
// SELFOCAL: Self Offset Calibration
void ADS1256_Command_SelfOffsetCalibration(void)
{
  // Синхронная задержка до готовности АЦП
  while(ADS1256_DRDY_BUSY());
  
  // Буфер
  uint8_t TxBuffer = ADS1256_COMMAND_SELFOCAL;
  // Команда
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));

  // Обязательная "задержка между командами", пока АЦП восстановится после только что принятой команды
  delay_us(ADS1256_DELAY_T11_US);

  // Синхронная задержка до готовности АЦП  (пока завершится автокалибровка)
  while(ADS1256_DRDY_BUSY());
}



//-------------------------------------
// SELFGCAL: Self Gain Calibration
void ADS1256_Command_SelfGainCalibration(void)
{
  // Синхронная задержка до готовности АЦП
  while(ADS1256_DRDY_BUSY());
  
  // Буфер
  uint8_t TxBuffer = ADS1256_COMMAND_SELFGCAL;
  // Команда
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));

  // Обязательная "задержка между командами", пока АЦП восстановится после только что принятой команды
  delay_us(ADS1256_DELAY_T11_US);

  // Синхронная задержка до готовности АЦП  (пока завершится автокалибровка)
  while(ADS1256_DRDY_BUSY());
}



//-------------------------------------
// SYSOCAL: System Offset Calibration
void ADS1256_Command_SystemOffsetCalibration(void)
{
  // Синхронная задержка до готовности АЦП
  while(ADS1256_DRDY_BUSY());
  
  // Буфер
  uint8_t TxBuffer = ADS1256_COMMAND_SYSOCAL;
  // Команда
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));

  // Обязательная "задержка между командами", пока АЦП восстановится после только что принятой команды
  delay_us(ADS1256_DELAY_T11_US);

  // Синхронная задержка до готовности АЦП  (пока завершится автокалибровка)
  while(ADS1256_DRDY_BUSY());
}



//-------------------------------------
// SYSGCAL: System Gain Calibration
void ADS1256_Command_SystemGainCalibration(void)
{
  // Синхронная задержка до готовности АЦП
  while(ADS1256_DRDY_BUSY());
  
  // Буфер
  uint8_t TxBuffer = ADS1256_COMMAND_SYSGCAL;
  // Команда
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));

  // Обязательная "задержка между командами", пока АЦП восстановится после только что принятой команды
  delay_us(ADS1256_DELAY_T11_US);

  // Синхронная задержка до готовности АЦП  (пока завершится автокалибровка)
  while(ADS1256_DRDY_BUSY());
}




//-------------------------------------
// Синхронизация (для одиночных замеров в точно заданный момент времени)
//-------------------------------------

//-------------------------------------
// SYNC: Synchronize the A/D Conversion
//
// Description: This command synchronizes the A/D conversion. 
//  To use, first shift in the command SYNC...
//  ...Then shift in the command WAKEUP. 
//  Synchronization occurs on the first CLKIN rising edge of WAKEUP command.
//
void ADS1256_Command_Synchronize(void)
{
  // Синхронная задержка до готовности АЦП
  while(ADS1256_DRDY_BUSY());
  
  // Буфер
  uint8_t TxBuffer = ADS1256_COMMAND_SYNC;
  // Команда
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));

  // Обязательная "задержка между командами", пока АЦП восстановится после только что принятой команды
  delay_us(ADS1256_DELAY_T11_US);
}



//-------------------------------------
// STANDBY: Standby Mode / One-Shot Mode
//
// Description: This command puts the ADS1255/6 into a low-power Standby mode. 
//  After issuing the STANDBY command, make sure there is no more activity on SCLK while CS is low, as this will interrupt Standby mode. If CS is high, SCLK activity is allowed during Standby mode. 
//  To exit Standby mode, issue the WAKEUP command. This command can also be used to perform single conversions (see One-Shot Mode section).
//  
void ADS1256_Command_Standby(void)
{
  // Синхронная задержка до готовности АЦП
  while(ADS1256_DRDY_BUSY());
  
  // Буфер
  uint8_t TxBuffer = ADS1256_COMMAND_STANDBY;
  // Команда
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));

  // Обязательная "задержка между командами", пока АЦП восстановится после только что принятой команды
  delay_us(ADS1256_DELAY_T11_US);
}



//-------------------------------------
// WAKEUP: Complete Synchronization or Exit Standby Mode
// 
// Description: Used in conjunction with the SYNC and STANDBY commands. 
//  Two values (all zeros or all ones) are available for this command.
//
void ADS1256_Command_WakeUp(void)
{
  // Синхронная задержка до готовности АЦП
  //while(ADS1256_DRDY_BUSY());     //здесь, нельзя ставить блокировку, во избежание deadlock - неизвестно в каком состоянии АЦП заснул...
  
  // Буфер
  uint8_t TxBuffer = ADS1256_COMMAND_WAKEUP0;
  // Команда
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));

  // Обязательная "задержка между командами", пока АЦП восстановится после только что принятой команды
  delay_us(ADS1256_DELAY_T11_US);
}



//-------------------------------------

// Форсировать немедленную конвертацию (чтобы актуализировать данные в регистре результата)
#define ADS1256_COMMAND_CONVERT() \
  ADS1256_Command_Synchronize();  \
  ADS1256_Command_WakeUp();




//============================================================================
// Настройка режимов микросхемы АЦП  (высокоуровневое API модуля, синхронные операции)
//============================================================================


//-------------------------------------
// Управление потоком выполнения
//-------------------------------------


//-------------------------------------
// Программный Сброс: Reset Registers to Default Values

void ADS1256_API_Reset(void)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  ADS1256_CS_ON();                                                            //Разрешить передачу
  ADS1256_Command_Reset();
  ADS1256_CS_OFF();                                                           //Отключить передачу
}



//-------------------------------------
// Послать команду SYNC: Synchronize the A/D Conversion
//  (приостановить конвертацию... но быть готовым сразу же продолжить, по команде WAKEUP)

void ADS1256_API_Synchronize(void)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  ADS1256_CS_ON();                                                            //Разрешить передачу
  ADS1256_Command_Synchronize();
  ADS1256_CS_OFF();                                                           //Отключить передачу
}



//-------------------------------------
// Послать команду STANDBY: Standby Mode / One-Shot Mode
//  (усыпить АЦП, поместить в режим пониженного энергопотребления... при этом, конвертация конечно приостанавливается, но для ее возобновления требуется больше времени, чем после команды SYNC)

void ADS1256_API_Standby(void)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  ADS1256_CS_ON();                                                            //Разрешить передачу
  ADS1256_Command_Standby();
  ADS1256_CS_OFF();                                                           //Отключить передачу
}



//-------------------------------------
// Послать команду WAKEUP: Complete Synchronization or Exit Standby Mode
//  ([разбудить АЦП, если был в спящем режиме]... и сразу после этого, произвести конвертацию)

void ADS1256_API_WakeUp(void)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  ADS1256_CS_ON();                                                            //Разрешить передачу
  ADS1256_Command_WakeUp();
  ADS1256_CS_OFF();                                                           //Отключить передачу
}




//-------------------------------------
// Настройки в регистре STATUS
//-------------------------------------


//-------------------------------------
// Заводской идентификатор микросхемы
// ID3, ID2, ID1, ID0 = Factory Programmed Identification Bits (Read Only) 

uint8_t ADS1256_API_GetDeviceID(void)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return ADS1256_STATUS_DATAC_ERROR;

  ADS1256_CS_ON();                                                            //Разрешить передачу

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_STATUS);  //Считываю значение из регистра
  value = (value >> ADS1256_STATUS_ID) & ADS1256_STATUS_IDMASK;               //Выделяю нужные биты
  
  ADS1256_CS_OFF();                                                           //Отключить передачу
  
  return value;                                                               //Возвращаю значение битов
}  



//-------------------------------------
// Порядок БИТОВ в ответе "данных семплирования"
// ORDER = Data Output Bit Order

uint8_t ADS1256_API_GetBitOrderMode(void)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return ADS1256_STATUS_DATAC_ERROR;

  ADS1256_CS_ON();                                                            //Разрешить передачу

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_STATUS);  //Считываю значение из регистра
  value = (value >> ADS1256_STATUS_ORDER) & 1;                                //Выделяю нужные биты
  
  ADS1256_CS_OFF();                                                           //Отключить передачу

  return value;                                                               //Возвращаю значение бита
}  


void ADS1256_API_SetBitOrderMode(uint8_t bit)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  ADS1256_CS_ON();                                                            //Разрешить передачу

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_STATUS);  //Считываю старое значение из регистра
  
  if(bit)
    value |= (1 << ADS1256_STATUS_ORDER);                                     //установить бит
  else
    value &= ~(1 << ADS1256_STATUS_ORDER);                                    //снять бит
  
  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_STATUS, value);            //Записываю обратно в регистр новое значение (с обновленным битом)
  
  ADS1256_CS_OFF();                                                           //Отключить передачу
}  



//-------------------------------------
// Режим "автоматически запускать процедуру перекалибровки, после каждой перенастройки режима АЦП"
// ACAL = Auto Calibration
//
// Note: When Auto-Calibration is enabled, self-calibration begins at the completion of the WREG command that changes the 
//  PGA   (bits 0-2 of ADCON register), 
//  DR    (bits 7-0 in the DRATE register) or 
//  BUFEN (bit 1 in the STATUS register) values.
//

uint8_t ADS1256_API_GetAutoCalibrationMode(void)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return ADS1256_STATUS_DATAC_ERROR;

  ADS1256_CS_ON();                                                            //Разрешить передачу

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_STATUS);  //Считываю значение из регистра
  value = (value >> ADS1256_STATUS_ACAL) & 1;                                 //Выделяю нужные биты
  
  ADS1256_CS_OFF();                                                           //Отключить передачу

  return value;                                                               //Возвращаю значение бита
}  


void ADS1256_API_SetAutoCalibrationMode(uint8_t bit)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  ADS1256_CS_ON();                                                            //Разрешить передачу

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_STATUS);  //Считываю старое значение из регистра
  
  if(bit)
    value |= (1 << ADS1256_STATUS_ACAL);                                      //установить бит
  else
    value &= ~(1 << ADS1256_STATUS_ACAL);                                     //снять бит
  
  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_STATUS, value);            //Записываю обратно в регистр новое значение (с обновленным битом)

  ADS1256_CS_OFF();                                                           //Отключить передачу
}  


//Возвращает логический результат: TRUE если автокалибровка включена; FALSE отключена.
extern __inline uint8_t ADS1256_API_IfAutoCalibrationOn(void)
{
  return (ADS1256_API_GetAutoCalibrationMode() == ADS1256_STATUS_ACAL_ON);
}



//-------------------------------------
// вкл/выкл Входной Повторитель (увеличивает импеданс до 10..80МОм, но уменьшает динамический диапазон до 0..3V)
// BUFEN = Analog Input Buffer Enable

uint8_t ADS1256_API_GetInputBufferMode(void)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return ADS1256_STATUS_DATAC_ERROR;

  ADS1256_CS_ON();                                                            //Разрешить передачу

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_STATUS);  //Считываю значение из регистра
  value = (value >> ADS1256_STATUS_BUFEN) & 1;                                //Выделяю нужные биты

  ADS1256_CS_OFF();                                                           //Отключить передачу

  return value;                                                               //Возвращаю значение бита
}  


void ADS1256_API_SetInputBufferMode(uint8_t bit)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  ADS1256_CS_ON();                                                            //Разрешить передачу

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_STATUS);  //Считываю старое значение из регистра
  
  if(bit)
    value |= (1 << ADS1256_STATUS_BUFEN);                                     //установить бит
  else
    value &= ~(1 << ADS1256_STATUS_BUFEN);                                    //снять бит

  // 1) Перенастроить режим
  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_STATUS, value);            //Записываю обратно в регистр новое значение (с обновленным битом)

  // 2) Запустить автокалибровку (это нужно, поскольку во входных цепях включается Повторитель на ОУ)
  ADS1256_Command_SelfCalibration();
  
  // 3) Форсировать немедленную конвертацию (чтобы актуализировать данные в регистре результата)
  ADS1256_COMMAND_CONVERT();
  
  ADS1256_CS_OFF();                                                           //Отключить передачу
}  


//Возвращает логический результат: TRUE если буфер включен; FALSE отключен.
extern __inline uint8_t ADS1256_API_IfInputBufferOn(void)
{
  return (ADS1256_API_GetInputBufferMode() == ADS1256_STATUS_BUFEN_ON);
}



//-------------------------------------
// Запросить статус готовности АЦП к чтению данных или принятию команды (бит DRDY)
// DRDY = Data Ready (Read Only) = Duplicates the state of the DRDY pin

uint8_t ADS1256_API_GetDataReadyBit(void)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return ADS1256_STATUS_DATAC_ERROR;

  ADS1256_CS_ON();                                                            //Разрешить передачу

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_STATUS);  //Считываю значение из регистра
  value = (value >> ADS1256_STATUS_DRDY) & 1;                                 //Выделяю нужные биты

  ADS1256_CS_OFF();                                                           //Отключить передачу

  return value;                                                               //Возвращаю значение бита
}


//Возвращает логический результат: TRUE если данные готовы; FALSE данные не готовы, еще обновляются.
extern __inline uint8_t ADS1256_API_IfDataReady(void)
{
  //Универсальный оптимальный вариант реализации: 
  //  Сперва сканируем физическую асинхронную Шину DRDY;
  //  и только если отдельная Шина не используется, то читаем бит DRDY.
  return (ADS1256_DRDY_READY());                                              // (**) Замечу: при активном DATAC-режиме, и при отсутствии отдельной шины DRDY - этот метод всегда будет возвращать FALSE.
}



//-------------------------------------
// Вспомогательная функция: ожидание готовности АЦП к чтению данных или принятию команды (сигнал DRDY)
void ADS1256_API_WaitDRDY(void)
{
  // Замечу, конкретно в этой функции: Если сигнал готовности идет по Внешнему Порту, то SPI интерфейс включать не нужно!
  #if !(defined(ADS1256_DRDY_PORT) && defined (ADS1256_DRDY_PIN))
    ADS1256_CS_ON();                                                          //Разрешить передачу
  #endif
  
  // Синхронная задержка до готовности АЦП
  while(ADS1256_DRDY_BUSY());                                                 // (**) Замечу: при активном DATAC-режиме, и при отсутствии отдельной шины DRDY - здесь будет глюк (синхронного ожидания не будет), зацикливания и зависания программы не произойдет (что лучше), но и "готовности АЦП" не обеспечит. 
                                                                              // Добавлю: при отсутствии шины DRDY, запрещено использовать DATAC-режим (он работать не будет)!
  #if !(defined(ADS1256_DRDY_PORT) && defined (ADS1256_DRDY_PIN))
    ADS1256_CS_OFF();                                                         //Отключить передачу
  #endif
}  




//-------------------------------------
// Настройка мультиплексора входных каналов (Регистр MUX)
//-------------------------------------

// Параметры: 
//  для ADS1255, Номер канала = [0..1]
//  для ADS1256, Номер канала = [0..7]


//-------------------------------------
// Коммутация каналов для "Однопроводного включения", для измерения ПОЛОЖИТЕЛЬНОГО напряжения
extern __inline void ADS1256_API_SetMultiplexerSingleP(uint8_t PositiveChannelNumber) 
{
  ADS1256_API_SetMultiplexerDifferential(PositiveChannelNumber, 0xFF);
}


//-------------------------------------
// Коммутация каналов для "Однопроводного включения", для измерения ОТРИЦАТЕЛЬНОГО напряжения
extern __inline void ADS1256_API_SetMultiplexerSingleN(uint8_t NegativeChannelNumber) 
{
  ADS1256_API_SetMultiplexerDifferential(0xFF, NegativeChannelNumber);
}


//-------------------------------------
// Коммутация каналов для "Дифференциального включения", для измерения РАЗНИЦЫ напряжений
void ADS1256_API_SetMultiplexerDifferential(uint8_t PositiveChannelNumber, uint8_t NegativeChannelNumber) 
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  uint8_t value;
  uint8_t muxp;
  uint8_t muxn;

  // Формируем значение, загружаемое в регистр
  switch (PositiveChannelNumber) 
  {
    case 0:
      muxp = ADS1256_MUXP_AIN0;
      break;
    case 1:
      muxp = ADS1256_MUXP_AIN1;
      break;
    case 2:
      muxp = ADS1256_MUXP_AIN2;
      break;
    case 3:
      muxp = ADS1256_MUXP_AIN3;
      break;
    case 4:
      muxp = ADS1256_MUXP_AIN4;
      break;
    case 5:
      muxp = ADS1256_MUXP_AIN5;
      break;
    case 6:
      muxp = ADS1256_MUXP_AIN6;
      break;
    case 7:
      muxp = ADS1256_MUXP_AIN7;
      break;
    default:
      muxp = ADS1256_MUXP_AINCOM;
  }

  switch (NegativeChannelNumber) 
  {
    case 0:
      muxn = ADS1256_MUXN_AIN0;
      break;
    case 1:
      muxn = ADS1256_MUXN_AIN1;
      break;
    case 2:
      muxn = ADS1256_MUXN_AIN2;
      break;
    case 3:
      muxn = ADS1256_MUXN_AIN3;
      break;
    case 4:
      muxn = ADS1256_MUXN_AIN4;
      break;
    case 5:
      muxn = ADS1256_MUXN_AIN5;
      break;
    case 6:
      muxn = ADS1256_MUXN_AIN6;
      break;
    case 7:
      muxn = ADS1256_MUXN_AIN7;
      break;
    default:
      muxn = ADS1256_MUXN_AINCOM;
  }

  value = muxp | muxn;
  
  
  ADS1256_CS_ON();                                                            //Разрешить передачу
  
  // 1) Перенастроить мультиплексор на новые каналы
  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_MUX, value);
  
  // 2) Форсировать немедленную конвертацию (чтобы актуализировать данные в регистре результата)
  ADS1256_COMMAND_CONVERT();
  
  ADS1256_CS_OFF();                                                           //Отключить передачу
}




//-------------------------------------
// Настройки в регистре ADCON
//-------------------------------------

//-------------------------------------
// Настройка "Транслятора тактовой Частоты на Выход" (для тактирования внешней периферии)
// CLK1, CLK0 = D0/CLKOUT Clock Out Rate Setting
//
// Примечание: не забудь также открыть GPIO на Output, чтобы транслировать этот выход наружу...

uint8_t ADS1256_API_GetClockOutMode(void)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return ADS1256_STATUS_DATAC_ERROR;

  ADS1256_CS_ON();                                                            //Разрешить передачу

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_ADCON);   //Считываю значение из регистра
  value = (value >> ADS1256_ADCON_CLK) & ADS1256_ADCON_CLKMASK;               //Выделяю нужные биты

  ADS1256_CS_OFF();                                                           //Отключить передачу

  return value;                                                               //Возвращаю значение битов
}


void ADS1256_API_SetClockOutMode(uint8_t bits)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  ADS1256_CS_ON();                                                            //Разрешить передачу

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_ADCON);   //Считываю старое значение из регистра
  
  // Сначала очищаем нужную область битов, по маске
  value &= ~(ADS1256_ADCON_CLKMASK << ADS1256_ADCON_CLK);
  
  // Затем записываем новое значение битов
  value |= ((bits & ADS1256_ADCON_CLKMASK) << ADS1256_ADCON_CLK);
  
  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_ADCON, value);             //Записываю обратно в регистр новое значение (с обновленными битами)

  ADS1256_CS_OFF();                                                           //Отключить передачу
}


//Возвращает логический результат: TRUE если транслятор тактовой частоты активен (DIO0=CLKOUT); FALSE отключен (DIO0 обычный GPIO).
extern __inline uint8_t ADS1256_API_IfClockOutOn(void)
{
  return (ADS1256_API_GetClockOutMode() != ADS1256_ADCON_CLK_OFF);
}




//-------------------------------------
// Настройка "Источников тока, используемых для определения наличия Датчика, подключенного ко входу АЦП"
// SDCS = Sensor Detection Current Sources
//
// Принцип действия здесь таков: если есть просадка напряжения - значит подключенный Датчик наличествует. 
//  И можно определить даже сопротивление датчика, по наводимому напряжению, от известного значения эмитируемого тока (поэтому номирал тока настраивается).
//  Т.е. включаешь эмиссию тока - грубо замеряешь напряжение АЦП - делаешь вывод... А затем, выключаешь "источник тока" и уже снимаешь "рабочие замеры" с Датчика.
//

uint8_t ADS1256_API_GetSensorDetectionCurrentSources(void)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return ADS1256_STATUS_DATAC_ERROR;

  ADS1256_CS_ON();                                                            //Разрешить передачу

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_ADCON);   //Считываю значение из регистра
  value = (value >> ADS1256_ADCON_SDCS) & ADS1256_ADCON_SDCSMASK;             //Выделяю нужные биты

  ADS1256_CS_OFF();                                                           //Отключить передачу

  return value;                                                               //Возвращаю значение битов
}


void ADS1256_API_SetSensorDetectionCurrentSources(uint8_t bits)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  ADS1256_CS_ON();                                                            //Разрешить передачу

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_ADCON);   //Считываю старое значение из регистра
  
  // Сначала очищаем нужную область битов, по маске
  value &= ~(ADS1256_ADCON_SDCSMASK << ADS1256_ADCON_SDCS);
  
  // Затем записываем новое значение битов
  value |= ((bits & ADS1256_ADCON_SDCSMASK) << ADS1256_ADCON_SDCS);
  
  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_ADCON, value);             //Записываю обратно в регистр новое значение (с обновленными битами)

  ADS1256_CS_OFF();                                                           //Отключить передачу
}


//Возвращает логический результат: TRUE если эмиссия тока включена; FALSE детектор отключен (нормальный режим).
extern __inline uint8_t ADS1256_API_IfSensorDetectionCurrentSourcesOn(void)
{
  return (ADS1256_API_GetSensorDetectionCurrentSources() != ADS1256_ADCON_SDCS_OFF);
}




//-------------------------------------
// Настройка "Усилителя с Программируемым коэффициентом" (PGA) для входного сигнала
// PGA = Programmable Gain Amplifier Setting

uint8_t ADS1256_API_GetProgrammableGainAmplifierMode(void)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return ADS1256_STATUS_DATAC_ERROR;

  ADS1256_CS_ON();                                                            //Разрешить передачу

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_ADCON);   //Считываю значение из регистра
  value = (value >> ADS1256_ADCON_PGA) & ADS1256_ADCON_PGAMASK;               //Выделяю нужные биты

  ADS1256_CS_OFF();                                                           //Отключить передачу

  return value;                                                               //Возвращаю значение битов
}  


void ADS1256_API_SetProgrammableGainAmplifierMode(uint8_t bits)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  ADS1256_CS_ON();                                                            //Разрешить передачу

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_ADCON);   //Считываю старое значение из регистра
  
  // Сначала очищаем нужную область битов, по маске
  value &= ~(ADS1256_ADCON_PGAMASK << ADS1256_ADCON_PGA);
  
  // Затем записываем новое значение битов
  value |= ((bits & ADS1256_ADCON_PGAMASK) << ADS1256_ADCON_PGA);
  
  // 1) Перенастроить режим
  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_ADCON, value);             //Записываю обратно в регистр новое значение (с обновленными битами)

  // 2) Запустить автокалибровку (это нужно, поскольку переключается Аналоговый Предусилитель входного сигнала)
  ADS1256_Command_SelfCalibration();
  
  // 3) Форсировать немедленную конвертацию (чтобы актуализировать данные в регистре результата)
  ADS1256_COMMAND_CONVERT();
  
  ADS1256_CS_OFF();                                                           //Отключить передачу
}  




//-------------------------------------
// Настройка портов ввода-вывода (Регистр I/O: GPIO)
//-------------------------------------

// TODO: в этом регистре ничего важного не настраивается, поэтому пока отложил реализацию кода "Настройки режима портов GPIO"...
// Замечу: по умолчанию, все порты GPIO настроены на Input, поэтому влияния не оказывают.




//-------------------------------------
// Настройка частоты семплирования АЦП (количество замеров в секунду) (Регистр DRATE)
//-------------------------------------

//-------------------------------------
// Вспомогательная функция: возвращает наиболее близкий поддерживаемый Режим по требуемой Частоте семплирования (конвертирует SPS -> DRATE)
uint8_t ADS1256_Convert_SPS2DRATE(uint16_t SPS)
{
  if      (SPS <= 3)
    // Замечу: здесь, на самом деле, следует использовать дробное число = 2.5 SPS ... Но я не использую здесь дробных (избегаю арифметики с плавающей точкой), да и не важно это все - поэтому округлено до целых!
    return ADS1256_DRATE_2_5SPS;
  else if (SPS <= 5)
    return ADS1256_DRATE_5SPS;
  else if (SPS <= 10)
    return ADS1256_DRATE_10SPS;
  else if (SPS <= 15)
    return ADS1256_DRATE_15SPS;
  else if (SPS <= 25)
    return ADS1256_DRATE_25SPS;
  else if (SPS <= 30)
    return ADS1256_DRATE_30SPS;
  else if (SPS <= 50)
    return ADS1256_DRATE_50SPS;
  else if (SPS <= 60)
    return ADS1256_DRATE_60SPS;
  else if (SPS <= 100)
    return ADS1256_DRATE_100SPS;
  else if (SPS <= 500)
    return ADS1256_DRATE_500SPS;
  else if (SPS <= 1000)
    return ADS1256_DRATE_1000SPS;
  else if (SPS <= 2000)
    return ADS1256_DRATE_2000SPS;
  else if (SPS <= 3750)
    return ADS1256_DRATE_3750SPS;
  else if (SPS <= 7500)
    return ADS1256_DRATE_7500SPS;
  else if (SPS <= 15000)
    return ADS1256_DRATE_15000SPS;
  else
    return ADS1256_DRATE_30000SPS;
}


//-------------------------------------
// Вспомогательная функция: разшифровывает Частоту семплирования из внутренней кодировки значение Регистра (конвертирует DRATE -> SPS)
uint16_t ADS1256_Convert_DRATE2SPS(uint8_t DRATE)
{
  switch(DRATE)
  {
    case ADS1256_DRATE_30000SPS:
      return 30000;
    case ADS1256_DRATE_15000SPS:
      return 15000;
    case ADS1256_DRATE_7500SPS:
      return 7500;
    case ADS1256_DRATE_3750SPS:
      return 3750;
    case ADS1256_DRATE_2000SPS:
      return 2000;
    case ADS1256_DRATE_1000SPS:
      return 1000;
    case ADS1256_DRATE_500SPS:
      return 500;
    case ADS1256_DRATE_100SPS:
      return 100;
    case ADS1256_DRATE_60SPS:
      return 60;
    case ADS1256_DRATE_50SPS:
      return 50;
    case ADS1256_DRATE_30SPS:
      return 30;
    case ADS1256_DRATE_25SPS:
      return 25;
    case ADS1256_DRATE_15SPS:
      return 15;
    case ADS1256_DRATE_10SPS:
      return 10;
    case ADS1256_DRATE_5SPS:
      return 5;
    case ADS1256_DRATE_2_5SPS:
      // Замечу: здесь, на самом деле, следует использовать дробное число = 2.5 SPS ... Но я не использую здесь дробных (избегаю арифметики с плавающей точкой), да и не важно это все - поэтому округлено до целых!
      return 3;
    default:
      // На мутные вопросы - отвечаем стандартной "максимально допустимой для данной аппаратной конфигурации" частотой
      return ADS1256_F_DATA;
  }
}



//-------------------------------------
// Базовая функциональная последовательность:
// 1) Перенастроить регистр DRATE на новую частоту семплирования
// 2) Запустить автокалибровку (это нужно, поскольку переключается внутренних цифровой фильтр помех).
// 3) Форсировать немедленную конвертацию (чтобы актуализировать данные в регистре результата)
void ADS1256_API_SetDataRateMode(uint8_t DRATE)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  // Проверка: нельзя устанавливать частоту выше Максимально допустимой для данной аппаратной конфигурации (ограничение сверху)
  if(ADS1256_Convert_DRATE2SPS(DRATE) > ADS1256_F_DATA) 
    return;

  ADS1256_CS_ON();                                                            //Разрешить передачу

  // 1) Перенастроить регистр DRATE на новую частоту семплирования
  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_DRATE, DRATE);

  // 2) Запустить автокалибровку (это нужно, поскольку переключается внутренний Цифровой Фильтр помех)
  ADS1256_Command_SelfCalibration();
  
  // 3) Форсировать немедленную конвертацию (чтобы актуализировать данные в регистре результата)
  ADS1256_COMMAND_CONVERT();

  ADS1256_CS_OFF();                                                           //Отключить передачу
}


//-------------------------------------
// Интерфейсные вспомогательные функции: 

// Установить новую частоту семплирования в SPS = [2.5 .. 30000]
//  Примечание: АЦП поддерживает не любую скорость, а некий набор дискретных значений, получаемых внутренним делителем частоты... И для однозначного результата установки, 
//  Рекомендуется указывать значения из стандартного ряда: SPS = 30000, 15000, 7500, 3750, 2000, 1000, 500, 100, 60, 50, 30, 25, 15, 10, 5, 3 (2.5)
extern __inline void ADS1256_API_SetDataRateModeSPS(uint16_t SPS)
{
  ADS1256_API_SetDataRateMode( ADS1256_Convert_SPS2DRATE(SPS) );
}


// Возвращает текущую частоту семплирования (в единицах SPS)
extern __inline uint16_t ADS1256_API_GetDataRateModeSPS(void)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return ADS1256_STATUS_DATAC_ERROR;

  ADS1256_CS_ON();                                                            //Разрешить передачу

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_DRATE);   //Считываю значение из регистра

  ADS1256_CS_OFF();                                                           //Отключить передачу

  return ADS1256_Convert_DRATE2SPS(value);                                    //Конвертирую тип данных и Возвращаю результат
}




//-------------------------------------
// Настройка регистров калибровочных коэффициентов OFCx и FSCx
//-------------------------------------

// CALIBRATION
//  Offset and gain errors can be minimized using the ADS1255/6 onboard calibration circuitry. 
//  Offset errors are corrected with the Offset Calibration (OFC) register and, likewise,
//  full-scale errors are corrected with the Full-Scale Calibration (FSC) register.


//-------------------------------------
// Считывает значение регистров OFC (Offset Calibration Coefficient)
uint32_t ADS1256_API_GetOffsetCalibrationCoefficient(void)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return ADS1256_STATUS_DATAC_ERROR;

  ADS1256_CS_ON();                                                                    //Разрешить передачу

  uint32_t result = 0;
  result  |= ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_OFC2);                 //Считываю значение из регистра OFC2 (старший)
  result <<= 8;                                                                       //После каждого полученного байта, задвигаем очередные 8-бит в старшие разряды
  result  |= ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_OFC1);                 //Считываю значение из регистра OFC1
  result <<= 8;                                                                       //После каждого полученного байта, задвигаем очередные 8-бит в старшие разряды
  result  |= ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_OFC0);                 //Считываю значение из регистра OFC0 (младший)

  ADS1256_CS_OFF();                                                                   //Отключить передачу

  return result;
}


//-------------------------------------
// Записывает значение в регистры OFC (Offset Calibration Coefficient)
void ADS1256_API_SetOffsetCalibrationCoefficient(uint32_t value)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  ADS1256_CS_ON();                                                                    //Разрешить передачу

  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_OFC0, (uint8_t) (value & 0xFF));   //Записать значение в регистр OFC0 (младший)
  value >>= 8;                                                                        //После каждой итерации, выдвигаем очередной байт из старших порядков на позицию младих 8-бит
  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_OFC1, (uint8_t) (value & 0xFF));   //Записать значение в регистр OFC1
  value >>= 8;                                                                        //После каждой итерации, выдвигаем очередной байт из старших порядков на позицию младих 8-бит
  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_OFC2, (uint8_t) (value & 0xFF));   //Записать значение в регистр OFC2 (старший)

  ADS1256_CS_OFF();                                                                   //Отключить передачу
}



//-------------------------------------
// Считывает значение регистров FSC (Fullscale Callibration Coefficient)
uint32_t ADS1256_API_GetFullscaleCallibrationCoefficient(void)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return ADS1256_STATUS_DATAC_ERROR;

  ADS1256_CS_ON();                                                                    //Разрешить передачу

  uint32_t result = 0;
  result  |= ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_FSC2);                 //Считываю значение из регистра FSC2 (старший)
  result <<= 8;                                                                       //После каждого полученного байта, задвигаем очередные 8-бит в старшие разряды
  result  |= ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_FSC1);                 //Считываю значение из регистра FSC1
  result <<= 8;                                                                       //После каждого полученного байта, задвигаем очередные 8-бит в старшие разряды
  result  |= ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_FSC0);                 //Считываю значение из регистра FSC0 (младший)

  ADS1256_CS_OFF();                                                                   //Отключить передачу

  return result;
}


//-------------------------------------
// Записывает значение в регистры FSC (Fullscale Callibration Coefficient)
void ADS1256_API_SetFullscaleCallibrationCoefficient(uint32_t value)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  ADS1256_CS_ON();                                                                    //Разрешить передачу

  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_FSC0, (uint8_t) (value & 0xFF));   //Записать значение в регистр FSC0 (младший)
  value >>= 8;                                                                        //После каждой итерации, выдвигаем очередной байт из старших порядков на позицию младих 8-бит
  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_FSC1, (uint8_t) (value & 0xFF));   //Записать значение в регистр FSC1
  value >>= 8;                                                                        //После каждой итерации, выдвигаем очередной байт из старших порядков на позицию младих 8-бит
  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_FSC2, (uint8_t) (value & 0xFF));   //Записать значение в регистр FSC2 (старший)

  ADS1256_CS_OFF();                                                                   //Отключить передачу
}


//-------------------------------------
// Запустить автокалибровку 
// (*) Note: после изменения режимов BUFEN, PGA, DRATE - настоятельно рекомендуется запускать "Self Calibration"...
void ADS1256_API_DoSelfCalibration(void)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  ADS1256_CS_ON();                                                            //Разрешить передачу

  // 1) Запустить автокалибровку (это нужно, поскольку переключается внутренний Цифровой Фильтр помех)
  ADS1256_Command_SelfCalibration();
  
  // 2) Форсировать немедленную конвертацию (чтобы актуализировать данные в регистре результата)
  ADS1256_COMMAND_CONVERT();

  ADS1256_CS_OFF();                                                           //Отключить передачу
}




//============================================================================
// Служебные методы
//============================================================================

//-------------------------------------
// Инициализация АЦП
void ADS1256_Init(void)
{
  //HAL_Delay(100);
  //ADS1256_LOG_DUMPREGISTERS("ADS1256_Init: перед инициализацией АЦП");
  //HAL_Delay(1000);
  

  // Команда RESET: Reset Registers to Default Values
  ADS1256_API_Reset();

  //ADS1256_LOG_DUMPREGISTERS("ADS1256_Init: регистры АЦП после RESET");
  //HAL_Delay(1000);
  
  
  // Установить Порядок БИТОВ в ответе "данных семплирования" (бит ORDER в регистре STATUS) = MSB (по-умолчанию)
  ADS1256_API_SetBitOrderMode( ADS1256_STATUS_ORDER_MSB );

  // Настройка "Транслятора тактовой Частоты на Выход" (для тактирования внешней периферии) (биты CLKx регистра ADCON)
  //  Рекомендуется отключить, если он не используется...
  ADS1256_API_SetClockOutMode( ADS1256_ADCON_CLK_OFF );
  
  // Настройка мультиплексора входных каналов (Регистр MUX)
  ADS1256_API_SetMultiplexerSingleP(0);   //номер канала AINPx = [0..7]

  
  // (*) Включить Входной Повторитель (увеличивает импеданс до 10..80МОм, но уменьшает динамический диапазон до 0..3V) (бит BUFEN в регистре STATUS)
  ADS1256_API_SetInputBufferMode( ADS1256_STATUS_BUFEN_ON );              //DEBUG: нужно включить в реальном изделии
  
  // (*) Настройка "Усилителя с Программируемым коэффициентом" для входного сигнала (биты PGAx регистра ADCON)
  ADS1256_API_SetProgrammableGainAmplifierMode( ADS1256_ADCON_PGA_2 );    //DEBUG: нужно включить в реальном изделии

  // (*) Настройка частоты семплирования АЦП (количество замеров в секунду) (Регистр DRATE)
  //  Установить значение регистра DRATE, для соответствующей скорости семплирования SPS = 30000, 15000, 7500, 3750, 2000, 1000, 500, 100, 60, 50, 30, 25, 15, 10, 5, 2.5
  ADS1256_API_SetDataRateModeSPS( ADS1256_F_DATA );
  //ADS1256_API_SetDataRateModeSPS( 2000 );  //DEBUG
  //ADS1256_API_SetDataRateModeSPS( 15 );   //DEBUG: при Скользящем Окне = 16, имеет ~1 SPS (очно наблюдаем, что будет на 1мс)

  // (*) Примечание: при каждой перенастройки режима (BUFEN,PGA,SPS) - в коде соответствующих API функций, также, неявно запускается "автокалибровка"...
  ADS1256_API_DoSelfCalibration();    //(хотя, это уже и не требуется)
  
  // Включить режим "автоматического запуска калибровки после каждой перенастройки режима АЦП" (бит ACAL в регистре STATUS)
  //  Замечу: это полезно только на случай, если программирование регистров АЦП будет осуществляться еще каким-то способом помимо API функций этого драйвера.
  //  Потому что соответствующие API-функции (*) имеют в коде неявный запуск "процедуры автокалибровки"...
  ADS1256_API_SetAutoCalibrationMode( ADS1256_STATUS_ACAL_ON );


  //HAL_Delay(100);
  //ADS1256_LOG_DUMPREGISTERS("ADS1256_Init: после инициализации АЦП");
  //HAL_Delay(1000);
}



//-------------------------------------
// DEBUG: Тестирование АЦП
void ADS1256_Test(void)
{
  HAL_Delay(100);
  ADS1256_LOG_DUMPREGISTERS("ADS1256_Test");
  HAL_Delay(1000);
}




//============================================================================
// Единичные измерения: Синхронное чтение Данных конвертации АЦП
//============================================================================


//-------------------------------------
// Чтение из регистров результата последнего преобразования (низкоуровневый метод)
int32_t ADS1256_Command_ReadData(void)
{
  // Синхронная задержка до готовности АЦП
  while(ADS1256_DRDY_BUSY());
  // Важно: здесь, явно убеждаемся, что данные ОЧЕРЕДНОГО ЗАМЕРА готовы.
  // Примечание: в datasheet есть рецепт максимально быстрой скорости семплирования нескольких каналов - 
  //  при котором можно быстро переключать каналы, и пока измеряется/готовится результат по новому каналу (пока ~DRDY=1), 
  //  то технически можно успеть шустро (за время <T18, см. Table 13, page 20) выгрести результат ПРЕДЫДУЩЕГО ЗАМЕРА (пока он не был перебит новым замером)... 
  //  Но мы не будем играть в эти гонки!
  
  
  // Буферы
  uint8_t TxBuffer;                                                                         //Buffer dlya otpravlyaemih dannih
  uint8_t RxBuffer;                                                                         //Buffer dlya prinimaemih dannih

  // Запрос
  TxBuffer = ADS1256_COMMAND_RDATA;                                                         //Команда "Чтение данных одиночное"
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));  

  // Обязательная задержка, пока микросхема АЦП подготовит ответ
  delay_us(ADS1256_DELAY_T6_US);
  
  
  // Приемник результата - ЗНАКОВОЕ целое! Примечание: битовыми операциями вполне нормально загружается/формируется значение в этой ячейке, по частям.
  int32_t value;

  // Ответ
  value = 0;
  for(uint8_t i = 0; i < 3; i++)                                                            //Цикл по трем байтам результата:
  {
    value <<= 8;                                                                            //Задвигаем предыдущие байты в старшие разряды.
    RxBuffer = 0;
    assert_param(HAL_OK == HAL_SPI_Receive(&SPI_ADC_HANDLE, &RxBuffer, 1, HAL_IO_TIMEOUT)); //Получаем очередной байт...
    value |= RxBuffer;                                                                      //Записываем его в позицию младшего байта.
  }
  
  // Коррекция: для отрицательных чисел, которые хранятся в "дополнительном коде" - старший (четвертый) байт должен быть заполнен "единицами". 
  // Критерием отрицательного числа является: единица в старшем бите, старшего (третьего) байта - в данном случае, 23й бит.
  if(value & (1 << 23))
    value |= 0xFF000000;
  
  
  // Обязательная "задержка между командами", пока АЦП восстановится после только что принятой команды
  delay_us(ADS1256_DELAY_T11_US);
  
  return value;
}



//-------------------------------------
// Произвести одиночное преобразование немедленно (синхронно и долго: до 2/SPS сек)
int32_t ADS1256_API_ConvertDataOnce(void)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return ADS1256_STATUS_DATAC_ERROR;

  ADS1256_CS_ON();                                                                          //Разрешить передачу
  
  // Форсируем немедленный замер
  ADS1256_COMMAND_CONVERT();
  
  // Получаем результат
  int32_t result = ADS1256_Command_ReadData();
  
  ADS1256_CS_OFF();                                                                         //Отключить передачу

  return result;
}



//-------------------------------------
// Вернуть результат последней конвертации, не производя нового замера (относительно быстро)
int32_t ADS1256_API_ReadLastData(void)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  if(ADS1256_API_IfDataContinuousMode())
    return ADS1256_STATUS_DATAC_ERROR;

  ADS1256_CS_ON();                                                                          //Разрешить передачу
  
  // Получаем результат
  int32_t result = ADS1256_Command_ReadData();
  
  ADS1256_CS_OFF();                                                                         //Отключить передачу

  return result;
}




//============================================================================
// Потоковая конвертация: Асинхронное и неблокирующее чтение Данных конвертации АЦП, с помощью Обработчиков Прерываний
//============================================================================


// Флаг "Режим потоковой конвертации сейчас включен"
volatile static uint8_t ADS1256_DATAC_Active = 0;

// Флаг "Запрос на остановку Режима потоковой конвертации" 
//  (устанавливается асинхронно API-методом ADS1256_API_StopDATAC
//  и далее обрабатывается флаговым автоматом в обработчиках прерывания)
volatile static uint8_t ADS1256_DATAC_RequestToStop = 0;


// Вспомогательные переменные для получения данных от АЦП, через Обработчики прерываний  (вынесены в глобальное пространство имен, чтобы быть доступны из разных функций)
#define         ADS1256_DATA_FRAME_SIZE         3                           /* число байт в посылке данных АЦП (3 байта = 24 бита) */
volatile static uint8_t ADS1256_DATAC_TxBuffer[ ADS1256_DATA_FRAME_SIZE ];  // Буфер передачи
volatile static uint8_t ADS1256_DATAC_RxBuffer[ ADS1256_DATA_FRAME_SIZE ];  // Буфер приема


// Указатель на callback-функцию для запоминания/учёта очередного результата конвертации АЦП (используется в асинхронном режиме)
//  Важно: данная функция должна быть неблокирующей и очень лёгкой, поскольку запускается из обработчика прерываания!
volatile static TDataRegistrator ADS1256_DataRegistrator = 0;


// Для режима "потоковой конвертации" (DATAC): 
//  если SPI зависнет? или из-за ошибки с таймингами (слишком быстрая частота семплирования, но медленный SPI)
//  новый семпл будет готов раньше (спад сигнала DRDY прийдет раньше), чем будет вычитан предыдущий результат конвертации?
//  то SPI-модуль будет сброшен, чтобы программа тупо не зависла!
//
// Автовосстановление SPI, в режиме DATAC: Количество спадов сигнала DRDY, которое пропускаем, прежде чем принудительно сбросить SPI:
//  0 = функция отключена
//  1 = сбрасываем сразу же как только пришел очередной DRDY, совсем не терпим зависания SPI (не рекомендуется)
//  2 = сбрасываем на второй раз, если SPI не восстановился сам за два Периода... (рекомендуется)
#define  ADS1256_SPI_DATAC_TIMEOUT_COUNT   2



//-------------------------------------
// Получить указатель на текущий Регистратор (полезно для отладочных целей или полиморфных реализаций - подвязать Wrapper...)
extern __inline TDataRegistrator ADS1256_API_GetDataRegistrator(void)
{
  return ADS1256_DataRegistrator;
}



//-------------------------------------
// Подключить callback-функцию для запоминания/учёта очередного результата конвертации АЦП  (Регистратор вызывается из "обработчика прерывания SPI" по завершении выборки очередного результата конвертации)
extern __inline void ADS1256_API_SetDataRegistrator(TDataRegistrator pFunc)
{
  ADS1256_DataRegistrator = pFunc;
}



//-------------------------------------
// Послать команду АЦП "RDATAC: Read Data Continuous" (низкоуровневый метод)
void ADS1256_Command_RunDataContinuousMode(void)
{
  // Синхронная задержка до готовности АЦП
  while(ADS1256_DRDY_BUSY());
  
  // Буферы
  uint8_t TxBuffer;                                                                         //Buffer dlya otpravlyaemih dannih
  // Запрос
  TxBuffer = ADS1256_COMMAND_RDATAC;                                                        //Команда "Чтение данных одиночное"
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));
  
  // Обязательная "задержка между командами", пока АЦП восстановится после только что принятой команды
  delay_us(ADS1256_DELAY_T11_US);
}



//-------------------------------------
// Активировать режим "RDATAC: Read Data Continuous" СИНХРОННО
//  (Внимание: после запуска этого метода никакие другие API-функции данного драйвера АЦП вызывать нельзя! Сперва, нужно вызвать метод ADS1256_API_StopDataContinuousMode[Synchronous], для остановки потоковой конвертации...)
void ADS1256_API_RunDataContinuousMode(void)
{
  // Проверка: если АЦП находится в режиме "потоковой конвертации", то он неспособен принимать Команды!
  //  (Также, исключаем глюк "повторный запуск функции"...)
  if(ADS1256_API_IfDataContinuousMode())
    return;
  
  // Проверка: Нельзя использовать режим "потоковой конвертации" в полудуплексном режиме SPI, т.е. в режиме "1-WIRE SPI" (BIDIMODE=1)!
  //  Datasheet (page 29): "Avoid using the continuous read mode (RDATAC) when DIN and DOUT are connected together."
  if(SPI_ADC_HANDLE.Init.Direction != SPI_DIRECTION_2LINES)
    return;


  // Послать команду АЦП для фактического включения режима
  ADS1256_CS_ON();                                                                          //Разрешить передачу
  ADS1256_Command_RunDataContinuousMode();
  ADS1256_CS_OFF();                                                                         //Отключить передачу

  // Установить флаг
  ADS1256_DATAC_RequestToStop = 0;  //Важно: этот Флаг следует снять ДО установки флага ADS1256_DATAC_Active - это поможет избежать гонок в ADS1256_API_StopDataContinuousMode().
  ADS1256_DATAC_Active = 1;         //Важно: этот Флаг обязательно устанавливать только УЖЕ ПОСЛЕ подачи команды RDATAC, поскольку Флаг включает Обработчики прерываний, которые блокируют собственно отправку Команды...
}



//-------------------------------------
// Послать сигнал остановки режима "SDATAC: Stop Read Data Continuous" АСИНХРОННО
//  (Внимание: управление вернется сразу же - еще до фактической остановки режима SDATAC!)
//  (код реентерабельный и асинхронный)
void ADS1256_API_StopDataContinuousMode(void)
{
  // Проверка: отрабатываем только если АЦП уже находится в режиме "потоковой конвертации"
  //  (Важно: нельзя посылать сигнал остановленному автомату - иначе, сигнал зависнет и не сбросится до следующего запуска автомата, и тогда тут же остановит автомат - глюк!)
  if(ADS1256_API_IfDataContinuousMode())
  {
    ADS1256_DATAC_RequestToStop = 1;
    
    // Возможная гонка сигналов (очень редкий случай):
    //  Флаг ADS1256_DATAC_RequestToStop мог быть установлен и ранее...
    //  И в момент после срабатывания условия If но до установки Флага Сигнала - если сработало прерывание SPI и остановило Автомат?

    // Перепроверить: если Автомат уже остановился, то снимем сигнал.
    if(!ADS1256_API_IfDataContinuousMode())
      ADS1256_DATAC_RequestToStop = 0;
  }
}



//-------------------------------------
// Остановить режим "SDATAC: Stop Read Data Continuous" СИНХРОННО
//  (Примечание: После выполнения этой функции, АЦП вновь станет доступным к управлению!)
void ADS1256_API_StopDataContinuousModeSynchronous(void)
{
  // Послать сигнал остановки АЦП
  ADS1256_API_StopDataContinuousMode();

  // Синхронная задержка до фактической остановки АЦП
  while(ADS1256_API_IfDataContinuousMode());
}



//-------------------------------------
// Возвращает логический результат: TRUE если режим "потоковой конвертации" сейчас включен; FALSE режим отключен (АЦП свободно для конфигурации)
extern __inline uint8_t ADS1256_API_IfDataContinuousMode(void)
{
  return (ADS1256_DATAC_Active != 0);
}



//-------------------------------------
// Обработчик прерывания EXTI порта GPIO, обслуживающего физическую шину DRDY, и настроенного на срабатывание "по спаду" сигнала!
//  (Функция запускает SPI для вычитки очередного результата конвертации)
extern __inline void ADS1256_INTERRUPT_OnDRDY(uint16_t GPIO_Pin)
{
  // Проверка: код обработчика отрабатывает только в режиме "потоковой конвертации" АЦП
  //  Важно: Причем очередное прерывание EXTI обслуживается только тогда, когда очередная асинхронная передача команды/данных по SPI уже завершена и Шина освободилась!
  if( ADS1256_API_IfDataContinuousMode() && GPIO_Pin == ADS1256_DRDY_PIN )
  {  

    #if defined(ADS1256_SPI_DATAC_TIMEOUT_COUNT) && (ADS1256_SPI_DATAC_TIMEOUT_COUNT != 0)
    // Проверка: если предыдущая операция чтения SPI все еще не завершилась? Значит, что-то пошло не так...
    if( !(SPI_ADC_HANDLE.State == HAL_SPI_STATE_READY  &&
          SPI_ADC_HANDLE.Lock  == HAL_UNLOCKED         ))
    {
      // Исключение: SPI не успел еще вычитать предыдущий сэмпл! 
      //  Значит, либо неправильно сконфигурирован АЦП "Скорость семплирования" > "Скорости SPI"
      //  Либо произошел сбой с самим модулем SPI и он завис (у меня такое было, когда я останавливал программу Отладчиком посередине коммуникации - когда дебажил "Обработчики прерываний"...)
      
      volatile static uint8_t AttemptsCounter = 0;

      AttemptsCounter++;
      if(AttemptsCounter < ADS1256_SPI_DATAC_TIMEOUT_COUNT)
        return;

      AttemptsCounter = 0;

      
      #ifdef  USE_FULL_ASSERT
      printf("DEBUG: Исключение в ADS1256_INTERRUPT_OnDRDY() Надо читать данные, а SPI не готов - сбрасываю...\r\n");
      #endif 

      // Чтобы программа тупо не зависала - сделаем костыль, который по таймауту будет перезапускать SPI
      assert_param(HAL_OK == HAL_SPI_Abort_IT(&SPI_ADC_HANDLE));    //Abort ongoing transfer (Interrupt mode)
      
      // Важно: Отключить передачу по SPI  (чтобы строббировать следующий пакет - вдруг, АЦП синхру сорвало?)
      ADS1256_CS_OFF();

      // И на этом шаге уже не пытаемся говорить с АЦП - дадим таймаут по шине CS...
      return;
    }
    #endif    
    
    
    // (Состояние: все ок! данные готовы, SPI готов... Инициируем вычитку очередного пакета данных.)
    
    // Буфер передачи инициализируется в зависимости от наличия команды на Остановку:
    if(ADS1256_DATAC_RequestToStop != 0)
    {
      // Запрошена Остановка Автомата - в Буфер передачи записывается команда Останова...
      for(int i=0; i<ADS1256_DATA_FRAME_SIZE; i++)
        ADS1256_DATAC_TxBuffer[i] = ADS1256_COMMAND_SDATAC;    
    }
    else
    {
      // Иначе, в Буфер передачи записывается нейтральная команда 
      for(int i=0; i<ADS1256_DATA_FRAME_SIZE; i++)
        ADS1256_DATAC_TxBuffer[i] = 0;      //Примечание: 0 == ADS1256_COMMAND_WAKEUP0
    }
  
    
    // Важно: Разрешить передачу по шине SPI  (перед сессией коммуникации с АЦП)
    ADS1256_CS_ON();

    // Получить/передать пакет данных по SPI в асинхронном режиме (на прерываниях)
    assert_param(HAL_OK == HAL_SPI_TransmitReceive_IT(&SPI_ADC_HANDLE, 
                                           (uint8_t*) ADS1256_DATAC_TxBuffer, 
                                           (uint8_t*) ADS1256_DATAC_RxBuffer, 
                                                      ADS1256_DATA_FRAME_SIZE));
  }
}



//-------------------------------------
// Обработчик прерывания SPI, который запускается ПОСЛЕ ПРИНЯТИЯ ВСЕГО ПАКЕТА данных (3 байт).
//  Примечание: данная функция автоматически слинкуется как CallBack в HAL-драйвере SPI (вручную никуда не требуется "подключать").
extern __inline void ADS1256_INTERRUPT_OnSPI(SPI_HandleTypeDef *hspi)
{
  // Пояснение: прерывание SPI "принят очередной байт" удобнее - поскольку оно возникает, когда прием байта уже действительно завершен, и данные перенесены из регистра в буфер в ОЗУ.
  //  В то время как, прерывание SPI для "передачи данных" сложнее - оно возникает, когда очередной байт был только был перемещен из буфера в аппаратный регистр, но передача все еще идет (это событие, по факту, означает: "буфер пуст, можно засылать очередные данные")...
  //  Однако, ВЫСОКОУРОВНЕВОЕ СОБЫТИЕ "передача данных полностью завершена", когда вызывается обработчик HAL_SPI_TxRxCpltCallback() - сформировано уже Драйвером HAL, потому оно не зависит от особенностей SPI модуля...
  
  
  // Проверка: код обработчика отрабатывает только в режиме "потоковой конвертации" АЦП,
  //  и только для аппаратного модуля SPI, обслуживающего АЦП (если их несколько в микроконтроллере).
  if(ADS1256_API_IfDataContinuousMode() &&
     hspi == &SPI_ADC_HANDLE)
  {
    // (Состояние: новые данные находятся в "Буфере приема")
    
    // Сборка принятого числа
    int32_t value = ADS1256_DATAC_RxBuffer[0]<<16 |   //старший байт (MSB) приходит первым
                    ADS1256_DATAC_RxBuffer[1]<< 8 |   //средний байт
                    ADS1256_DATAC_RxBuffer[2];        //младший байт (LSB) последний
    
    // Коррекция: для отрицательных чисел, которые хранятся в "дополнительном коде" - старший (четвертый) байт должен быть заполнен "единицами". 
    // Критерием отрицательного числа является: единица в старшем бите, старшего (третьего) байта - в данном случае, 23й бит.
    if(value & (1 << 23))
      value |= 0xFF000000;
    
    // Если подключен "регистратор данных", то отправить ему полученные данные  (иначе, данные потеряются)
    if(ADS1256_DataRegistrator)
      (*ADS1256_DataRegistrator)(value);    
    
    
    // Проверка: Если запрошена Остановка Автомата?
    //  Замечу: здесь мы не будем использовать проверку Флага ADS1256_DATAC_RequestToStop!=0, во избежание гонок сигналов. 
    //  В то же время, если буфер передачи был проинициализирован командой "SDATAC", то АЦП уже гарантированно ее принял и остановился - это более надежное условие проверки...
    if(ADS1256_DATAC_TxBuffer[0] == ADS1256_COMMAND_SDATAC)
    {
      // Остановить автомат
      ADS1256_DATAC_Active = 0;
      // И снять сигнал "Запрос на остановку"
      ADS1256_DATAC_RequestToStop = 0;
    }
    
    
    // Важно: Отключить передачу по SPI  (по завершению асинхронной коммуникации с АЦП)
    ADS1256_CS_OFF();
  }
}


