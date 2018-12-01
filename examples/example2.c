/******************************************************************************
 * Модуль:      example2.c
 * Автор:       Celeron (c) 2018
 * Назначение:  Более продвинутый пример (кусок кода из реального проекта) - 
                реализация пользовательского интерфейса, демонстрирующего режимы работы АЦП...
 ******************************************************************************/

// Замечу: в этом коде, используются также вызовы методов из библиотек наработанного мною фреймворка (драйвер дисплея, кнопок и т.п.)
// а также, весь этот код исполняется в Потоке FreeRTOS.


//-------------------------------------
// Экран "Тест АЦП"

  volatile static TDataRegistrator ADS1256_TEST_OriginalDataRegistrator = 0;
  volatile static uint32_t         ADS1256_TEST_Counter = 0;

  // Добавить очередной Замер к Выборке (обертка, чтобы еще считать число замеров)
  void ADS1256_TEST_DataRegistratorWrapper(const int32_t value)
  {
    ADS1256_TEST_Counter++;
    
    // Если подключен "регистратор данных", то отправить ему полученные данные
    if(ADS1256_TEST_OriginalDataRegistrator)
      (*ADS1256_TEST_OriginalDataRegistrator)(value);   
  };


TControlMode Control_MODE_AdcTest(void)
{
  // При входе в новую подсистему - сбросить статусы Кнопок и счетчики Энкодеров
  keyResetStatusForAllButtons();
  
  // Остановить режим "SDATAC: Stop Read Data Continuous" СИНХРОННО
  //  (Примечание: После выполнения этой функции, АЦП вновь станет доступным к управлению!)
  ADS1256_API_StopDataContinuousModeSynchronous();

  static char s1[25] = {0};
  static char s2[25] = {0};
  static char s3[25] = {0};
  static char s4[25] = {0};

  ADS1256_TEST_Counter = 0;
  ADS1256_TEST_OriginalDataRegistrator = ADS1256_API_GetDataRegistrator();
  ADS1256_API_SetDataRegistrator( ADS1256_TEST_DataRegistratorWrapper );
  
  int32_t value   = 0; 
  int32_t average = 0;
  int32_t max     = INT32_MIN;
  int32_t min     = INT32_MAX;
  uint8_t mode    = 0;

  while(1)
  {
    
    // Выбор П.1: Переключить Режим теста
    if(BUTTON_HAVE_FLAG( BUTTON_MENU_PREV, BUTTON_IS_HOLDDOWN ))
    {
      BUTTON_RESET(BUTTON_MENU_PREV);                             // Обработал событие... Сбросить статус кнопки.

      // Режим теста: 
      //  "Одиночный замер / результат в Коде АЦП"; 
      //  "Одиночный замер / результат Конвертировать в реальные единицы"; 
      //  "Потоковая конвертация / результат в Коде АЦП"
      //  "Потоковая конвертация / результат Конвертировать в реальные единицы"
      mode = (mode+1)%4;

      if(mode&0x2)
      {
        // Сбросить "Скользящее окно" и статистику АЦП
        ADC_AVG_ResetArray();
        ADS1256_TEST_Counter = 0;
        max = INT32_MIN;
        min = INT32_MAX;
        
        // Активировать режим "RDATAC: Read Data Continuous"  (Замечу: в этом режиме, никакие API-функции драйвера АЦП вызывать нельзя! Сперва, нужно вызвать метод ADS1256_API_StopDataContinuousMode[Synchronous], для остановки потоковой конвертации...)
        ADS1256_API_RunDataContinuousMode();
      }
      else
      {
        // Остановить режим "SDATAC: Stop Read Data Continuous" СИНХРОННО
        //  (Примечание: После выполнения этой функции, АЦП вновь станет доступным к управлению!)
        ADS1256_API_StopDataContinuousModeSynchronous();

        // Сбросить "Скользящее окно" и статистику АЦП
        ADC_AVG_ResetArray();
        ADS1256_TEST_Counter = 0;
        max = INT32_MIN;
        min = INT32_MAX;
      }
    }

    // Выбор П.2: Обнулить Счетчики
    if(BUTTON_HAVE_FLAG( BUTTON_MENU_OK, BUTTON_IS_HOLDDOWN ))
    {
      BUTTON_RESET(BUTTON_MENU_OK);                           // Обработал событие... Сбросить статус кнопки.
      
      // Сбросить "Скользящее окно" и статистику АЦП
      ADC_AVG_ResetArray();
      ADS1256_TEST_Counter = 0;
      max = INT32_MIN;
      min = INT32_MAX;
    }
    
    // Выбор П.3: Выход из Теста
    if(BUTTON_HAVE_FLAG( BUTTON_MENU_NEXT, BUTTON_IS_HOLDDOWN ))
    {
      BUTTON_RESET(BUTTON_MENU_NEXT);                                       // Обработал событие... Сбросить статус кнопки.
      ADS1256_API_SetDataRegistrator(ADS1256_TEST_OriginalDataRegistrator); // Восстановить Регистратор данных, который был установлен до входа в Экран Теста
      return CONTROL_MODE_DEFAULT;                                          // Вернуть интерфейс на Экран, по-умолчанию
    }
    
    
    // В режиме "Одиночного запроса" требуется вручную прочитать Замер и передать его Регистратору
    if(!ADS1256_API_IfDataContinuousMode())
    {
      //value = ADS1256_API_ConvertDataOnce();
      value = ADS1256_API_ReadLastData();
      ADC_AVG_IncludeSample(value);           //Рекомендация: усреднять "Скользящим окном" лучше сырые данные, в Коде АЦП (так выше точность). А уже потом, обработанные показатели конвертировать в Реальные единицы (если требуется).
      ADS1256_TEST_Counter++;
    }
    else
      value = 0;
    
    // Статистическая обработка Результата
    average = ADC_AVG_GetMovingAverage();
    if(average)
    {
      if(max < average)
        max = average;      
      if(min > average)
        min = average;
    }
    ADC_AVG_CalcArrayStatistics();
    
    
    // Рендеринг дисплея
    #define _CNV(value)  ((mode&0x1)?ADC_CNV_ConvertCode2Real(value)     :(value))
    #define _CNVD(value) ((mode&0x1)?ADC_CNV_ConvertDeltaCode2Real(value):(value))
    
    if(ADS1256_API_IfDataContinuousMode())
      sprintf(s1, "NOW  _DATAC_ C%6d", ADS1256_TEST_Counter);
    else
      sprintf(s1, "NOW %8d C%6d", _CNV(value),   ADS1256_TEST_Counter);
    
    sprintf(s2, "AVG %8d S%6d", _CNV(average), _CNVD(ADC_AVG_GetArrayStdDev()));
    sprintf(s3, "History Window Avg.D");
    sprintf(s4, "%6d %6d %6d", _CNV(ADC_AVG_GetHistoricalMax()) - _CNV(ADC_AVG_GetHistoricalMin()),  //выражение эквивалентно: _CNVD(ADC_AVG_GetHistoricalMax() - ADC_AVG_GetHistoricalMin())
                               _CNV(ADC_AVG_GetArrayMax())      - _CNV(ADC_AVG_GetArrayMin())     ,
                               _CNV(max)                        - _CNV(min)                       );

    // Альтернативно, вывод "дельт" и "абсолютных" значений оценок:
    //sprintf(s3, "H%3d %7d %-7d", _CNVD(ADC_AVG_GetHistoricalMax() - ADC_AVG_GetHistoricalMin()), _CNV(ADC_AVG_GetHistoricalMin()), _CNV(ADC_AVG_GetHistoricalMax()));
    //sprintf(s4, "A%3d %7d %-7d", _CNVD(ADC_AVG_GetArrayMax()      - ADC_AVG_GetArrayMin()),      _CNV(ADC_AVG_GetArrayMin()),      _CNV(ADC_AVG_GetArrayMax()));
    #undef _CNV
    
    
    // Обновить дисплей
    Display_Show_Message( .Line1 = s1,
                          .Line2 = s2,
                          .Line3 = s3,
                          .Line4 = s4 );
    
    // Ожидание (задержка в суперцикле потока)
    osDelay(20);
  }
}


