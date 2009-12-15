
#ifndef _ADC_H_
#define _ADC_H_

#include <stdint.h>

#define ADC_DISCRETE            0.0025       //одна дискрета АЦП в вольтах

#define TSENS_SLOPP             0.01        //наклон прямой датчика температуры вольт/градус
#define TSENS_ZERO_POINT        2.73        //напряжение на выходе датчика температуры при 0 градусов цельсия

#define ADC_VREF_TYPE           0xC0

//номера используемых каналов АЦП
#define ADCI_MAP                2
#define ADCI_UBAT               1         
#define ADCI_TEMP               0
#define ADCI_KNOCK              3
#define ADCI_STUB               4  //заглушка, используется для ADCI_KNOCK

#define MAP_PHYSICAL_MAGNITUDE_MULTIPLAYER  64
#define UBAT_PHYSICAL_MAGNITUDE_MULTIPLAYER (1.0/ADC_DISCRETE) //=400
#define TEMP_PHYSICAL_MAGNITUDE_MULTIPLAYER (TSENS_SLOPP / ADC_DISCRETE) //=4

//эти функции возвращают текущие значения из буферов усреднения
uint16_t adc_get_map_value(void);
uint16_t adc_get_ubat_value(void);
uint16_t adc_get_temp_value(void);
uint16_t adc_get_knock_value(void);

//запускает измерение значений с датчиков, но только если предыдущее  
//измерение завершено.
void adc_begin_measure(void);
//запускает измерение значения с интегратора канала детонации. Так как после установки
//сигнала INT/HOLD в 0 выход INTOUT перейдет в полностью корректное состояние только через
//20мкс (приблизительно), а запуск измерения может быть произведен сразу, то делаем первое
//измерение холостым.
void adc_begin_measure_knock(void);

//запускает измерение значений с датчиков и сигнала с ДД. Первыми снимаются значения
//с датчиков, последним сигнал с ДД
void adc_begin_measure_all(void);

//возвращает не 0 если измерение готово (АЦП не занято)
uint8_t adc_is_measure_ready(void); 

//инициализация АЦП и его переменных состояния
void adc_init(void);

int16_t adc_compensate(int16_t adcvalue, int16_t factor, int32_t correction);

//переводит значение АЦП в физическую величину - давление
//физическая величина * MAP_PHYSICAL_MAGNITUDE_MULTIPLAYER
uint16_t map_adc_to_kpa(int16_t adcvalue, uint16_t offset, uint16_t gradient);

//переводит значение АЦП в физическую величину - напряжение
//физическая величина * UBAT_PHYSICAL_MAGNITUDE_MULTIPLAYER
uint16_t ubat_adc_to_v(int16_t adcvalue);

//переводит значение АЦП в физическую величину - температура
//физическая величина * TEMP_PHYSICAL_MAGNITUDE_MULTIPLAYER
int16_t temp_adc_to_c(int16_t adcvalue);

#endif //_ADC_H_
