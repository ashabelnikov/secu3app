
#ifndef _ADC_H_
#define _ADC_H_

#define ADC_DISCRETE            0.0025       //одна дискрета АЦП в вольтах

#define TSENS_SLOPP             0.01        //наклон прямой датчика температуры вольт/градус
#define TSENS_ZERO_POINT        2.73        //напряжение на выходе датчика температуры при 0 градусов цельсия

#define ADC_VREF_TYPE           0xC0

//номера используемых каналов АЦП
#define ADCI_MAP                2
#define ADCI_UBAT               1         
#define ADCI_TEMP               0
#define ADCI_KNOCK              3

#define MAP_PHYSICAL_MAGNITUDE_MULTIPLAYER  64
#define UBAT_PHYSICAL_MAGNITUDE_MULTIPLAYER (1.0/ADC_DISCRETE) //=400
#define TEMP_PHYSICAL_MAGNITUDE_MULTIPLAYER (TSENS_SLOPP / ADC_DISCRETE) //=4

#define MAP_CURVE_OFFSET_V      0.547  //Вольт
#define MAP_CURVE_GRADIENT_KPA  20.9   //кПа

//эти функции возвращают текущие значения из буферов усреднения
unsigned int adc_get_map_value(void);
unsigned int adc_get_ubat_value(void);
unsigned int adc_get_temp_value(void);

//запускает измерение значений с датчиков, но только если предыдущее  
//измерение завершено.
void adc_begin_measure(void);

//возвращает не 0 если измерение готово (АЦП не занято)
char adc_is_measure_ready(void); 

//инициализация АЦП и его переменных состояния
void adc_init(void);

signed int adc_compensate(signed int adcvalue, signed int factor, signed long correction);

//переводит значение АЦП в физическую величину - давление
//физическая величина * MAP_PHYSICAL_MAGNITUDE_MULTIPLAYER
unsigned int map_adc_to_kpa(signed int adcvalue);

//переводит значение АЦП в физическую величину - напряжение
//физическая величина * UBAT_PHYSICAL_MAGNITUDE_MULTIPLAYER
unsigned int ubat_adc_to_v(signed int adcvalue);

//переводит значение АЦП в физическую величину - температура
//физическая величина * TEMP_PHYSICAL_MAGNITUDE_MULTIPLAYER
signed int temp_adc_to_c(signed int adcvalue);


#define TEMPERATURE_MAGNITUDE(t) ((t) * TEMP_PHYSICAL_MAGNITUDE_MULTIPLAYER)
#define VOLTAGE_MAGNITUDE(t) ((t) * UBAT_PHYSICAL_MAGNITUDE_MULTIPLAYER)
#define PRESSURE_MAGNITUDE(t) ((t) * MAP_PHYSICAL_MAGNITUDE_MULTIPLAYER)

#endif //_ADC_H_
