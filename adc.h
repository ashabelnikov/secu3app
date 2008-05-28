
#ifndef _ADC_H_
#define _ADC_H_

#define ADC_DISCRETE            0.0025       //одна дискрета АЦП в вольтах

#define TSENS_SLOPP             0.01        //наклон прямой датчика температуры вольт/градус
#define TSENS_ZERO_POINT        2.73        //напряжение на выходе датчика температуры при 0 градусов цельсия

//переводит температуру из градусов Цельсия в дискреты АЦП
#define T_TO_DADC(Tc) ((unsigned int)((TSENS_ZERO_POINT + (Tc*TSENS_SLOPP))/ADC_DISCRETE)) 

#define ADC_VREF_TYPE           0xC0

//номера используемых каналов АЦП
#define ADCI_MAP                2
#define ADCI_UBAT               1         
#define ADCI_TEMP               0

//размер буферов усреднения по каждому датчику
#define MAP_AVERAGING           4   
#define BAT_AVERAGING           4   
#define TMP_AVERAGING           8  


//эти функции возвращают текущие значения из буферов усреднения
unsigned int adc_get_map_value(unsigned char index);
unsigned int adc_get_ubat_value(unsigned char index);
unsigned int adc_get_temp_value(unsigned char index);

//запускает измерение значений с датчиков, но только если предыдущее  
//измерение завершено.
void adc_begin_measure(void);

//возвращает не 0 если измерение готово (АЦП не занято)
char adc_is_measure_ready(void); 

//инициализация АЦП и его переменных состояния
void adc_init(void);

#endif //_ADC_H_
