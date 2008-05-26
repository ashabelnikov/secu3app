
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
