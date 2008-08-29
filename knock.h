//параметры функций принимают данные в соответствии с форматом регистров 
//HIP9011. 

//Установка центральной цастоты полосового фильтра
void knock_set_band_pass(unsigned char freq);

//установка усиления программируемого усилителя
void knock_set_attenuation(unsigned char attenuation);

//установка времени интегрирования
void knock_set_integration_time(unsigned char inttime);

//подготовка канала детектора детонации
void knock_module_initialize(void);

