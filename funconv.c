 /****************************************************************
 *       SECU-3  - An open source, free engine control unit
 *    Designed by Alexey A. Shabelnikov. Ukraine, Gorlovka 2007.
 *       Microprocessors systems - design & programming.
 *    contacts:
 *              http://secu-3.narod.ru
 *              ICQ: 405-791-931
 ****************************************************************/

#include <stdlib.h>
#include "funconv.h"
#include "adc.h"
#include "ckps.h"
#include "secu3.h"

//данные массивы констант задают сетку по оси оборотов, для рабочей карты и карты ХХ.
__flash const int F_SlotsRanges[16] = {600,720,840,990,1170,1380,1650,1950,2310,2730,3210,3840,4530,5370,6360,7500}; 
__flash const int F_SlotsLength[15] = {120,120,150,180, 210, 270, 300, 360, 420, 480, 630, 690, 840, 990, 1140}; 

// Функция билинейной интерполяции (поверхность)
// x, y - значения аргументов интерполируемой функции
// a1,a2,a3,a4 - значение функции в узлах интерполяции (углы четырехугольника)
// x_s,y_s - значения аргументов функции соответствующие началу прямоугольной области
// x_l,y_l - размеры прямоугольной области (по x и y соответственно)
// возвращает интерполированное значение функции * 16         
int bilinear_interpolation(int x,int y,int a1,int a2,int a3,int a4,int x_s,int y_s,int x_l,int y_l)
{
   int a23,a14;  
   a23 = ((a2 * 16) + (((long)(a3 - a2) * 16) * (x - x_s)) / x_l);
   a14 = (a1 * 16) + (((long)(a4 - a1) * 16) * (x - x_s)) / x_l;
   return (a14 + ((((long)(a23 - a14)) * (y - y_s)) / y_l));
} 

// Функция линейной интерполяции
// x - значение аргумента интерполируемой функции
// a1,a2 - значения функции в узлах интерполяции
// x_s - значение аргумента функции в начальной точке
// x_l - длина отрезка между точками
// возвращает интерполированное значение функции * 16                   
int simple_interpolation(int x,int a1,int a2,int x_s,int x_l)
{
  return ((a1 * 16) + (((long)(a2 - a1) * 16) * (x - x_s)) / x_l);
}


// Реализует функцию УОЗ от оборотов для холостого хода
// Возвращает значение угла опережения в целом виде * 32. 2 * 16 = 32.
int idling_function(ecudata* d)
{
  signed char i;
  int rpm = d->sens.inst_frq;

  //находим узлы интерполяции, вводим ограничение если обороты выходят за пределы
  for(i = 14; i >= 0; i--)
    if (d->sens.inst_frq >= F_SlotsRanges[i]) break;                        

  if (i < 0)  {i = 0; rpm = 600;}

  return simple_interpolation(rpm,
              d->fn_dat->f_idl[i],d->fn_dat->f_idl[i+1],
              F_SlotsRanges[i],F_SlotsLength[i]);
}


// Реализует функцию УОЗ от оборотов для пуска двигателя
// Возвращает значение угла опережения в целом виде * 32, 2 * 16 = 32.
int start_function(ecudata* d)
{
  int i,i1,rpm = d->sens.inst_frq;                                           

  if (rpm < 200) rpm = 200; //200 - минимальное значение оборотов

  i = (rpm - 200) / 40;   //40 - шаг по оборотам

  if (i >= 15) i = i1 = 15; 
  else i1 = i + 1;

  return simple_interpolation(rpm,d->fn_dat->f_str[i],d->fn_dat->f_str[i1], (i * 40) + 200, 40);
}


// Реализует функцию УОЗ от оборотов(мин-1) и нагрузки(кПа) для рабочего режима двигателя
// Возвращает значение угла опережения в целом виде * 32, 2 * 16 = 32.
int work_function(ecudata* d)
{    
   int  gradient, discharge, rpm = d->sens.inst_frq, l;
   signed char f,fp1,lp1;   

   //находим узлы интерполяции, вводим ограничение если обороты выходят за пределы            
   for(f = 14; f >= 0; f--)   
     if (rpm >= F_SlotsRanges[f]) break; 
                            
   //рабочая карта работает на 600-х оборотах и выше                                                        
   if (f < 0)  {f = 0; rpm = 600;}
   fp1 = f + 1;   
   
   discharge = (d->param.map_upper_pressure - d->sens.map);
   if (discharge < 0) discharge = 0;         
   
   //map_upper_pressure - верхнее значение давления
   //map_lower_pressure - нижнее значение давления
   gradient = (d->param.map_upper_pressure - d->param.map_lower_pressure) / 16; //делим на количество узлов интерполяции по оси давления
   if (gradient < 1)
     gradient = 1;  //исключаем деление на ноль и отрицательное значение если верхнее давление меньше нижнего
   l = (discharge / gradient);
   
   if (l >= (F_WRK_POINTS_F - 1))
     lp1 = l = F_WRK_POINTS_F - 1;
   else
     lp1 = l + 1;      

   //обновляем переменную расхода воздуха
   d->airflow = 16 - l;
   
   return bilinear_interpolation(rpm, discharge,
	   d->fn_dat->f_wrk[l][f],
	   d->fn_dat->f_wrk[lp1][f],
	   d->fn_dat->f_wrk[lp1][fp1],
	   d->fn_dat->f_wrk[l][fp1],
	   F_SlotsRanges[f],
	   (gradient * l),
	   F_SlotsLength[f],
	   gradient);
}


//Реализует функцию коррекции УОЗ по температуре(град. Цельсия) охлаждающей жидкости
// Возвращает значение угла опережения в целом виде * 32, 2 * 16 = 32.
int coolant_function(ecudata* d)
{ 
  int i,i1,t = d->sens.temperat;                                           

  if (!d->param.tmp_use) 
    return 0;   //нет коррекции, если блок неукомплектован ДТОЖ-ом
    
  //-30 - минимальное значение температуры
  if (t < TEMPERATURE_MAGNITUDE(-30)) 
    t = TEMPERATURE_MAGNITUDE(-30);   

  //10 - шаг между узлами интерполяции по температуре
  i = (t - TEMPERATURE_MAGNITUDE(-30)) / TEMPERATURE_MAGNITUDE(10);   

  if (i >= 15) i = i1 = 15; 
  else i1 = i + 1;

  return simple_interpolation(t,d->fn_dat->f_tmp[i],d->fn_dat->f_tmp[i1], 
  (i * TEMPERATURE_MAGNITUDE(10)) + TEMPERATURE_MAGNITUDE(-30), TEMPERATURE_MAGNITUDE(10));   
}


//-------------------------------------------------------------------------------------------
//       Регулятор холостого хода
typedef struct
{
 int output_state;    //память регулятора для хранения последнего значения управляющего воздействия (коррекции)
}IDLREGULSTATE;

IDLREGULSTATE idl_prstate;

//сброс состояния РХХ
void idling_regulator_init(void)
{
 idl_prstate.output_state = 0;
}

//Пропорциональный регулятор для регулирования оборотов ХХ углом опережения зажигания     
// Возвращает значение угла опережения в целом виде * 32.
int idling_pregulator(ecudata* d)
{
  int error,factor,diff;
  //зона "подхвата" регулятора при возвращени двигателя из рабочего режима в ХХ
  unsigned int capture_range = 200; 
    
  //если запрещено автоматическое регулирование оборотов ХХ или обороты значительно
  // выше от нормальных холостых оборотов то выходим  с нулевой корректировкой        
  if (!d->param.idl_regul || (d->sens.frequen4 >(d->param.idling_rpm + capture_range)))
    return 0;  
    
  //вычисляем значение ошибки, ограничиваем ошибку (если нужно), а также если мы в зоне 
  //нечувствительности, то нет регулирования.     
  diff = d->param.idling_rpm - d->sens.frequen4;   
  if (diff > 350) diff = 350;
  if (diff <-350) diff = -350;
  if (abs(diff) <= d->param.MINEFR) 
    return idl_prstate.output_state;
    
  //выбираем необходимый коэффициент и знач. ошибки для регулятора, в зависимости от знака ошибки
  if (diff > d->param.MINEFR)
  {
    error = diff - d->param.MINEFR;
    factor = d->param.ifac1;
  }
  if (diff < -d->param.MINEFR)    
  {
    error = diff + d->param.MINEFR;
    factor = d->param.ifac2;                         
  }
     
  //при коэффициенте равном 1.0, скорость изменения УОЗ равна скорости изменения ошибки,
  //дискретность коэффициента равна дискретности УОЗ!   
  idl_prstate.output_state = (factor * error);
  
  //ограничиваем коррекцию нижним и верхним пределами регулирования
  if (idl_prstate.output_state > ANGLE_MAGNITUDE(30))  
   idl_prstate.output_state = ANGLE_MAGNITUDE(30);
  if (idl_prstate.output_state < ANGLE_MAGNITUDE(-30))  
   idl_prstate.output_state = ANGLE_MAGNITUDE(-30);
      
  return idl_prstate.output_state;    
}

//-------------------------------------------------------------------------------------------


//Нелинейный фильтр ограничивающий скорость изменения УОЗ на переходных режимах двигателя
//new_advance_angle - новое значение УОЗ
//intstep - значение шага интегрирования, положительное число
//is_enabled - если равен 1, то корректировка разрешена, 0 - запрещена
//Возвращает скорректированный УОЗ
int transient_state_integrator(int new_advance_angle, unsigned int intstep, char is_enabled)
{
 static signed int old_advance_angle = 0;
 signed int difference;
 if (is_enabled)
 {
  difference = new_advance_angle - old_advance_angle;  
  if (abs(difference) > intstep)
  {
   if (difference > 0)
     old_advance_angle+=intstep;
   else    
     old_advance_angle-=intstep;
   return old_advance_angle;
  }
 }
 //текущий УОЗ будет предыдущим в следующий раз
 old_advance_angle = new_advance_angle;
 return old_advance_angle;
}
