/*
	SECU-3  - An open source, free engine control unit
	Copyright (C) 2007 Alexey A. Shabelnikov. Ukraine, Kiev

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.

	contacts:
		http://secu-3.org
		email: shabelnikov@secu-3.org
	file ltft.c
	author Kosh Almaron, Alexey A. Shabelnikov
	Implementation of the long term fuel trim algorithm by KoshAlmaron
*/

#ifdef FUEL_INJECT

#include "port/port.h"
#include "port/pgmspace.h"
#include <stdlib.h>
#include "ltft.h"
#include "ecudata.h"
#include "eeprom.h"
#include "suspendop.h"
#include "funconv.h"
#include "lambda.h"
#include "mathemat.h"
#include "bitmask.h"

//forward declarations
void kosh_ltft_control(uint8_t Channel);
void kosh_write_value(uint8_t y, uint8_t x, uint8_t n, uint8_t Channel);
void kosh_find_cells(void);
void kosh_points_weight(void);
void kosh_add_ve_calculate(uint8_t Channel);
void kosh_rpm_map_calc(void);
void kosh_circular_buffer_update(void);

// Размер буфера
#define KOSH_CBS 40

// =============================================================================
// ============ Костыль для коррекции ячеек с помощью интерполяции =============
// =============================================================================

// Вытащил эти макросы из funconv.c
#define secu3_offsetof(type,member)   ((size_t)(&((type *)0)->member))
#define _GWU12(x,i,j) (d.mm_ptr12(secu3_offsetof(struct f_data_t, x), (i*16+j) ))

// Структура для хранения переменных
typedef struct {
	uint16_t RPM;					// Обороты x1
	uint16_t MAP;					// Давление x64
	uint16_t Kf;					// Коэффициент выравнивания x64
	uint8_t x1;						// Координаты рабочих ячеек Обороты
	uint8_t x2;						// -//-
	uint8_t y1;						// Координаты рабочих ячеек Давление
	uint8_t y2;						// -//-
	uint16_t StartVE[4];			// Начальные значения VE x2048
	uint16_t LTFTVE[4];				// Значения VE с текущей коррекцией LTFT x2048
	uint16_t CalcVE;				// Интерполяция начальной VE x2048
	uint16_t TargetVe;				// Целевое VE x2048
	uint16_t CellsProp[4];			// Вес ячеек в коррекции x2048
	int16_t VEAlignment[4];			// Добавка для выравнивания ячеек x2048
	int16_t AddVE[4];				// Добавка к VE по коррекции x2048
	int16_t LTFTAdd[4];				// Добавочный коэффициент LTFT x512
	uint16_t BufferRPM[KOSH_CBS];	// Кольцевой буфер оборотов
	uint16_t BufferMAP[KOSH_CBS];	// Кольцевой буфер давления
	uint8_t BufferIndex;			// Текущая позиция буфера
	uint8_t BufferAvg;				// Текущая позиция усреднения
	uint32_t BufferSumRPM;			// Переменная для суммирования оборотов
	uint32_t BufferSumMAP;			// Переменная для суммирования давления
	uint8_t UseGrid;				// Использовать сетку давления
	int16_t StepMAP;      			// Шаг сетки давления при использовании двух значений
} Kosh_t;

// Инициализация структуры
Kosh_t Kosh = {
				.RPM = 0,
				.MAP = 0,
				.Kf = 0,
				.x1 = 0,
				.x2 = 0,
				.y1 = 0,
				.y2 = 0,
				.StartVE = {0},
				.LTFTVE = {0},
				.CalcVE = 0,
				.TargetVe = 0,
				.CellsProp = {0},
				.VEAlignment = {0},
				.AddVE = {0},
				.LTFTAdd = {0},
				.BufferRPM = {0},
				.BufferMAP = {0},
				.BufferIndex = 0,
				.BufferAvg = 0,
				.BufferSumRPM = 0,
				.BufferSumMAP = 0,
				.UseGrid = 0,
				.StepMAP = 0
	};

// Порядок нумерации ячеек в массивах
//	1  2
//	0  3

// Расчет коррекции 
void kosh_ltft_control(uint8_t Channel) {
	// Уходим, пока не накопится коррекция
	if (d.corr.lambda[Channel] > -3 && d.corr.lambda[Channel] < 3) {return;}

	// Верхний порог по температуре на впуске 42 градуса x4
	if (d.sens.air_temp > 168) {return;}

	// Находим целевые обороты и давления с учетом задержки
	kosh_rpm_map_calc();

	// Пороги по оборотам и давлению (в основном для ХХ)
	if (Kosh.RPM < 500 || Kosh.RPM > 6000) {return;}
	if (Kosh.MAP < 10 * 64 || Kosh.MAP > 180 * 64) {return;}

	// Коэффициент выравнивания x64
	Kosh.Kf = 26;

	// Флаг использовать сетку давления
	Kosh.UseGrid = CHECKBIT(d.param.func_flags, FUNC_LDAX_GRID);
	if (!Kosh.UseGrid) {Kosh.StepMAP = (d.param.load_upper - d.param.load_lower) / 15;}

	// Поиск задействованных ячеек в расчете
	kosh_find_cells();

	// Извлечение значений из таблицы VE.
	// Умножаем значение на 8 для большей точности.
	// VE = VE1 (default)
	if (d.param.ve2_map_func == VE2MF_1ST) {
		Kosh.StartVE[0] = (_GWU12(inj_ve, Kosh.y1, Kosh.x1)) << 3;
		Kosh.StartVE[1] = (_GWU12(inj_ve, Kosh.y2, Kosh.x1)) << 3;
		Kosh.StartVE[2] = (_GWU12(inj_ve, Kosh.y2, Kosh.x2)) << 3;
		Kosh.StartVE[3] = (_GWU12(inj_ve, Kosh.y1, Kosh.x2)) << 3;
	}
	// VE = VE1 * VE2
	else if (d.param.ve2_map_func == VE2MF_MUL) {
		Kosh.StartVE[0] = (((int32_t) _GWU12(inj_ve, Kosh.y1, Kosh.x1) * _GWU12(inj_ve2, Kosh.y1, Kosh.x1)) >> 11) << 3;
		Kosh.StartVE[1] = (((int32_t) _GWU12(inj_ve, Kosh.y2, Kosh.x1) * _GWU12(inj_ve2, Kosh.y2, Kosh.x1)) >> 11) << 3;
		Kosh.StartVE[2] = (((int32_t) _GWU12(inj_ve, Kosh.y2, Kosh.x2) * _GWU12(inj_ve2, Kosh.y2, Kosh.x2)) >> 11) << 3;
		Kosh.StartVE[3] = (((int32_t) _GWU12(inj_ve, Kosh.y1, Kosh.x2) * _GWU12(inj_ve2, Kosh.y2, Kosh.x2)) >> 11) << 3;
	}
	// VE = VE1 + VE2
	else if (d.param.ve2_map_func == VE2MF_ADD) {
		Kosh.StartVE[0] = (_GWU12(inj_ve, Kosh.y1, Kosh.x1) + _GWU12(inj_ve2, Kosh.y1, Kosh.x1)) << 3;
		Kosh.StartVE[1] = (_GWU12(inj_ve, Kosh.y2, Kosh.x1) + _GWU12(inj_ve2, Kosh.y2, Kosh.x1)) << 3;
		Kosh.StartVE[2] = (_GWU12(inj_ve, Kosh.y2, Kosh.x2) + _GWU12(inj_ve2, Kosh.y2, Kosh.x2)) << 3;
		Kosh.StartVE[3] = (_GWU12(inj_ve, Kosh.y1, Kosh.x2) + _GWU12(inj_ve2, Kosh.y2, Kosh.x2)) << 3;
	}
	// На всякий случай, дерьмо случается.
	else {
		return;
	}

	// Вычисление значений с учетом имеющейся коррекции LTFT
	if (Channel) {
		Kosh.LTFTVE[0] = ((uint32_t) Kosh.StartVE[0] * (512 + d.inj_ltft2[Kosh.y1][Kosh.x1])) >> 9;
		Kosh.LTFTVE[1] = ((uint32_t) Kosh.StartVE[1] * (512 + d.inj_ltft2[Kosh.y2][Kosh.x1])) >> 9;
		Kosh.LTFTVE[2] = ((uint32_t) Kosh.StartVE[2] * (512 + d.inj_ltft2[Kosh.y2][Kosh.x2])) >> 9;
		Kosh.LTFTVE[3] = ((uint32_t) Kosh.StartVE[3] * (512 + d.inj_ltft2[Kosh.y1][Kosh.x2])) >> 9;
	}
	else {
		Kosh.LTFTVE[0] = ((uint32_t) Kosh.StartVE[0] * (512 + d.inj_ltft1[Kosh.y1][Kosh.x1])) >> 9;
		Kosh.LTFTVE[1] = ((uint32_t) Kosh.StartVE[1] * (512 + d.inj_ltft1[Kosh.y2][Kosh.x1])) >> 9;
		Kosh.LTFTVE[2] = ((uint32_t) Kosh.StartVE[2] * (512 + d.inj_ltft1[Kosh.y2][Kosh.x2])) >> 9;
		Kosh.LTFTVE[3] = ((uint32_t) Kosh.StartVE[3] * (512 + d.inj_ltft1[Kosh.y1][Kosh.x2])) >> 9;
	}

	// Расчет веса точек в коррекции
	kosh_points_weight();

	Kosh.CalcVE = bilinear_interpolation(Kosh.RPM, Kosh.MAP,
					Kosh.LTFTVE[0],
					Kosh.LTFTVE[1],
					Kosh.LTFTVE[2],
					Kosh.LTFTVE[3],
					PGM_GET_WORD(&fw_data.extabs.rpm_grid_points[Kosh.x1]),
					Kosh.UseGrid ? PGM_GET_WORD(&fw_data.extabs.load_grid_points[Kosh.y1]) : (Kosh.StepMAP * Kosh.y1 + d.param.load_lower),
					PGM_GET_WORD(&fw_data.extabs.rpm_grid_sizes[Kosh.x1]),
					Kosh.UseGrid ? PGM_GET_WORD(&fw_data.extabs.load_grid_sizes[Kosh.y1]) : (Kosh.StepMAP),
					1);

	// Целевое VE 
	Kosh.TargetVe = ((uint32_t) Kosh.CalcVE * (512 + d.corr.lambda[Channel])) >> 9;
	Kosh.TargetVe += 1;

	// Расчет добавки для выравнивания ячеек
	int8_t Ng = 1;
	for (uint8_t i = 0; i < 4; ++i) {
		// Тут может быть отрицательное число, а сдвигать биты в этом случае
		// это плохая идея, потому минус добавляем в конце.
		Ng = 1;
		Kosh.VEAlignment[i] = Kosh.TargetVe - Kosh.LTFTVE[i];
		if (Kosh.VEAlignment[i] < 0) {
			Ng = -1;
			Kosh.VEAlignment[i] *= Ng;
		}
		Kosh.VEAlignment[i] = ((uint32_t) Kosh.VEAlignment[i] * Kosh.CellsProp[i]) >> 11;
		Kosh.VEAlignment[i] = ((uint32_t) Kosh.VEAlignment[i] * Kosh.Kf) >> 6;
		Kosh.VEAlignment[i] *= Ng;
	}

	// Расчет добавки по лямбде
	kosh_add_ve_calculate(Channel);

	// Итого мы имеем два массива значений VEAlignment и AddVE,
	// которые необходимо добавить к VE.
	// Мы их считали уже с учетом имеющейся коррекции LTFT.

	// Теперь надо найти процент добавки от начального значения VE (без LTFT)
	// и добавить к текущему значению LTFT.

	// Расчет добавочного коэффициента LTFT
	for (uint8_t i = 0; i < 4; ++i) {
		Kosh.LTFTAdd[i] = (int32_t) (Kosh.VEAlignment[i] + Kosh.AddVE[i]) * 512 / Kosh.StartVE[i];
	}

	// Запись значений в таблицу LTFT
	kosh_write_value(Kosh.y1, Kosh.x1, 0, Channel);
	kosh_write_value(Kosh.y2, Kosh.x1, 1, Channel);
	kosh_write_value(Kosh.y2, Kosh.x2, 2, Channel);
	kosh_write_value(Kosh.y1, Kosh.x2, 3, Channel);

	// Обнуление лямбда коррекции
	d.corr.lambda[Channel] = 0;
}

void kosh_write_value(uint8_t y, uint8_t x, uint8_t n, uint8_t Channel) {
	// // Ограничение значения коррекции
	int8_t Value = Channel ? d.inj_ltft2[y][x] : d.inj_ltft1[y][x];
	int8_t Min = PGM_GET_BYTE(&fw_data.exdata.ltft_min);
	int8_t Max = PGM_GET_BYTE(&fw_data.exdata.ltft_max);

	if (Value + Kosh.LTFTAdd[n] > Max) {Kosh.LTFTAdd[n] = Max - Value;}
	else if (Value + Kosh.LTFTAdd[n] < Min) {Kosh.LTFTAdd[n] = Min - Value;}

	// Добавляем коррекцию в таблицу LTFT (Давление / Обороты)
	if (Channel) {d.inj_ltft2[y][x] += Kosh.LTFTAdd[n];}
	else 		 {d.inj_ltft1[y][x] += Kosh.LTFTAdd[n];}
}

// Поиск задействованных ячеек в расчете	
void kosh_find_cells(void) {
	// Чтобы убрать здесь и дальше исключительные ситуации,
	// когда обороты меньше сетки или попали точно в сетку и т.п.,
	// буду просто добавлять или отнимать единицу.

	// Обороты
	if (Kosh.RPM <= PGM_GET_WORD(&fw_data.extabs.rpm_grid_points[0])) {
		Kosh.RPM = PGM_GET_WORD(&fw_data.extabs.rpm_grid_points[0]) + 1;
	}
	if (Kosh.RPM >= PGM_GET_WORD(&fw_data.extabs.rpm_grid_points[15])) {
		Kosh.RPM = PGM_GET_WORD(&fw_data.extabs.rpm_grid_points[15]) - 1;
	}

	for (uint8_t i = 1; i < 16; i++) {
		if (Kosh.RPM <= PGM_GET_WORD(&fw_data.extabs.rpm_grid_points[i])) {
			if (Kosh.RPM == PGM_GET_WORD(&fw_data.extabs.rpm_grid_points[i])) {
				Kosh.RPM -= 1;
			}
			Kosh.x1 = i - 1;
			Kosh.x2 = i;
			break;
		}
	}

	// Давление
	if (Kosh.MAP <= (Kosh.UseGrid ? PGM_GET_WORD(&fw_data.extabs.load_grid_points[0]) : (d.param.load_lower))) {
		Kosh.MAP = Kosh.UseGrid ? (PGM_GET_WORD(&fw_data.extabs.load_grid_points[0]) + 1) : (d.param.load_lower + 1);
	}
	if (Kosh.MAP >= (Kosh.UseGrid ? PGM_GET_WORD(&fw_data.extabs.load_grid_points[15]) : (Kosh.StepMAP * 15 + d.param.load_lower))) {
		Kosh.MAP = Kosh.UseGrid ? (PGM_GET_WORD(&fw_data.extabs.load_grid_points[15]) - 1) : (Kosh.StepMAP * 15 + d.param.load_lower - 1);
	}

	for (uint8_t i = 1; i < 16; i++) {
		if (Kosh.MAP <= (Kosh.UseGrid ? PGM_GET_WORD(&fw_data.extabs.load_grid_points[i]) : (Kosh.StepMAP * i + d.param.load_lower))) {
			if (Kosh.MAP == (Kosh.UseGrid ? PGM_GET_WORD(&fw_data.extabs.load_grid_points[i]) : (Kosh.StepMAP * i + d.param.load_lower))) {
				Kosh.MAP -= 1;
			}
			Kosh.y1 = i - 1;
			Kosh.y2 = i;
			break;
		}
	}
}

// Расчет веса точек в коррекции
void kosh_points_weight(void) {
	uint16_t x1 = PGM_GET_WORD(&fw_data.extabs.rpm_grid_points[Kosh.x1]);
	uint16_t x2 = PGM_GET_WORD(&fw_data.extabs.rpm_grid_points[Kosh.x2]);
	uint16_t y1 = Kosh.UseGrid ? PGM_GET_WORD(&fw_data.extabs.load_grid_points[Kosh.y1]) : (Kosh.StepMAP * Kosh.y1 + d.param.load_lower);
	uint16_t y2 = Kosh.UseGrid ? PGM_GET_WORD(&fw_data.extabs.load_grid_points[Kosh.y2]) : (Kosh.StepMAP * Kosh.y2 + d.param.load_lower);

	uint16_t x = Kosh.RPM;
	uint16_t y = Kosh.MAP;

	uint16_t CFx1 = 0; // x2048 << 11
	uint16_t CFx2 = 0; // x2048
	uint16_t CFy1 = 0; // x2048
	uint16_t CFy2 = 0; // x2048

	CFx1 = (uint32_t) (x2 - x) * 2048 / (x2 - x1);
	CFx2 = (uint32_t) (x - x1) * 2048 / (x2 - x1);
				
	CFy1 = (uint32_t) (y2 - y) * 2048 / (y2 - y1);
	CFy2 = (uint32_t) (y - y1) * 2048 / (y2 - y1);

	Kosh.CellsProp[0] = ((uint32_t) CFx1 * CFy1) >> 11;
	Kosh.CellsProp[1] = ((uint32_t) CFx1 * CFy2) >> 11;
	Kosh.CellsProp[2] = ((uint32_t) CFx2 * CFy2) >> 11;
	Kosh.CellsProp[3] = ((uint32_t) CFx2 * CFy1) >> 11;
}

// Расчет добавки к VE
void kosh_add_ve_calculate(uint8_t Channel) {
	// Значение ячейки VE * Коррекцию * Долю
	uint16_t G[4] = {0, 0, 0, 0};
	uint16_t SummDelta = 0;

	// Если сдвигать биты с отрицательными числами, это может плохо закончиться.
	// Потому d.corr.lambda[Channel] временно делаем положительным.
	int8_t Ng = 1;
	int8_t Lambda = d.corr.lambda[Channel];
	if (Lambda < 0) {
		Ng = -1;
		Lambda *= Ng;
	}

	for (uint8_t i = 0; i < 4; ++i) {
		G[i] = ((uint32_t) Kosh.LTFTVE[i] * Lambda) >> 9;
		G[i] = ((uint32_t) G[i] * Kosh.CellsProp[i]) >> 11;
		// Сумма отклонения
		SummDelta += ((uint32_t) G[i] * Kosh.CellsProp[i]) >> 11;
	}

	uint16_t CalcVE2 = bilinear_interpolation(Kosh.RPM, Kosh.MAP,
				Kosh.LTFTVE[0] + Kosh.VEAlignment[0],
				Kosh.LTFTVE[1] + Kosh.VEAlignment[1],
				Kosh.LTFTVE[2] + Kosh.VEAlignment[2],
				Kosh.LTFTVE[3] + Kosh.VEAlignment[3],
				PGM_GET_WORD(&fw_data.extabs.rpm_grid_points[Kosh.x1]),
				Kosh.UseGrid ? PGM_GET_WORD(&fw_data.extabs.load_grid_points[Kosh.y1]) : (Kosh.StepMAP * Kosh.y1 + d.param.load_lower),
				PGM_GET_WORD(&fw_data.extabs.rpm_grid_sizes[Kosh.x1]),
				Kosh.UseGrid ? PGM_GET_WORD(&fw_data.extabs.load_grid_sizes[Kosh.y1]) : (Kosh.StepMAP),
				1);

	// Коэффициент отклонения от цели
	uint16_t Cf = (Kosh.TargetVe - CalcVE2) * Ng;
	Cf = (uint32_t) Cf * 1024 / SummDelta;

	// Добавка к VE
	for (uint8_t i = 0; i < 4; ++i) {
		Kosh.AddVE[i] = ((uint32_t) G[i] * Cf) >> 10;
		Kosh.AddVE[i] *= Ng;
	}
}

// Вычисление оборотов и давления с учетом задержки
void kosh_rpm_map_calc(void) {
	// Берем последние 8 значений давления для вычисления среднего
	uint32_t MAPAVG = 0;
	for (uint8_t i = 0; i < 8; i++) {
		int8_t Index = Kosh.BufferIndex - i;
		if (Index < 0) {Index = KOSH_CBS + Index;}
		MAPAVG += Kosh.BufferMAP[Index];
	}

  	MAPAVG = MAPAVG >> 3;
  	if (MAPAVG > (Kosh.UseGrid ? PGM_GET_WORD(&fw_data.extabs.load_grid_points[15]) : (Kosh.StepMAP * 15 + d.param.load_lower))) {
  		MAPAVG = Kosh.UseGrid ? PGM_GET_WORD(&fw_data.extabs.load_grid_points[15]) : (Kosh.StepMAP * 15 + d.param.load_lower);
  	}
  	// Находим задержку из сетки
  	for (uint8_t i = 0; i < 16; i++) {
  		if (MAPAVG <= (Kosh.UseGrid ? PGM_GET_WORD(&fw_data.extabs.load_grid_points[i]) : (Kosh.StepMAP * i + d.param.load_lower))) {

  			// Значения лага хранятся в таблице задержки лямбды
  			int8_t Index = (PGM_GET_BYTE(&fw_data.extabs.inj_ego_delay[i])) >> 2;
  			// Находим индекс оборотов и давления
  			Index = Kosh.BufferIndex - Index;
  			if (Index < 0) {Index = KOSH_CBS + Index;}

  			// Вытаскиваем оборотов и давления из прошлого
  			Kosh.RPM = Kosh.BufferRPM[Index];
  			Kosh.MAP = Kosh.BufferMAP[Index];
  			break;
  		}
 	}
}

// Обновление буфера
void kosh_circular_buffer_update(void) {
	Kosh.BufferSumRPM += d.sens.inst_rpm;
	Kosh.BufferSumMAP += d.sens.inst_map;
	Kosh.BufferAvg++;

	// Достигнут предел усреднения
	if (Kosh.BufferAvg >= 4) {
		uint16_t RPM = Kosh.BufferSumRPM >> 2;
		Kosh.BufferRPM[Kosh.BufferIndex] = RPM;
		uint16_t MAP = Kosh.BufferSumMAP >> 2;
		Kosh.BufferMAP[Kosh.BufferIndex] = MAP;

		Kosh.BufferAvg = 0;
		Kosh.BufferSumRPM = 0;
		Kosh.BufferSumMAP = 0;

		Kosh.BufferIndex++;
		// Достигнут конец буфера
		if (Kosh.BufferIndex >= KOSH_CBS) {
			Kosh.BufferIndex = 0;
		}
	}
}

// FUEL_INJECT
#endif
