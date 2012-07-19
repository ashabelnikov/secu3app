
    SECU-3 Application software. Distributed under GPL license

    Designed by Alexey A. Shabelnikov 2007. Ukraine, Gorlovka.
    Microprocessors systems - design & programming.
    http://secu-3.org e-mail: shabelnikov@secu-3.org


    How to compile the project
    Как компилировать проект

    It is possible to compile project for ATMega16, ATMega32, ATMega64. Version
for ATMega64 compiles,but it will not work! You can compile the project using 
either IAR(MS Windows) or GCC(Linux, MS Windows).
    Under MS Windows: Run configure.bat with corresponding options (type of MCU
                      and type of compiler),it will create Makefile and start 
                      building.
    Under Linux:      Run configure.sh with option - type of MCU, it will create
                      Makefile and start building.

    Проект можно скомпилировать под ATMega16, ATMega32, ATMega64. Для ATMega64 
код компилируется, но работать он не будет! Вы можете компилировать проект 
используя IAR или GCC. Запустите configure.bat c соответствующими опциями (тип 
микроконтроллера и тип компилятора), будет создан Makefile и начнется сборка 
проекта.

    List of symbols which affects compilation:
    Список символов управляющих компиляцией:

    VPSEM                For using of starter blocking output for indication of 
                         idle economizer valve's state
                         для индикации состояния клапана ЭПХХ используется выход
                         блокировки стартера


    WHEEL_36_1           For using 36-1 crank (60-2 used by default)
                         для использования зубчатого диска 36-1 (по умолчанию 
                         60-2)


    INVERSE_IGN_OUTPUTS  Use for to invert ignition outputs
                         для инвертирования выходов управления зажиганием


    DWELL_CONTROL        For direct controlling of dwell
                         для прямого управления накоплением энергии в катушках 
                         зажигания


    COOLINGFAN_PWM       Use PWM for controlling of electric cooling fan
                         использовать или нет ШИМ для управления оборотами 
                         вентилятора

    REALTIME_TABLES      Allow editing of tables in realtime (use RAM)
                         разрешить редактирование таблиц в реальном времени

    DEBUG_VARIABLES      For watching and editing of some firmware variables 
                         (used for debug by developers)
                         разрешить режим отладки позволяющий отслеживать и 
                         менять некоторые переменные прошивки

    PHASE_SENSOR         Use of phase (cam) sensor
                         (разрешить использование датчика фаз)


    PHASED_IGNITION      Use phased ignition. PHASE_SENSOR must be also used.
                         (разрешить фазированное зажигание)

    FUEL_PUMP            Electric fuel pump control
                         (Управление электробензонасосом)

    BL_BAUD_RATE         Baud rate for boot loader. Can be set to 9600, 14400,
                         19200, 28800, 38400, 57600. Note! Will not take effect without
                         reprogramming using ISP programmator.
                         (Скорость передачи данных для загрузчика)

    THERMISTOR_CS        Use a resistive temperature sensor
                         (Используется датчик температуры охлаждающей жидкости
                         резистивного типа)

    SECU3T               Build for SECU-3T unit. Additional functionality will be added
                         (Сборка под блок SECU-3T. Добавляется доп. функциональность)

    REV9_BOARD           Build for SECU-3T boards of revision 9 and greater.
                         (Сборка для плат SECU-3T ревизии 9 и выше)

    DIAGNOSTICS          Include hardware diagnostics functionality
                         (Включить поддержку диагностики аппаратной части)

    HALL_OUTPUT          Include Hall sensor emulation functionality. Separate output
                         will be used.
                         (Включить поддержку эмуляции сигнала с Датчика Холла)

Necessary symbols you can define in the preprocessor's options of compiler
(edit corresponding Makefile).
Нужные вам символы вы можете определить в опциях препроцессора компилятора 
(редактируйте соответствующий Makefile).
