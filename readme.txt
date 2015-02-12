
    SECU-3 Application software. Distributed under GPL license

    Designed by Alexey A. Shabelnikov 2007. Ukraine, Kiev.
    Microprocessor systems - design & programming.
    http://secu-3.org e-mail: shabelnikov@secu-3.org


    How to compile the project
    Как компилировать проект

    It is possible to compile the project for ATMega16, ATMega32, ATMega64,
ATMega644. Version for ATMega64 compiles, but it will not work! You can compile
the project using either IAR(MS Windows) or GCC(Linux, MS Windows).
    Under MS Windows: Run configure.bat with corresponding options (type of MCU
                      and type of compiler),it will create Makefile and start
                      building.
    Under Linux:      Run configure.sh with option - type of MCU, it will create
                      Makefile and start building.

    Проект можно скомпилировать для ATMega16, ATMega32, ATMega64. Для ATMega64
код компилируется, но работать он не будет! Вы можете компилировать проект
используя IAR или GCC. Запустите configure.bat c соответствующими опциями (тип
микроконтроллера и тип компилятора), будет создан Makefile и начнется сборка
проекта. Ниже представлен список возможных опций компиляции. Исключение ненужных
опций позволит вам экономить память и ресурсы МК.

    List of symbols which affects compilation:
    Список символов управляющих компиляцией:

    VPSEM                For using of starter blocking output for indication of
                         idle cut off valve's state
                         для индикации состояния клапана ЭПХХ используется выход
                         блокировки стартера

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


    THERMISTOR_CS        Use a resistive temperature sensor
                         (Используется датчик температуры охлаждающей жидкости
                         резистивного типа)

    REV9_BOARD           Build for SECU-3T boards of revision 9 and greater.
                         (Сборка для плат SECU-3T ревизии 9 и выше)

    DIAGNOSTICS          Include hardware diagnostics functionality
                         (Включить поддержку диагностики аппаратной части)

    HALL_OUTPUT          Include Hall sensor emulation functionality. Separate
                         output will be used.
                         (Включить поддержку эмуляции сигнала с Датчика Холла)

    STROBOSCOPE          Include stroboscope functionality
                         (Включить поддержку стробоскопа)

    SM_CONTROL           Enable stepper motor and choke control functionality
                         (Включить функциональность по управлению шаговым
                         двигателем и воздушной заслонкой)

    VREF_5V              Use 5V ADC reference voltage. In this case divider
                         bottom resistors are not necessary. So, input impedance
                         will be high.
                         (Использовать опорное напряжение для АЦП 5В)

    HALL_SYNC            Use synchronization from Hall sensor (connected to PS
                         input) instead of CKP sensor
                         Использовать синхронизацию от ДХ вместо ДПКВ

    CKPS_2CHIGN          Build firmware for use 2 channel igniters (driven by
                         both edges)
                         Собрать прошивку с поддержкой 2-х канальных
                         коммутаторов (управляются 2-мя фронтами)

    UART_BINARY          Use binary mode for UART instead of ASCII
                         Использовать бинарный режим при передаче данных через
                         UART вместо ASCII

    FUEL_INJECT          Include support of fuel injection
                         Включить поддержку управления впрыском топлива

    BL_BAUD_RATE   *     Baud rate for boot loader. Can be set to 9600, 14400,
                         19200, 28800, 38400, 57600. Note! Will not take effect
                         without reprogramming using ISP programmator.
                         (Скорость передачи данных для загрузчика)

    SPEED_SENSOR   *     Include speed sensor support
                         Включить поддержку датчика скорости

    INTK_HEATING   *     Include support of intake manifold heating control
                         Включить поддержку управления подогревом впускного
                         коллектора

    AIRTEMP_SENS   *     Include support of intake air temperature sensor
                         Включить поддержку датчика температуры воздуха

    BLUETOOTH_SUPP *     Include functionality for working with Bluetooth
                         Включить поддержку работы с Bluetooth

    IMMOBILIZER    *     Include immobilizer and iButton functionality
                         Включить поддержку иммобилайзера и iButton

    UNI_OUTPUT     *     Include support of an universal programmable output
                         Включить поддержку универсального программируемого
                         выхода

* means that option is internal and not displayed in the list of options in the
  SECU-3 Manager
  означает что опция является внутренней и не отображается в списке опций в
  SECU-3 Manager

Necessary symbols you can define in the preprocessor's options of the compiler
(edit corresponding Makefile).
Нужные вам символы вы можете определить в опциях препроцессора компилятора 
(редактируйте соответствующий Makefile).
