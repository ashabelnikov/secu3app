
    SECU-3 Firmware (Application). Distributed under GPL license

    Designed by Alexey A. Shabelnikov 2007. Ukraine, Kiev.
    Microprocessor systems - design & programming.
    http://secu-3.org e-mail: shabelnikov@secu-3.org


    HOW TO COMPILE THE PROJECT
    Как компилировать проект

    You can compile the project using either IAR(MS Windows) or GCC(Linux, MS
    Windows). Project compiles for ATMega644/ATMega644P
    Under MS Windows: Run configure.bat with corresponding options (type of MCU
                      and type of compiler),it will create Makefile and start
                      building.
    Under Linux:      Run configure.sh with option - type of MCU and type of
                      compiler, it will create Makefile and start building.

    Вы можете компилировать проект используя IAR или GCC. Запустите configure.bat
c соответствующими опциями (тип микроконтроллера и тип компилятора), будет создан
Makefile и начнется сборка проекта. Проект собирается для ATMega644/ATMega644P.
Ниже представлен список возможных опций компиляции. Исключение ненужных опций
позволит вам экономить память и ресурсы МК.

    LIST OF SYMBOLS WHICH AFFECT COMPILATION:
    Список символов управляющих компиляцией:

    SECU3T               Build for SECU-3T or for SECU-3i. If not defined, then
                         build for SECU-3i
                         собирать прошивку для SECU-3T или SECU-3i.

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

    GD_CONTROL           Enable stepper motor control for gas dose control
                         (Включить функциональность по управлению шаговым
                         двигателем для дозатора газа)

    CARB_AFR             Enable control of AFR on carburetor by means of
                         electronic actuators (valves driven by PWM)
                         (Включить поддержку управления составом смеси на
                          карбюраторе при помощи электронных актюаторов)

    CKPS_NPLUS1          Synchronization from N+1 crankshaft wheel (e.g. 2+1
                         used in kawasaki ZX6R)
                         Синхронизация от задающего диска N+1, например 2+1

    OBD_SUPPORT          OBD support (send data to the CAN network)
                         Поддержка OBD (cлать данные в CAN сеть)

    TPIC8101             Support of TPIC8101 knock chip
                         Поддержка микросхемы TPIC8101

    CAM_SYNC             Support of synchronization from camshaft wheel
                         Синхронизация от диска на распределительном вале

    BL_BAUD_RATE   *     Baud rate for boot loader. Can be set to 9600, 14400,
                         19200, 28800, 38400, 57600, 115200. Note! Will not take
                         effect without reprogramming using ISP programmator.
                         (Скорость передачи данных для загрузчика)

    SPEED_SENSOR    *    Include speed sensor support
                         Включить поддержку датчика скорости

    INTK_HEATING    *    Include support of intake manifold heating control
                         Включить поддержку управления подогревом впускного
                         коллектора

    AIRTEMP_SENS    *    Include support of intake air temperature sensor
                         Включить поддержку датчика температуры воздуха

    BLUETOOTH_SUPP  *    Include functionality for working with Bluetooth
                         Включить поддержку работы с Bluetooth

    IMMOBILIZER     *    Include immobilizer and iButton functionality
                         Включить поддержку иммобилайзера и iButton

    UNI_OUTPUT      *    Include support of an universal programmable output
                         Включить поддержку универсального программируемого
                         выхода

    PA4_INP_IGNTIM  *    Use PA4 as analog input for manual correction of
                         ignition timing

    SEND_INST_VAL   *    Send instant values (RPM, voltage) instead of averaged
                         Передавать мгновенные значения, без усреднения

    EVAP_CONTROL    *    Canister purge valve control
                         Управление клапаном продувки адсорбера

    AIRCONDIT       *    Air conditioner control
                         Управление кондиционером

    EGOS_HEATING    *    EGO sensor's heater control
                         Управление подогревом датчика кислорода

    MCP3204         *    Support of additional 4 analog inputs (using MCP3204)
                         Поддержка 4 дополнительных аналоговых входов

    IFR_VS_MAP_CORR *    Turn on correction of pressure in the fuel rail
                         Включить коррекцию давления в топливной рампе по ДАД

    DEFERRED_CRC    *    Turn on background checking of the firmware's CRC
                         Включить фоновую проверку контрольной суммы прошивки

* means that option is internal and not displayed in the list of options in the
  SECU-3 Manager
  означает что опция является внутренней и не отображается в списке опций в
  SECU-3 Manager

Necessary symbols you can define in the preprocessor's options of the compiler
(edit corresponding Makefile).
Нужные вам символы вы можете определить в опциях препроцессора компилятора 
(редактируйте соответствующий Makefile).
