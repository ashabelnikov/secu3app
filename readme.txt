
    SECU-3 Firmware (Application). Distributed under GPL license

    Designed by Alexey A. Shabelnikov 2007. Ukraine, Kiev.
    Microprocessor systems - design & programming.
    http://secu-3.org e-mail: shabelnikov@secu-3.org


    HOW TO COMPILE THE PROJECT
    ��� ������������� ������

    You can compile the project using either IAR(MS Windows) or GCC(Linux, MS
    Windows). Project compiles for ATMega644/ATMega644P
    Under MS Windows: Run configure.bat with corresponding options (type of MCU
                      and type of compiler),it will create Makefile and start
                      building.
    Under Linux:      Run configure.sh with option - type of MCU and type of
                      compiler, it will create Makefile and start building.

    �� ������ ������������� ������ ��������� IAR ��� GCC. ��������� configure.bat
c ���������������� ������� (��� ���������������� � ��� �����������), ����� ������
Makefile � �������� ������ �������. ������ ���������� ��� ATMega644/ATMega644P.
���� ����������� ������ ��������� ����� ����������. ���������� �������� �����
�������� ��� ��������� ������ � ������� ��.

    LIST OF SYMBOLS WHICH AFFECT COMPILATION:
    ������ �������� ����������� �����������:

    SECU3T               Build for SECU-3T or for SECU-3i. If not defined, then
                         build for SECU-3i
                         �������� �������� ��� SECU-3T ��� SECU-3i.

    DWELL_CONTROL        For direct controlling of dwell
                         ��� ������� ���������� ����������� ������� � ��������
                         ���������


    COOLINGFAN_PWM       Use PWM for controlling of electric cooling fan
                         ������������ ��� ��� ��� ��� ���������� ���������
                         �����������

    REALTIME_TABLES      Allow editing of tables in realtime (use RAM)
                         ��������� �������������� ������ � �������� �������

    DEBUG_VARIABLES      For watching and editing of some firmware variables
                         (used for debug by developers)
                         ��������� ����� ������� ����������� ����������� �
                         ������ ��������� ���������� ��������

    PHASE_SENSOR         Use of phase (cam) sensor
                         (��������� ������������� ������� ���)


    PHASED_IGNITION      Use phased ignition. PHASE_SENSOR must be also used.
                         (��������� ������������ ���������)

    FUEL_PUMP            Electric fuel pump control
                         (���������� �������������������)


    THERMISTOR_CS        Use a resistive temperature sensor
                         (������������ ������ ����������� ����������� ��������
                         ������������ ����)

    REV9_BOARD           Build for SECU-3T boards of revision 9 and greater.
                         (������ ��� ���� SECU-3T ������� 9 � ����)

    DIAGNOSTICS          Include hardware diagnostics functionality
                         (�������� ��������� ����������� ���������� �����)

    HALL_OUTPUT          Include Hall sensor emulation functionality. Separate
                         output will be used.
                         (�������� ��������� �������� ������� � ������� �����)

    STROBOSCOPE          Include stroboscope functionality
                         (�������� ��������� �����������)

    SM_CONTROL           Enable stepper motor and choke control functionality
                         (�������� ���������������� �� ���������� �������
                         ���������� � ��������� ���������)

    VREF_5V              Use 5V ADC reference voltage. In this case divider
                         bottom resistors are not necessary. So, input impedance
                         will be high.
                         (������������ ������� ���������� ��� ��� 5�)

    HALL_SYNC            Use synchronization from Hall sensor (connected to PS
                         input) instead of CKP sensor
                         ������������ ������������� �� �� ������ ����

    CKPS_2CHIGN          Build firmware for use 2 channel igniters (driven by
                         both edges)
                         ������� �������� � ���������� 2-� ���������
                         ������������ (����������� 2-�� ��������)

    FUEL_INJECT          Include support of fuel injection
                         �������� ��������� ���������� �������� �������

    GD_CONTROL           Enable stepper motor control for gas dose control
                         (�������� ���������������� �� ���������� �������
                         ���������� ��� �������� ����)

    CARB_AFR             Enable control of AFR on carburetor by means of
                         electronic actuators (valves driven by PWM)
                         (�������� ��������� ���������� �������� ����� ��
                          ����������� ��� ������ ����������� ����������)

    CKPS_NPLUS1          Synchronization from N+1 crankshaft wheel (e.g. 2+1
                         used in kawasaki ZX6R)
                         ������������� �� ��������� ����� N+1, �������� 2+1

    OBD_SUPPORT          OBD support (send data to the CAN network)
                         ��������� OBD (c���� ������ � CAN ����)

    TPIC8101             Support of TPIC8101 knock chip
                         ��������� ���������� TPIC8101

    CAM_SYNC             Support of synchronization from camshaft wheel
                         ������������� �� ����� �� ����������������� ����

    SPLIT_ANGLE          Split angle for ignition on rotary engines
                         ��������� ��� ��� �������-��������� ����������

    BL_BAUD_RATE   *     Baud rate for boot loader. Can be set to 9600, 14400,
                         19200, 28800, 38400, 57600, 115200. Note! Will not take
                         effect without reprogramming using ISP programmator.
                         (�������� �������� ������ ��� ����������)

    SPEED_SENSOR    *    Include speed sensor support
                         �������� ��������� ������� ��������

    INTK_HEATING    *    Include support of intake manifold heating control
                         �������� ��������� ���������� ���������� ���������
                         ����������

    AIRTEMP_SENS    *    Include support of intake air temperature sensor
                         �������� ��������� ������� ����������� �������

    BLUETOOTH_SUPP  *    Include functionality for working with Bluetooth
                         �������� ��������� ������ � Bluetooth

    IMMOBILIZER     *    Include immobilizer and iButton functionality
                         �������� ��������� ������������� � iButton

    UNI_OUTPUT      *    Include support of an universal programmable output
                         �������� ��������� �������������� ����������������
                         ������

    PA4_INP_IGNTIM  *    Use PA4 as analog input for manual correction of
                         ignition timing

    SEND_INST_VAL   *    Send instant values (RPM, voltage) instead of averaged
                         ���������� ���������� ��������, ��� ����������

    EVAP_CONTROL    *    Canister purge valve control
                         ���������� �������� �������� ���������

    AIRCONDIT       *    Air conditioner control
                         ���������� �������������

    EGOS_HEATING    *    EGO sensor's heater control
                         ���������� ���������� ������� ���������

    MCP3204         *    Support of additional 4 analog inputs (using MCP3204)
                         ��������� 4 �������������� ���������� ������

    IFR_VS_MAP_CORR *    Turn on correction of pressure in the fuel rail
                         �������� ��������� �������� � ��������� ����� �� ���

    DEFERRED_CRC    *    Turn on background checking of the firmware's CRC
                         �������� ������� �������� ����������� ����� ��������

* means that option is internal and not displayed in the list of options in the
  SECU-3 Manager
  �������� ��� ����� �������� ���������� � �� ������������ � ������ ����� �
  SECU-3 Manager

Necessary symbols you can define in the preprocessor's options of the compiler
(edit corresponding Makefile).
������ ��� ������� �� ������ ���������� � ������ ������������� ����������� 
(������������ ��������������� Makefile).
