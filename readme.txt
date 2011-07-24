
    SECU-3 Application software. Distributed under GPL license
    
    Designed by Alexey A. Shabelnikov 2007. Ukraine, Gorlovka.
    Microprocessors systems - design & programming.
    http://secu-3.org e-mail: shabelnikov@secu-3.org


      How to compile the project
      Как компилировать проект

    It is possible to compile project for ATMega16, ATMega32, ATMega64. Version for ATMega64 compiles,
but it will not work! You can compile the project using either IAR or WinAvr. Run configure.bat with 
corresponding options (type of MCU and type of compiler), it will create Makefile and start building.
    Проект можно скомпилировать под ATMega16, ATMega32, ATMega64. Для ATMega64 код компилируется, но
работать он не будет! Вы можете компилировать проект используя IAR или WinAvr. Запустите configure.bat
c соответствующими опциями (тип микроконтроллера и тип компилятора), будет создан Makefile и начнется
сборка проекта.

    List of symbols which affects compilation:
    Список символов управляющих компиляцией:

    VPSEM - for using of starter blocking output for indication of idle economizer valve's state
            для индикации состояния клапана ЭПХХ используется выход блокировки стартера


    WHEEL_36_1 - for using 36-1 crank (60-2 by default)
                 для использования зубчатого диска 36-1 (по умолчанию 60-2)


    INVERSE_IGN_OUTPUTS - use for to invert ignition outputs
                          для инвертирования выходов управления зажиганием


    COIL_REGULATION - for direct controlling of coil regulation
                      для прямого управления накоплением энергии в катушках зажигания


    COOLINGFAN_PWM - use PWM for controlling of electric cooling fan
                     использовать или нет ШИМ для управления оборотами вентилятора

    REALTIME_TABLES - allow editing of tables in realtime (use RAM)


Necessary symbols you can define in the preprocessor's options of compiler
(edit corresponding Makefile).
Нужные вам символы вы можете определить в опциях препроцессора компилятора 
(редактируйте соответствующий Makefile).
