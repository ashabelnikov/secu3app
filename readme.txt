
    SECU-3 Application software. Distributed under GPL license

    Created using IAR Embedded Workbench for Atmel AVR

    Designed by Alexey A. Shabelnikov 2007. Ukraine, Gorlovka. 
    Microprocessors systems - design & programming.
    http://secu-3.narod.ru e-mail: secu-3@narod.ru


      Как компилировать проект

    Проект можно скомпилировать под ATMega16, ATMega32, ATMega64. Для ATMega64 код компилируется, но 
работать он не будет! Выберите конфигурацию Release и в опциях выберите тип процессора. Проект можно 
компилировать. 

    Список символов управляющих компиляцией:

    VPSEM - для индикации состояния клапана ЭПХХ используется выход блокировки стартера
    WHEEL_36_1 - для использования зубчатого диска 36-1 (по умолчанию 60-2)
    INVERSE_IGN_OUTPUTS - для инвертирования выходов управления зажиганием
    COIL_REGULATION - для прямого управления накоплением энергии в катушках зажигания

Нужные вам символы вы можете определить в опциях препроцессора компилятора.
