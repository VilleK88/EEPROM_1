# Exercise 1 – Store the state of the program to EEPROM    
  
Implement a program that switches LEDs on and off and remembers the state of the LEDs across reboot  
and/or power off. The program should work as follows:  
-  When the program starts it reads the state of the LEDs from EEPROM. If no valid state is found in  
    the EEPROM the middle LED is switched on and the other two are switched off. The program must  
    print number of seconds since power up and the state of the LEDs to stdout. Use time_us_64() to  
    get a timestamp and convert that to seconds.  
-   Each of the buttons SW0, SW1, and SW2 on the development board is associated with an LED.  
    When user presses a button, the corresponding LED toggles. Pressing and holding the button may  
    not make the LED to blink or to toggle multiple times. When state of the LEDs is changed the new  
    state must be printed to stdout with a number of seconds since program was started.  
-   When the state of an LEDs changes the program stores the state of all LEDs in the EEPROM and  
    prints the state to LEDs to the debug UART. The program must employ a method to validate that  
    settings read from the EEPROM are correct.  
-   The program must use the highest possible EEPROM address to store the LED state.  
  
A simple way to validate the LED state is store it to EEPROM twice: normally (un-inverted) and inverted.  
When the state is read back both values are read, the inverted value is inverted after reading, and then the  
values are compared. If the values match then LED state was stored correctly in the EEPROM. By storing an  
inverted value, we can avoid case where both bytes are identical, a typical case with erased/out of the box  
memory, to be accepted as a valid value.  
