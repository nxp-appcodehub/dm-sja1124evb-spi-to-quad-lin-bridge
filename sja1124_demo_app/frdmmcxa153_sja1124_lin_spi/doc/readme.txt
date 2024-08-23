Hardware requirements
===================
- Mini/micro C USB cable
- FRDM-MCXA153 board
- PCA9957-ARD board 
- Personal Computer

Board settings

- There are jumpers corresponding to each LED, we can remove particular jumper to 
  disconnect a particular LED from LED Channel.
  eg. J76,J77,J78.
- To connect external LED, Use J21 and J26.
- To test short and open circuit:
  Short Circuit: J49, J58, J57 etc.
  Open Circuit:  J76,J77,J78 etc. 
- Jumper to control Imax: J84

Prepare the Demos
===============
1.  Connect a USB cable between the host PC and the OpenSDA USB port on the target board.
2.  Open a serial terminal with the following settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
3.  Download the program to the target board.
4.  Either press the reset button on your board or launch the debugger in your IDE to begin running the demo.

Running the demo
===============
When the demo runs successfully, you can see the logs printed on the terminal based on the menu options user chooses.

LOGS:
=============
 =~=~=~=~=~=~=~=~=~=~=~= PuTTY log 2024.05.22 17:24:04 =~=~=~=~=~=~=~=~=~=~=~=

 ISSDK PCA9957 LED driver example demonstration for SPI with interrupt mode.

 Successfully Applied PCA9957 Configuration

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 1


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 1

 ALL LED Switched ON


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 1

 ALL LED Switched ON


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 1

 ALL LED Switched ON


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 1

 ALL LED Switched ON


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 1

 ALL LED Switched ON


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 1

 ALL LED Switched ON


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 
 ISSDK PCA9957 LED driver example demonstration for SPI with interrupt mode.

 Successfully Applied PCA9957 Configuration

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 8

 1. Auto Switch Off Enable 

 2. Auto Switch Off Disable 

 Enter your choice :- 2

 Auto Switch Off Disable 

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 1


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 1

 ALL LED Switched ON


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 2

 ALL LED Switched OFF


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 1

 ALL LED Switched ON


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 3

 Enter Brightness from 1 to 255 :- 240

 ALL LED Brightness Set Done 


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 3

 Enter Brightness from 1 to 255 :- 8

 ALL LED Brightness Set Done 


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 4

 1. LED Off

 2. LED On

 3. Individual LED brightness Control

 4. Individual LED brightness and group dimming/blinking

 Enter your choice :- 4

 ALL LED State Set Done


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 5

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 3

 ********  Dimming/Blinking Control ********

 1. Set Dimming/Blinking LED Brightness 

 2. Set Dimming/Blinking LED Frequency 

 3. Enable Dimming/Blinking 

 4. Disable Dimming/Blinking 

 5. Exit Dimming/Blinking Control 

 Enter your choice :- 1

 Enter the Blinking PWM [1 to 255] :- 20

 Blinking Brightness Set Done 

 ********  Dimming/Blinking Control ********

 1. Set Dimming/Blinking LED Brightness 

 2. Set Dimming/Blinking LED Frequency 

 3. Enable Dimming/Blinking 

 4. Disable Dimming/Blinking 

 5. Exit Dimming/Blinking Control 

 Enter your choice :- 2

 Enter the Blinking frequency [1 to 255] :- 20

 Blinking frequency Set Done 

 ********  Dimming/Blinking Control ********

 1. Set Dimming/Blinking LED Brightness 

 2. Set Dimming/Blinking LED Frequency 

 3. Enable Dimming/Blinking 

 4. Disable Dimming/Blinking 

 5. Exit Dimming/Blinking Control 

 Enter your choice :- 3

 Blinking Enabled 

 ********  Dimming/Blinking Control ********

 1. Set Dimming/Blinking LED Brightness 

 2. Set Dimming/Blinking LED Frequency 

 3. Enable Dimming/Blinking 

 4. Disable Dimming/Blinking 

 5. Exit Dimming/Blinking Control 

 Enter your choice :- 5

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 2


 ******** Individual LED Control ********

 1. Individual LED On 

 2. Individual LED Off 

 3. Set Individual LED Brightness 

 4. Set Individual LED State 

 5. Exit From Individual LED Control 

 Enter your choice :- 1

 Enter LED Number from 1 to 24 :- 1

 LED-1 On 


 ******** Individual LED Control ********

 1. Individual LED On 

 2. Individual LED Off 

 3. Set Individual LED Brightness 

 4. Set Individual LED State 

 5. Exit From Individual LED Control 

 Enter your choice :- 2

 Enter LED Number from 1 to 24 :- 2

 LED-2 Off Done


 ******** Individual LED Control ********

 1. Individual LED On 

 2. Individual LED Off 

 3. Set Individual LED Brightness 

 4. Set Individual LED State 

 5. Exit From Individual LED Control 

 Enter your choice :- 3

 Enter LED Number from 1 to 24 :- 2

 Enter Brightness from 1 to 255 :- 100

 LED-2 Brightness Set Done


 ******** Individual LED Control ********

 1. Individual LED On 

 2. Individual LED Off 

 3. Set Individual LED Brightness 

 4. Set Individual LED State 

 5. Exit From Individual LED Control 

 Enter your choice :- 5

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 4

 ********* Gradation Main Menu *********

 1. Gradation Configuration 

 2. Assign LED to Gradation Group 

 3. Enable LED channel for Gradation 

 4. Disable LED channel for Gradation 

 5. Start Gradation 

 6. Stop Gradation 

 7. Exit Gradation Main Menu 

 Enter your choice :- 7

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 5

 Reset Done 

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 4

 ********* Gradation Main Menu *********

 1. Gradation Configuration 

 2. Assign LED to Gradation Group 

 3. Enable LED channel for Gradation 

 4. Disable LED channel for Gradation 

 5. Start Gradation 

 6. Stop Gradation 

 7. Exit Gradation Main Menu 

 Enter your choice :- 1

 Enter Gradation Group [1 to 6] :- 1

 1. Ramp Configuration 

 2. Step Time Configuration 

 3. Hold Configuration 

 4. Ramp Current Configuration 

 5. Set Linear/Exponential adjustment 

 6. Exit Gradation Configuration 

 Enter your choice :- 1

 **** Ramp Configuration **** 

 1: Enable Ramp UP 

 2: Disable Ramp UP 

 3: Enable Ramp Down 

 4: Disable Ramp Down 

 5: Set Ramp Value 

 6. Exit from Ramp Configuration 

 Enter your choice :- 1

 Ramp UP Enabled 

 **** Ramp Configuration **** 

 1: Enable Ramp UP 

 2: Disable Ramp UP 

 3: Enable Ramp Down 

 4: Disable Ramp Down 

 5: Set Ramp Value 

 6. Exit from Ramp Configuration 

 Enter your choice :- 3

 Ramp Down Enabled 

 **** Ramp Configuration **** 

 1: Enable Ramp UP 

 2: Disable Ramp UP 

 3: Enable Ramp Down 

 4: Disable Ramp Down 

 5: Set Ramp Value 

 6. Exit from Ramp Configuration 

 Enter your choice :- 5

 Enter Ramp Value [1 to 64] :- 30

 Ramp Value Set Done 

 **** Ramp Configuration **** 

 1: Enable Ramp UP 

 2: Disable Ramp UP 

 3: Enable Ramp Down 

 4: Disable Ramp Down 

 5: Set Ramp Value 

 6. Exit from Ramp Configuration 

 Enter your choice :- 6

 1. Ramp Configuration 

 2. Step Time Configuration 

 3. Hold Configuration 

 4. Ramp Current Configuration 

 5. Set Linear/Exponential adjustment 

 6. Exit Gradation Configuration 

 Enter your choice :- 2

 **** Step Time Configuration **** 

 1: 0.5 MS Cycle time

 2: 8 MS Cycle time

 3: Set Step Time MF

 4. Exit from Step Time Configuration 

 Enter your choice :- 2

 8 ms Cycle time Set Done

 **** Step Time Configuration **** 

 1: 0.5 MS Cycle time

 2: 8 MS Cycle time

 3: Set Step Time MF

 4. Exit from Step Time Configuration 

 Enter your choice :- 3

 Enter Step Time [1 to 64] :- 50

 Multiple factor Set Done

 **** Step Time Configuration **** 

 1: 0.5 MS Cycle time

 2: 8 MS Cycle time

 3: Set Step Time MF

 4. Exit from Step Time Configuration 

 Enter your choice :- 4

 1. Ramp Configuration 

 2. Step Time Configuration 

 3. Hold Configuration 

 4. Ramp Current Configuration 

 5. Set Linear/Exponential adjustment 

 6. Exit Gradation Configuration 

 Enter your choice :- 3

 **** Hold Configuration **** 

 1: Enable Hold On

 2: Disable Hold On

 3: Enable Hold Off

 4: Disable Hold Off

 5. Exit from Hold Configuration 

 Enter your choice :- 1

 Enter Hold Time in Second (0, 0.25, 0.5, 0.75, 1, 2, 4, 6) [1 to 8] :- 5

 Hold On Enabled 

 **** Hold Configuration **** 

 1: Enable Hold On

 2: Disable Hold On

 3: Enable Hold Off

 4: Disable Hold Off

 5. Exit from Hold Configuration 

 Enter your choice :- 3

 Enter Hold Time in Second (0, 0.25, 0.5, 0.75, 1, 2, 4, 6) [1 to 8] :- 5

 Hold Off Enabled 

 **** Hold Configuration **** 

 1: Enable Hold On

 2: Disable Hold On

 3: Enable Hold Off

 4: Disable Hold Off

 5. Exit from Hold Configuration 

 Enter your choice :- 5

 1. Ramp Configuration 

 2. Step Time Configuration 

 3. Hold Configuration 

 4. Ramp Current Configuration 

 5. Set Linear/Exponential adjustment 

 6. Exit Gradation Configuration 

 Enter your choice :- 4

 Enter the RampUp Current Gain [1 to 255] :- 200

 Final Ramp UP current set Done 

 1. Ramp Configuration 

 2. Step Time Configuration 

 3. Hold Configuration 

 4. Ramp Current Configuration 

 5. Set Linear/Exponential adjustment 

 6. Exit Gradation Configuration 

 Enter your choice :- 5

 1. Linear adjustment for gradation control 

 2. Exponential adjustment for gradation control 

 Enter your choice :- 2

 Gradation Adjustment Done 

 1. Ramp Configuration 

 2. Step Time Configuration 

 3. Hold Configuration 

 4. Ramp Current Configuration 

 5. Set Linear/Exponential adjustment 

 6. Exit Gradation Configuration 

 Enter your choice :- 6

 ********* Gradation Main Menu *********

 1. Gradation Configuration 

 2. Assign LED to Gradation Group 

 3. Enable LED channel for Gradation 

 4. Disable LED channel for Gradation 

 5. Start Gradation 

 6. Stop Gradation 

 7. Exit Gradation Main Menu 

 Enter your choice :- 2

 Enter Gradation Group [1 to 6] :- 5

 Enter LED Number from 1 to 24 :- 4

 LED Assignment Done 

 ********* Gradation Main Menu *********

 1. Gradation Configuration 

 2. Assign LED to Gradation Group 

 3. Enable LED channel for Gradation 

 4. Disable LED channel for Gradation 

 5. Start Gradation 

 6. Stop Gradation 

 7. Exit Gradation Main Menu 

 Enter your choice :- 3

 1. Individual LED Gradation Enable 

 2. All LED Gradation Enable 

 Enter your choice :- 2

 Gradation for All LED Enabled 

 ********* Gradation Main Menu *********

 1. Gradation Configuration 

 2. Assign LED to Gradation Group 

 3. Enable LED channel for Gradation 

 4. Disable LED channel for Gradation 

 5. Start Gradation 

 6. Stop Gradation 

 7. Exit Gradation Main Menu 

 Enter your choice :- 5

 Enter Gradation Group [1 to 6] :- 5

  1. Single Shot Gradation Operation 

  2. Continuous Gradation Operation 

 Enter your choice :- 2

 Gradation Start 

 ********* Gradation Main Menu *********

 1. Gradation Configuration 

 2. Assign LED to Gradation Group 

 3. Enable LED channel for Gradation 

 4. Disable LED channel for Gradation 

 5. Start Gradation 

 6. Stop Gradation 

 7. Exit Gradation Main Menu 

 Enter your choice :- 2

 Enter Gradation Group [1 to 6] :- 5

 Enter LED Number from 1 to 24 :- 3

 LED Assignment Done 

 ********* Gradation Main Menu *********

 1. Gradation Configuration 

 2. Assign LED to Gradation Group 

 3. Enable LED channel for Gradation 

 4. Disable LED channel for Gradation 

 5. Start Gradation 

 6. Stop Gradation 

 7. Exit Gradation Main Menu 

 Enter your choice :- 6

 Enter Gradation Group [1 to 6] :- 5

 Gradation Stop 

 ********* Gradation Main Menu *********

 1. Gradation Configuration 

 2. Assign LED to Gradation Group 

 3. Enable LED channel for Gradation 

 4. Disable LED channel for Gradation 

 5. Start Gradation 

 6. Stop Gradation 

 7. Exit Gradation Main Menu 

 Enter your choice :- 7

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 6

 Enter the LED o/p Delay [1 to 12] :- 4

 LED Delay Set Done 

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 1


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 1

 ALL LED Switched ON


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 5

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 7

 1. Enable Sleep 

 2. Disable Sleep 

 Enter your choice :- 1

 Enabled Sleep 

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 7

 1. Enable Sleep 

 2. Disable Sleep 

 Enter your choice :- 2

 Disable Sleep

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 8

 1. Auto Switch Off Enable 

 2. Auto Switch Off Disable 

 Enter your choice :- 1

 Auto Switch Off Enable 

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 8

 1. Auto Switch Off Enable 

 2. Auto Switch Off Disable 

 Enter your choice :- 2

 Auto Switch Off Disable 

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 9

 1. Set Max Current 30 MA 

 2. Set Max Current 20 MA 

 Enter your choice :- 1

 30 MA maximum Current Set 

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 10

 1. Check Error 

 2. Clear Error 

 3. Check Individual LED Error 

 Enter your choice :- 1

 Error Not Occurred 

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 8

 1. Auto Switch Off Enable 

 2. Auto Switch Off Disable 

 Enter your choice :- 1

 Auto Switch Off Enable 

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 1


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 1

 ALL LED Switched ON


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 5

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 10

 1. Check Error 

 2. Clear Error 

 3. Check Individual LED Error 

 Enter your choice :- 1

 Error Not Occurred 

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 10

 1. Check Error 

 2. Clear Error 

 3. Check Individual LED Error 

 Enter your choice :- 3

 Enter LED Number from 1 to 24 :- 1

 No Error Occurred 

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 1


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 1

 ALL LED Switched ON


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 5

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 8

 1. Auto Switch Off Enable 

 2. Auto Switch Off Disable 

 Enter your choice :- 2

 Auto Switch Off Disable 

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 1


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 1

 ALL LED Switched ON


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 5

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 5

 Reset Done 

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 1


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 1

 ALL LED Switched ON


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 5

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 8

 1. Auto Switch Off Enable 

 2. Auto Switch Off Disable 

 Enter your choice :- 2

 Auto Switch Off Disable 

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 8

 1. Auto Switch Off Enable 

 2. Auto Switch Off Disable 

 Enter your choice :- 1

 Auto Switch Off Enable 

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 1


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 2

 ALL LED Switched OFF


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 1

 ALL LED Switched ON


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 2

 ALL LED Switched OFF


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 1

 ALL LED Switched ON


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 2

 ALL LED Switched OFF


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 1

 ALL LED Switched ON


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 2

 ALL LED Switched OFF


 ******** ALL LED Control ********

 1. ALL LED On 

 2. ALL LED Off 

 3. Set ALL LED Brightness 

 4. Set ALL LED State 

 5. Exit From ALL LED Control 

 Enter your choice :- 5

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 10

 1. Check Error 

 2. Clear Error 

 3. Check Individual LED Error 

 Enter your choice :- 1

 Error Occurred 

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 10

 1. Check Error 

 2. Clear Error 

 3. Check Individual LED Error 

 Enter your choice :- 3

 Enter LED Number from 1 to 24 :- 1

 Short Circuit Occurred 

 *********** Main Menu ***************

 1.  All LED Control 

 2.  Individual LED Control 

 3.  Dimming/Blinking Control 

 4.  Gradation Group Control 

 5.  Reset 

 6.  LED Output Delay (No of Cycle) 

 7.  Sleep Control 

 8.  Auto Sleep off on Error Control 

 9.  Maximum Current Control 

 10. LED Error 

 11. Over Temperature Control 

 Enter your choice :- 11

 Operating in Under Temperature 
