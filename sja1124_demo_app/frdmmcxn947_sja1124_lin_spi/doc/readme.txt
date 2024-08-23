Hardware requirements
===================
- Mini/micro C USB cable
- FRDM-MCXN947 board
- SJA1124EVB-ARD board 
- Personal Computer

Board settings:
1) Connect the FRDM-MCXN947 and SJA1124EVB with arduino header.
2) Apply external supply voltage (5V - 6V) either on VBAT pin of LIN header or by external 5V power adapter.
3) Use LIN pins on LIN header to establish the LIN communication with the responder device.
 
Note:
Settings for selection of EDMA/Interrupt Mode for both SPI:
SPI: Set RTE_SPI1_DMA_EN to 1 for EDMA mode and RTE_SPI1_DMA_EN to 0 for Interrupt mode in board/RTE_Device.h

Note: SJA1124EVB can only be configured as a LIN Master. It cant be configured as a LIN Slave.
 

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
 =~=~=~=~=~=~=~=~=~=~=~= PuTTY log 2024.07.23 11:52:04 =~=~=~=~=~=~=~=~=~=~=~=

  ISSDK SJA1124 SPI to Quad LIN Bridge driver example demonstration.

 All LIN Channels initialized with default configurations.

 Baud Rate-20000, 1 Stop Bit, 1-bit Delimiter, 10 bits Commander Break Length and Hardware Checksum Enabled.


*********** Main Menu ***************

1. Send/Receive LIN Frame via LIN Channel 1

2. Send/Receive LIN Frame via LIN Channel 2

3. Send/Receive LIN Frame via LIN Channel 3

4. Send/Receive LIN Frame via LIN Channel 4

5. Change Baud Rate

6. Enable Checksum

7. Set Stop Bit

8. Set Delimiter

9. Set Commander Break Length

10. Enter Low Power Mode

11. Reset SJA1124EVB

12. SJA1124EVB Device Status

13. SJA1124EVB Device ID

14. Read any register

*************************************

Enter your choice :-

1) Choose options #1 to #4 to Send/Receive LIN Frame via respective LIN Channel

- Press #1 to Send/Receive LIN Frame via LIN Channel 1

- Send/Receive LIN Frame provides two options:

       - Send LIN Frame
       - Receive LIN Frame 

   - Refer below logs:

    - To Send LIN Frame : Enter number of bytes to send and data bytes from Commander to Responder.

		1. Send LIN Frame
		2. Receive LIN Frame
		  
        Enter your choice :- 1

		********************************
		
		Enter no of bytes to send (1 to 8):- 5
		
		Data[0] = 0x1
		
		Data[1] = 0x23
		
		Data[2] = 0x43
		
		Data[3] = 0x54
		
		Data[4] = 0x65
		
		********************************
		
		Data is sent Successfully from Commander to Responder.
      

       2. Receive LIN Frame
       - To Receive LIN Frame: Enter number of bytes to receive from Responder to Commander.

        	1. Send LIN Frame

			2. Receive LIN Frame
			
			Enter your choice :- 2
			
			********************************
			
			Enter no of bytes to receive (1 to 8): 4
			
			Data received from the responder:
			
			Data[0] =  0xdd
			
			Data[1] =  0xcc
			
			Data[2] =  0xbb
			
			Data[3] =  0xaa
			
			Checksum = 0xeb
			
			********************************
       

  Note: 
  - Same size of LIN frames should be send/receive via both commander and responder.
  
2) Choose option #5 to change baud rate for respective LIN Channel
 
Press #1 to #4 to select specific LIN Channel 
********************************

Select LIN Channel

1. LIN Channel 1

2. LIN Channel 2

3. LIN Channel 3

4. LIN Channel 4

Enter your choice :- 2

LIN Channel 2 Selected

********************************

Enter Channel Baud Rate in Hz (upto 20000Hz):- 10000

Baud Rate Changed Successfully.

3) Choose option #6 to enable hardware/software checksum for respective LIN Channel

********************************

Select LIN Channel

1. LIN Channel 1

2. LIN Channel 2

3. LIN Channel 3

4. LIN Channel 4

Enter your choice :- 1

LIN Channel 1 Selected

********************************

Currently Hardware Checksum is enabled.

1. Hardware Checksum

2. Software Checksum

Enter your choice :- 2

********************************

Enabled Software Checksum.

********************************

4) Choose option #7 to set stop bit for respective LIN Channel

********************************

Select LIN Channel

1. LIN Channel 1

2. LIN Channel 2

3. LIN Channel 3

4. LIN Channel 4

Enter your choice :- 2

LIN Channel 2 Selected

********************************

Currently Stop Bit = 1

1. One stop bit

2. Two stop bit

Enter your choice :- 2

********************************

Stop Bit is set to 2.

********************************

5) Choose option #8 to set delimiter for respective LIN Channel

********************************

Select LIN Channel

1. LIN Channel 1

2. LIN Channel 2

3. LIN Channel 3

4. LIN Channel 4

Enter your choice :- 4

LIN Channel 4 Selected

********************************

Current Delimiter = 2 bit

1. 1-bit Delimiter

2. 2-bit Delimiter

Enter your choice :- 1

********************************

Delimiter is set to 1 bit.

********************************

6) Choose option #9 to set commander break length for respective LIN Channel

********************************

Select LIN Channel

1. LIN Channel 1

2. LIN Channel 2

3. LIN Channel 3

4. LIN Channel 4

Enter your choice :- 3

LIN Channel 3 Selected

********************************

Currently Commander Break Length = 13

Select any one Commander Break Length

1. 10 bit

2. 11 bit

3. 12 bit

4. 13 bit

5. 14 bit

6. 15 bit

7. 16 bit

8. 17 bit

9. 18 bit

10. 19 bit

11. 20 bit

12. 21 bit

13. 22 bit

14. 23 bit

15. 36 bit

16. 50 bit

Enter your choice :- 1

********************************

Commander Break Length is set to 10 Bit

********************************

7) Choose option #10 for device to enter into low power mode

SPI wake-up:

Enter your choice :- 10

 Successfully entered into Low Power Mode

********************************

Choose any one option to exit

1. Wake-up via SPI

2. Wake-up via LIN Channel

Enter your choice :- 1

********************************

 Successfully exit from Low Power Mode via SPI Wake-up.

********************************

LIN wake-up:

 Successfully entered into Low Power Mode

********************************

Choose any one option to exit

1. Wake-up via SPI

2. Wake-up via LIN Channel

Enter your choice :- 2

********************************

waiting for LIN wake-up request via responder to exit from low power mode........

 Successfully exit from Low Power Mode via LIN Wakeup.

********************************

8) Choose option #11 to reset SJA1124EVB

********************************

 Device reset is done successfully.


 All LIN Channels initialized with default configurations.

 Baud Rate-20000, 1 Stop Bit, 1-bit Delimiter, 10 bits Commander Break Length and Hardware Checksum Enabled.

********************************

9) Choose option #12 to read SJA1124EVB Device status

*************************************

Enter your choice :- 12

1. PLL lock status

2. Over-temperature warning

3. PLL input frequency status

********************************

Enter your choice :- 1

********************************

PLL in lock

********************************

Note: Similarly over-temperature warning and PLL input frequency status can be checked.

10) Choose option #13 to read SJA1124EVB Device ID

********************************

Device ID = 19

********************************

11) Choose option #14 to read any SJA1124EVB device register

*************************************

Enter your choice :- 14

Enter register address in HEX :- 0x13

Data = 0x8

********************************

Note 1: This test application will bridge SPI to quad LIN communication in between Commander and Responder.
Different options are available to change the configuration settings of bridge and device to operate
in different mode. 

Note 2: SJA1124_SPI_Initialize() API should be called first in order to use other APIs for differnt device features.
User can refer sja1124_drv.h header file for more information.

