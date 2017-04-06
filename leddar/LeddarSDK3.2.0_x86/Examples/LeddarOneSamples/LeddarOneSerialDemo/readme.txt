==============================================================================
LeddarOneSerialDemo
==============================================================================

This sample can be used as a base to connect to LeddarOne sensors and gather data from them.

---------------------
DESCRIPTION
---------------------

There are 4 layers:

- The OS layer (OS.c and OS.h) consisting of code that is OS dependant (opening the serial port and sending/receiving data, getting info from the keyboard). This is the only layer that has differences between the Windows and Linux code.

- The MODBUS layer (Modbus.c and Modbus.h) that provides a simplistic implementation of the MODBUS protocol using the OS layer functions to access the serial port.

- The Leddar layer (Leddar.c and Leddar.h) that sits above the MODBUS layer and formats the command and interprets the results in the context of a LeddarOne sensor.

- The UI layer (Main.c) that implements a simple text menu system to allow the user to control the sensor.


---------------------
HOW TO BUILD
---------------------

Under Windows, use the Visual Studio project file.
Under Linux, use the makefile provided.


---------------------
HOW TO USE THE SAMPLE
---------------------

When trying to connect to a sensor, a port name is requested.  Enter the COM port, followed by the baudrate and the connection parameters.  For the default serial configuration, it would be like this:

    COM8 115200 8N1


---------------------
SUPPORT
---------------------

Feel free to ask questions to our support staff:

    support@leddartech.com
