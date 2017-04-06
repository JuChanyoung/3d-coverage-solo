==============================================================================
MultiChannelLibModbusDemo
==============================================================================

This sample uses the libmodbus open source library.
For more information about libmodbus, go to http://libmodbus.org/.

To support the custom Leddar command 0x41, we had to modify the library. 
However, the modification is not required if Modbus registers are being used.
Function changed: compute_data_length_after_meta() in "modbus.c".


---------------------
HOW TO BUILD
---------------------

Windows:
	Use the provided Visual Studio solution.
	
Linux:
	- Build the libmodbus library, found in ../extern/libmodbus*.
		- Run 'configure', then 'make' and 'make install'.
	- Run 'make' in the MultiChannelLibModbusDemo directory.


---------------------
HOW TO USE THE SAMPLE
---------------------

The serial port must be a complete path, e.g. "/dev/ttyXXXX".

---------------------
SUPPORT
---------------------

Feel free to ask questions to our support staff:

    support@leddartech.com

