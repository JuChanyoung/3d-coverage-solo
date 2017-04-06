# Makefile for Linux version of Leddar SDK.
# Copyrigth 2015 LeddarTech Inc.

#********************************************************
# IMPORTANT
# Specify the QT5PATH variable to your QT5 lib path.
#********************************************************

include ./Definitions.mif

Options = $(OptionsBase) -I../../Includes
Solution = ../../Release/
Output = Release/
Module := $(Solution)LeddarCDemo
QT5PATH = /usr/local/Qt-5.4.2/lib

Base := Main
Base := $(Base:%=$(Output)%.o)

Tout := $(Base)

$(Module) : $(Output) $(Tout)
	gcc -o$(Module) -pthread -L../../Release -L$(QT5PATH) $(Tout) -lLeddarC -lLeddar -lLeddarTech -lQt5SerialPort -lstdc++ -Wl,-rpath=.

$(Output)%.o : %.c Makefile ./Definitions.mif
	$(Compiler) $< -o$@ $(Options)

$(Output) :
	mkdir -p $(Output)

-include $(Tout:.o=.d)
