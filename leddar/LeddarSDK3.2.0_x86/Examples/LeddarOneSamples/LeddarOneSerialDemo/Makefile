IDIR =.
CC=gcc
CFLAGS=-I$(IDIR) -DLT_LINUX -std=c99 -fmax-errors=1

ODIR=obj
LDIR =lib

LIBS=

_DEPS = Definitions.h LeddarOne.h Modbus.h OS.h
DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ = Main.o LeddarOne.o Modbus.o OS.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))


$(ODIR)/%.o: %.c $(DEPS)
	@mkdir -p $(@D)
	$(CC) -c -o $@ $< $(CFLAGS)

SerialDemo: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ 

