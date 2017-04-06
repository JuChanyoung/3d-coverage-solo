IDIR =..
IDIR2 = ../..
DEFINES=LT_LINUX

CFLAGS=$(addprefix -I,$(IDIR))  $(addprefix -I,$(IDIR2)) $(addprefix -D,$(DEFINES)) -std=gnu99 -fmax-errors=1
CXXFLAGS=$(addprefix -I,$(IDIR))  $(addprefix -I,$(IDIR2)) $(addprefix -D,$(DEFINES)) -std=gnu++11 -fmax-errors=1 -pthread

LIBS=modbus
LDFLAGS=$(addprefix -l,$(LIBS))

ODIR=obj

_OBJ = MultiChannelLibModbusDemo.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))

all: MultiChannelLibModbusDemo

$(ODIR)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) -c -o $@ $< $(CXXFLAGS)

$(ODIR)/%.o: %.c
	@mkdir -p $(@D)
	$(CC) -c -o $@ $< $(CFLAGS)

MultiChannelLibModbusDemo: $(OBJ)
	$(CXX) -o $@ $^ $(CXXFLAGS) $(LDFLAGS)

.PHONY: clean all

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ 

