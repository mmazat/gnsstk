# $Id$

OBJS=cmatrix.o cplot.o cycle_slip.o geodesy.o gps.o ionosphere.o kiss_fft.o navigation.o novatel.o rinex.o sem.o time_conversion.o troposphere.o yuma.o

all: geodesy

libgnsstk.a: $(OBJS)
	ar cru $@ $(OBJS)
	ranlib $@

geodesy: libgnsstk.a
	$(CC) $(CFLAGS) -o $@ geodesy_main.c -lm ./libgnsstk.a

clean:
	rm -f *.o *.a *.core geodesy

