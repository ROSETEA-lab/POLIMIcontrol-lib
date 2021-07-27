PREFIX = /usr/local

CCLNFLAGS = -lm -lpthread

CCLNDIRS = 

CCOPT = -m64 -O -fPIC -fno-strict-aliasing -fexceptions -DNDEBUG -DIL_STD

CCFLAGS = $(CCOPT) -I/usr/include/eigen3

CCC = g++ -O0 -std=c++11

CC = gcc

libControl.so: FIR_filter.cpp discrete_tf.cpp PID_controller.cpp
	$(CCC) -c $(CCFLAGS) FIR_filter.cpp -o FIR_filter.o
	$(CCC) -c $(CCFLAGS) discrete_tf.cpp -o discrete_tf.o
	$(CCC) -c $(CCFLAGS) discrete_ss.cpp -o discrete_ss.o
	$(CCC) -c $(CCFLAGS) continuous_ss.cpp -o continuous_ss.o
	$(CCC) -c $(CCFLAGS) PID_controller.cpp -o PID_controller.o
	$(CCC) -shared -Wl,-soname,libControl.so -o libControl.so FIR_filter.o discrete_tf.o discrete_ss.o continuous_ss.o PID_controller.o

all: libControl.so

clean:
	@rm -rf *.o *.so

install:
	@mkdir -p $(DESTDIR)$(PREFIX)/lib
	@mkdir -p $(DESTDIR)$(PREFIX)/include
	@cp libControl.so $(DESTDIR)$(PREFIX)/lib
	@cp FIR_filter.h $(DESTDIR)$(PREFIX)/include
	@cp discrete_tf.h $(DESTDIR)$(PREFIX)/include
	@cp discrete_ss.h $(DESTDIR)$(PREFIX)/include
	@cp continuous_ss.h $(DESTDIR)$(PREFIX)/include
	@cp PID_controller.h $(DESTDIR)$(PREFIX)/include

uninstall:
	@rm -f $(DESTDIR)$(PREFIX)/lib/libControl.so
	@rm -f $(DESTDIR)$(PREFIX)/include/FIR_filter.h
	@rm -f $(DESTDIR)$(PREFIX)/include/discrete_tf.h
	@rm -f $(DESTDIR)$(PREFIX)/include/discrete_ss.h
	@rm -f $(DESTDIR)$(PREFIX)/include/continuous_ss.h
	@rm -f $(DESTDIR)$(PREFIX)/include/PID_controller.h

