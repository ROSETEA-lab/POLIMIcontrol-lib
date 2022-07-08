PREFIX = /usr/local

CCLNFLAGS = -lm -lpthread -lmatio

CCLNDIRS = 

CCOPT = -m64 -O -fPIC -fno-strict-aliasing -fexceptions -DNDEBUG -DIL_STD

CCFLAGS = $(CCOPT) -I/usr/include/eigen3 -I./ -I./util

CCC = g++ -O0 -std=c++11

CC = gcc

libPOLIMIcontrol.so:
	$(CCC) -c $(CCFLAGS) FIR_filter.cpp -o FIR_filter.o
	$(CCC) -c $(CCFLAGS) discrete_tf.cpp -o discrete_tf.o
	$(CCC) -c $(CCFLAGS) discrete_integrator.cpp -o discrete_integrator.o
	$(CCC) -c $(CCFLAGS) discrete_derivative.cpp -o discrete_derivative.o
	$(CCC) -c $(CCFLAGS) discrete_ss.cpp -o discrete_ss.o
	$(CCC) -c $(CCFLAGS) continuous_ss.cpp -o continuous_ss.o
	$(CCC) -c $(CCFLAGS) differentiator.cpp -o differentiator.o
	$(CCC) -c $(CCFLAGS) ./controller/PID_controller.cpp -o PID_controller.o
	$(CCC) -c $(CCFLAGS) ./controller/AGS_controller.cpp -o AGS_controller.o
	$(CCC) -c $(CCFLAGS) ./util/matfile_fun.cpp -o matfile_fun.o
	$(CCC) -shared -Wl,-soname,libPOLIMIcontrol.so -o libPOLIMIcontrol.so FIR_filter.o discrete_tf.o discrete_integrator.o discrete_derivative.o discrete_ss.o continuous_ss.o differentiator.o PID_controller.o AGS_controller.o matfile_fun.o

all: libPOLIMIcontrol.so

clean:
	@rm -rf *.o *.so

install:
	@mkdir -p $(DESTDIR)$(PREFIX)/lib
	@mkdir -p $(DESTDIR)$(PREFIX)/include/POLIMIcontrol
	@cp libPOLIMIcontrol.so $(DESTDIR)$(PREFIX)/lib
	@cp ./controller/PID_controller.h $(DESTDIR)$(PREFIX)/include/POLIMIcontrol
	@cp ./controller/AGS_controller.h $(DESTDIR)$(PREFIX)/include/POLIMIcontrol
	@cp ./util/matfile_fun.h $(DESTDIR)$(PREFIX)/include/POLIMIcontrol
	@cp FIR_filter.h $(DESTDIR)$(PREFIX)/include/POLIMIcontrol
	@cp discrete_tf.h $(DESTDIR)$(PREFIX)/include/POLIMIcontrol
	@cp discrete_integrator.h $(DESTDIR)$(PREFIX)/include/POLIMIcontrol
	@cp discrete_derivative.h $(DESTDIR)$(PREFIX)/include/POLIMIcontrol
	@cp discrete_ss.h $(DESTDIR)$(PREFIX)/include/POLIMIcontrol
	@cp continuous_ss.h $(DESTDIR)$(PREFIX)/include/POLIMIcontrol
	@cp differentiator.h $(DESTDIR)$(PREFIX)/include/POLIMIcontrol

uninstall:
	@rm -f $(DESTDIR)$(PREFIX)/lib/libPOLIMIcontrol.so
	@rm -f $(DESTDIR)$(PREFIX)/include/POLIMIcontrol/FIR_filter.h
	@rm -f $(DESTDIR)$(PREFIX)/include/POLIMIcontrol/discrete_tf.h
	@rm -f $(DESTDIR)$(PREFIX)/include/POLIMIcontrol/discrete_integrator.h
	@rm -f $(DESTDIR)$(PREFIX)/include/POLIMIcontrol/discrete_derivative.h
	@rm -f $(DESTDIR)$(PREFIX)/include/POLIMIcontrol/discrete_ss.h
	@rm -f $(DESTDIR)$(PREFIX)/include/POLIMIcontrol/continuous_ss.h
	@rm -f $(DESTDIR)$(PREFIX)/include/POLIMIcontrol/differentiator.h
	@rm -f $(DESTDIR)$(PREFIX)/include/POLIMIcontrol/PID_controller.h
	@rm -f $(DESTDIR)$(PREFIX)/include/POLIMIcontrol/AGS_controller.h
	@rm -f $(DESTDIR)$(PREFIX)/include/POLIMIcontrol/matfile_fun.h

