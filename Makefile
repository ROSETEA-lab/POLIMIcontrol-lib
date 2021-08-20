PREFIX = /usr/local

CCLNFLAGS = -lm -lpthread -lmatio

CCLNDIRS = 

CCOPT = -m64 -O -fPIC -fno-strict-aliasing -fexceptions -DNDEBUG -DIL_STD

CCFLAGS = $(CCOPT) -I/usr/include/eigen3 -I./

CCC = g++ -O0 -std=c++11

CC = gcc

libPOLIMIControl.so:
	$(CCC) -c $(CCFLAGS) FIR_filter.cpp -o FIR_filter.o
	$(CCC) -c $(CCFLAGS) discrete_tf.cpp -o discrete_tf.o
	$(CCC) -c $(CCFLAGS) discrete_ss.cpp -o discrete_ss.o
	$(CCC) -c $(CCFLAGS) continuous_ss.cpp -o continuous_ss.o
	$(CCC) -c $(CCFLAGS) ./controller/PID_controller.cpp -o PID_controller.o
	$(CCC) -c $(CCFLAGS) ./controller/AGS_controller.cpp -o AGS_controller.o
	$(CCC) -c $(CCFLAGS) ./util/matfile_fun.cpp -o matfile_fun.o
	$(CCC) -shared -Wl,-soname,libPOLIMIControl.so -o libPOLIMIControl.so FIR_filter.o discrete_tf.o discrete_ss.o continuous_ss.o PID_controller.o AGS_controller.o matfile_fun.o

all: libPOLIMIControl.so

clean:
	@rm -rf *.o *.so

install:
	@mkdir -p $(DESTDIR)$(PREFIX)/lib
	@mkdir -p $(DESTDIR)$(PREFIX)/include/POLIMIcontrol
	@cp libPOLIMIControl.so $(DESTDIR)$(PREFIX)/lib
	@cp ./controller/PID_controller.h $(DESTDIR)$(PREFIX)/include/POLIMIcontrol
	@cp ./controller/AGS_controller.h $(DESTDIR)$(PREFIX)/include/POLIMIcontrol
	@cp ./util/matfile_fun.h $(DESTDIR)$(PREFIX)/include/POLIMIcontrol
	@cp FIR_filter.h $(DESTDIR)$(PREFIX)/include/POLIMIcontrol
	@cp discrete_tf.h $(DESTDIR)$(PREFIX)/include/POLIMIcontrol
	@cp discrete_ss.h $(DESTDIR)$(PREFIX)/include/POLIMIcontrol
	@cp continuous_ss.h $(DESTDIR)$(PREFIX)/include/POLIMIcontrol

uninstall:
	@rm -f $(DESTDIR)$(PREFIX)/lib/libControl.so
	@rm -f $(DESTDIR)$(PREFIX)/include/POLIMIcontrol/FIR_filter.h
	@rm -f $(DESTDIR)$(PREFIX)/include/POLIMIcontrol/discrete_tf.h
	@rm -f $(DESTDIR)$(PREFIX)/include/POLIMIcontrol/discrete_ss.h
	@rm -f $(DESTDIR)$(PREFIX)/include/POLIMIcontrol/continuous_ss.h
	@rm -f $(DESTDIR)$(PREFIX)/include/POLIMIcontrol/PID_controller.h
	@rm -f $(DESTDIR)$(PREFIX)/include/POLIMIcontrol/AGS_controller.h
	@rm -f $(DESTDIR)$(PREFIX)/include/POLIMIcontrol/matfile_fun.h

