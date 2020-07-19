CCLNFLAGS = -lstdc++ -leng -lMatlabEngine -lMatlabDataArray -lmx -lmat -lm -lpthread -ldl -lControl

CCLNDIRS = -L/usr/local/MATLAB/R2020a/bin/glnxa64 -L/usr/local/MATLAB/R2020a/extern/bin/glnxa64

CCOPT = -m64 -fexceptions -fPIC -fno-omit-frame-pointer -fno-strict-aliasing -pthread -O2 -fwrapv -DNDEBUG -DIL_STD

CCFLAGS = $(CCOPT) -I/usr/local/MATLAB/R2020a/extern/include -I/usr/local/MATLAB/R2020a/simulink/include -I/usr/include/eigen3

CCC = g++ -O0 -std=c++11

CC = gcc

test_discrete_tf: test_discrete_tf.o
	$(CC) $(CCFLAGS) $(CCLNDIRS) -o test_discrete_tf test_discrete_tf.o $(CCLNFLAGS)
test_discrete_tf.o: test_discrete_tf.cpp
	$(CC) -c $(CCFLAGS) test_discrete_tf.cpp -o test_discrete_tf.o

test_discrete_ss: test_discrete_ss.o
	$(CC) $(CCFLAGS) $(CCLNDIRS) -o test_discrete_ss test_discrete_ss.o $(CCLNFLAGS)
test_discrete_ss.o: test_discrete_ss.cpp
	$(CC) -c $(CCFLAGS) test_discrete_ss.cpp -o test_discrete_ss.o

test_FIR_filter: test_FIR_filter.o
	$(CC) $(CCFLAGS) $(CCLNDIRS) -o test_FIR_filter test_FIR_filter.o $(CCLNFLAGS)
test_FIR_filter.o: test_FIR_filter.cpp
	$(CC) -c $(CCFLAGS) test_FIR_filter.cpp -o test_FIR_filter.o

test_PID_controller: test_PID_controller.o
	$(CC) $(CCFLAGS) $(CCLNDIRS) -o test_PID_controller test_PID_controller.o $(CCLNFLAGS)
test_PID_controller.o: test_PID_controller.cpp
	$(CC) -c $(CCFLAGS) test_PID_controller.cpp -o test_PID_controller.o

all: test_discrete_tf test_discrete_ss test_FIR_filter test_PID_controller

clean:
	@rm -rf *~ *.o
	@rm -rf test_discrete_tf test_discrete_ss test_FIR_filter test_PID_controller
