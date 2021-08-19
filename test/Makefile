CCLNFLAGS = -lstdc++ -leng -lMatlabEngine -lMatlabDataArray -lmx -lmat -lm -lpthread -ldl -lControl -lpython2.7

CCLNDIRS = -L/usr/local/MATLAB/R2021a/bin/glnxa64 -L/usr/local/MATLAB/R2021a/extern/bin/glnxa64

CCOPT = -m64 -fexceptions -fPIC -fno-omit-frame-pointer -fno-strict-aliasing -pthread -O2 -fwrapv -DNDEBUG -DIL_STD

CCFLAGS = $(CCOPT) -I/usr/local/MATLAB/R2021a/extern/include -I/usr/local/MATLAB/R2021a/simulink/include -I/usr/include/eigen3 -I/usr/include/python2.7

CCC = g++ -O0 -std=c++11

CC = gcc

test_discrete_tf: test_discrete_tf.o
	$(CC) $(CCFLAGS) $(CCLNDIRS) -o test_discrete_tf test_discrete_tf.o $(CCLNFLAGS)
test_discrete_tf.o: test_discrete_tf.cpp
	$(CC) -c $(CCFLAGS) test_discrete_tf.cpp -o test_discrete_tf.o

test_discrete_ss_ti: test_discrete_ss_ti.o
	$(CC) $(CCFLAGS) $(CCLNDIRS) -o test_discrete_ss_ti test_discrete_ss_ti.o $(CCLNFLAGS)
test_discrete_ss_ti.o: test_discrete_ss_ti.cpp
	$(CC) -c $(CCFLAGS) test_discrete_ss_ti.cpp -o test_discrete_ss_ti.o

test_continuous_ss_ti: test_continuous_ss_ti.o
	$(CC) $(CCFLAGS) $(CCLNDIRS) -o test_continuous_ss_ti test_continuous_ss_ti.o $(CCLNFLAGS)
test_continuous_ss_ti.o: test_continuous_ss_ti.cpp
	$(CC) -c $(CCFLAGS) test_continuous_ss_ti.cpp -o test_continuous_ss_ti.o

test_continuous_ss_ti_single: test_continuous_ss_ti_single.o
	$(CC) $(CCFLAGS) $(CCLNDIRS) -o test_continuous_ss_ti_single test_continuous_ss_ti_single.o $(CCLNFLAGS)
test_continuous_ss_ti_single.o: test_continuous_ss_ti_single.cpp
	$(CC) -c $(CCFLAGS) test_continuous_ss_ti_single.cpp -o test_continuous_ss_ti_single.o

test_discrete_ss_tv: test_discrete_ss_tv.o
	$(CC) $(CCFLAGS) $(CCLNDIRS) -o test_discrete_ss_tv test_discrete_ss_tv.o $(CCLNFLAGS)
test_discrete_ss_tv.o: test_discrete_ss_tv.cpp
	$(CC) -c $(CCFLAGS) test_discrete_ss_tv.cpp -o test_discrete_ss_tv.o

test_continuous_ss_tv: test_continuous_ss_tv.o
	$(CC) $(CCFLAGS) $(CCLNDIRS) -o test_continuous_ss_tv test_continuous_ss_tv.o $(CCLNFLAGS)
test_continuous_ss_tv.o: test_continuous_ss_tv.cpp
	$(CC) -c $(CCFLAGS) test_continuous_ss_tv.cpp -o test_continuous_ss_tv.o

test_continuous_ss_tv_single: test_continuous_ss_tv_single.o
	$(CC) $(CCFLAGS) $(CCLNDIRS) -o test_continuous_ss_tv_single test_continuous_ss_tv_single.o $(CCLNFLAGS)
test_continuous_ss_tv_single.o: test_continuous_ss_tv_single.cpp
	$(CC) -c $(CCFLAGS) test_continuous_ss_tv_single.cpp -o test_continuous_ss_tv_single.o

test_discrete_ss_lpv: test_discrete_ss_lpv.o
	$(CC) $(CCFLAGS) $(CCLNDIRS) -o test_discrete_ss_lpv test_discrete_ss_lpv.o $(CCLNFLAGS)
test_discrete_ss_lpv.o: test_discrete_ss_lpv.cpp
	$(CC) -c $(CCFLAGS) test_discrete_ss_lpv.cpp -o test_discrete_ss_lpv.o

test_continuous_ss_lpv: test_continuous_ss_lpv.o
	$(CC) $(CCFLAGS) $(CCLNDIRS) -o test_continuous_ss_lpv test_continuous_ss_lpv.o $(CCLNFLAGS)
test_continuous_ss_lpv.o: test_continuous_ss_lpv.cpp
	$(CC) -c $(CCFLAGS) test_continuous_ss_lpv.cpp -o test_continuous_ss_lpv.o

test_FIR_filter: test_FIR_filter.o
	$(CC) $(CCFLAGS) $(CCLNDIRS) -o test_FIR_filter test_FIR_filter.o $(CCLNFLAGS)
test_FIR_filter.o: test_FIR_filter.cpp
	$(CC) -c $(CCFLAGS) test_FIR_filter.cpp -o test_FIR_filter.o

test_PID_controller: test_PID_controller.o
	$(CC) $(CCFLAGS) $(CCLNDIRS) -o test_PID_controller test_PID_controller.o $(CCLNFLAGS)
test_PID_controller.o: test_PID_controller.cpp
	$(CC) -c $(CCFLAGS) test_PID_controller.cpp -o test_PID_controller.o

PID_controller_sim: PID_controller_sim.o
	$(CC) $(CCFLAGS) $(CCLNDIRS) -o PID_controller_sim PID_controller_sim.o $(CCLNFLAGS)
PID_controller_sim.o: PID_controller_sim.cpp
	$(CC) -c $(CCFLAGS) PID_controller_sim.cpp -o PID_controller_sim.o

PID_controller_sim_odeint: PID_controller_sim_odeint.o
	$(CC) $(CCFLAGS) $(CCLNDIRS) -o PID_controller_sim_odeint PID_controller_sim_odeint.o $(CCLNFLAGS)
PID_controller_sim_odeint.o: PID_controller_sim_odeint.cpp
	$(CC) -c $(CCFLAGS) PID_controller_sim_odeint.cpp -o PID_controller_sim_odeint.o

all: test_discrete_tf test_discrete_ss_ti test_continuous_ss_ti test_continuous_ss_ti_single test_discrete_ss_tv\
     test_continuous_ss_tv test_continuous_ss_tv_single test_discrete_ss_lpv test_continuous_ss_lpv\
     test_FIR_filter test_PID_controller PID_controller_sim PID_controller_sim_odeint

clean:
	@rm -rf *~ *.o
	@rm -rf test_discrete_tf test_discrete_ss_ti test_continuous_ss_ti test_continuous_ss_ti_single test_discrete_ss_tv\
	        test_continuous_ss_tv test_continuous_ss_tv_single test_discrete_ss_lpv test_continuous_ss_lpv test_FIR_filter\
            test_PID_controller PID_controller_sim PID_controller_sim_odeint

