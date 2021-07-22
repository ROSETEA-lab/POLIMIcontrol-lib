#include "MatlabDataArray.hpp"
#include "MatlabEngine.hpp"
#include <iostream>
#include <functional>
#include <algorithm>

#include "PID_controller.h"

#define SAMPLING_TIME   0.01
#define KC              20.0
#define TI              10.0
#define UMIN            -100.0
#define UMAX            100.0
#define SIM_CYCLE       500

int main() {
    std::string matlabLine;
    using namespace matlab::engine;

    // Start MATLAB engine synchronously
    std::unique_ptr<MATLABEngine> matlabPtr = startMATLAB();

    // Initialize the MATLAB simulation
    matlabPtr->eval(u"clear all; close all");
    matlabPtr->eval(u"open_system('PID_controller_sim');");
    matlabPtr->eval(u"set_param('PID_controller_sim', 'SaveFinalState', 'on', 'FinalStateName', ['PID_controller_sim' 'SimState'],'SaveCompleteFinalSimState', 'on');");

    matlabLine = "set_param('PID_controller_sim/u','SampleTime','" + std::to_string(SAMPLING_TIME) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('PID_controller_sim/y','SampleTime','" + std::to_string(SAMPLING_TIME) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('PID_controller_sim/ysp','SampleTime','" + std::to_string(SAMPLING_TIME) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('PID_controller_sim/t','SampleTime','" + std::to_string(SAMPLING_TIME) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));

    // Initialize controller
    PID_controller PID(KC, TI, SAMPLING_TIME, UMIN, UMAX);

    // Start the simulation
    matlabLine = "simres = sim('PID_controller_sim', [0 " + std::to_string(SAMPLING_TIME) + "]);";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));

    // Execute the simulation
    std::vector<double> time, measurement, control, reference;

    std::cout << "Executing the simulation..." << std::endl;
    for (auto k=1; k<SIM_CYCLE; k++) {
        // Get data from Matlab
        matlab::data::TypedArray<double> y   = matlabPtr->getVariable(u"y");
        matlab::data::ArrayDimensions y_size = y.getDimensions();

        matlab::data::TypedArray<double> t   = matlabPtr->getVariable(u"t");
        matlab::data::TypedArray<double> u   = matlabPtr->getVariable(u"u");
        matlab::data::TypedArray<double> ysp = matlabPtr->getVariable(u"ysp");

        // Compute PID control law
        double u_act = 0.0;
        double ysp_act = 10.0;
        PID.evaluate(y[y_size.at(0)-1], ysp_act, u_act);

        // Set reference and control variable
        matlabLine = "set_param('PID_controller_sim/u_act','Value','" + std::to_string(u_act) + "');";
        matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));

        matlabLine = "set_param('PID_controller_sim/ysp_act','Value','" + std::to_string(ysp_act) + "');";
        matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));

        // Do next simulation step
        matlabPtr->eval(u"set_param('PID_controller_sim', 'LoadInitialState', 'on', 'InitialState', ['PID_controller_sim' 'SimState']);");
        matlabLine = "simres = sim('PID_controller_sim', [" + std::to_string(k*SAMPLING_TIME) + " " + std::to_string((k+1)*SAMPLING_TIME) + "]);";
        matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));

        // Store simulation data
        time.insert(time.end(), std::begin(t), std::end(t));
        measurement.insert(measurement.end(), std::begin(y), std::end(y));
        control.insert(control.end(), std::begin(u), std::end(u));
        reference.insert(reference.end(), std::begin(ysp), std::end(ysp));
    }
    matlabPtr->eval(u"set_param('PID_controller_sim', 'LoadInitialState', 'off');");
    matlabPtr->eval(u"close_system('PID_controller_sim',0);");

    std::cout << "Test completed, plotting results" << std::endl;

    matlab::data::ArrayFactory factory;
    matlab::data::TypedArray<double> m_time = factory.createArray({time.size(),1}, time.begin(), time.end());
    matlabPtr->setVariable(u"time", std::move(m_time));
    matlab::data::TypedArray<double> m_measurement = factory.createArray({measurement.size(),1}, measurement.begin(), measurement.end());
    matlabPtr->setVariable(u"measurement", std::move(m_measurement));
    matlab::data::TypedArray<double> m_control = factory.createArray({control.size(),1}, control.begin(), control.end());
    matlabPtr->setVariable(u"control", std::move(m_control));
    matlab::data::TypedArray<double> m_reference = factory.createArray({reference.size(),1}, reference.begin(), reference.end());
    matlabPtr->setVariable(u"reference", std::move(m_reference));

    matlabPtr->eval(u"figure,plot(time,measurement, time,reference,'r'),grid,xlabel('Time [s]'),ylabel('Controlled variable / Reference')");
    matlabPtr->eval(u"figure,plot(time,reference-measurement),grid,xlabel('Time [s]'),ylabel('Error')");
    matlabPtr->eval(u"figure,plot(time,control),grid,xlabel('Time [s]'),ylabel('Control signal')");

    // Terminate MATLAB session
    matlab::engine::terminateEngineClient();

	return 0;
}