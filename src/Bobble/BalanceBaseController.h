#ifndef BALANCE_ROBOT_CONTROL_BALANCE_CONTROLLER_H
#define BALANCE_ROBOT_CONTROL_BALANCE_CONTROLLER_H

#include <cstddef>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <thread>
#include <mutex>

#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>
#include "BalanceControllerData.h"
//#include <balance_robot_control/gainConfig.h>
//#include <dynamic_reconfigure/server.h>

class BalanceBaseController {
    public:
        BalanceBaseController(){};
        ~BalanceBaseController(){};
        void init(ros::NodeHandle &nh);
        void reset();
        void update();
        int getControlMode() { return state.ActiveControlMode; };

    protected:
        ros::NodeHandle node_;
        bool run_thread_;
        ros::Subscriber sub_command_;
        std::mutex mutex_;
	    std::thread* subscriber_thread_;
        BalanceControllerConfig   config;
        BalanceControllerCommands received_commands;
        BalanceControllerCommands processed_commands;
        BalanceControllerState    state;
        BalanceControllerOutputs  outputs;
        BalanceControllerFilters  filters;
        BalancePIDControllers     pid_controllers;
        virtual void estimateState() = 0;
        virtual void sendMotorCommands() = 0;
        virtual void loadConfig();
        void setupFilters();
        void setupControllers();
        void runStateLogic();
        void idleMode();
        void diagnosticMode();
        void startupMode();
        void balanceMode();
        void unpackParameter(std::string parameterName, double &referenceToParameter, double defaultValue);
        void unpackParameter(std::string parameterName, float &referenceToParameter, float defaultValue);
        void unpackParameter(std::string parameterName, int &referenceToParameter, int defaultValue);
        void unpackParameter(std::string parameterName, std::string &referenceToParameter, std::string defaultValue);
        void unpackFlag(std::string parameterName, bool &referenceToFlag, bool defaultValue);
        double limit(double cmd, double max);

        //dynamic_reconfigure::Server<balance_robot_control::gainConfig> param_server_;
        //dynamic_reconfigure::Server<balance_robot_control::gainConfig>::CallbackType callback_;
        //void param_callback(const balance_robot_control::gainConfig& config, uint32_t level);

    private:
        void populateImuData();
        void clearCommandState(BalanceControllerCommands& cmds);
        void populateCommands();
        void applyFilters();
        void applySafety();
};
#endif