//
// Created by zhenz on 2022/1/23.
//

#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace UNITREE_LEGGED_SDK;
namespace py = pybind11;

// PYBIND11_MAKE_OPAQUE(std::vector<float>);

class UnitreeUDPWrapper {
 public:
  UnitreeUDPWrapper(): ros_udp(UNITREE_LEGGED_SDK::LOWLEVEL), safe(LeggedType::Aliengo),
    proc_action(12),
    motor_torques(12, 0.0),
    loop_control("control_loop", 0.002, boost::bind(&UnitreeUDPWrapper::RobotControl, this))
  {
    ros_udp.udpState.RecvCount = 0;
    ros_udp.udpState.SendCount = 0;

    float init_kp = 10.0;
    float init_kd = 1.0;

    ros_udp.InitCmdData(cmd_udp);
    cmd_udp.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;
    for(int i = 0; i<12; i++){
      cmd_udp.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
      cmd_udp.motorCmd[i].q = PosStopF;        // 禁止位置环
      cmd_udp.motorCmd[i].Kp = 0;
      cmd_udp.motorCmd[i].dq = VelStopF;        // 禁止速度环
      cmd_udp.motorCmd[i].Kd = 0;
      cmd_udp.motorCmd[i].tau = 0;
    }
    ros_udp.SetSend(cmd_udp);
    ros_udp.Send();
    // loop_control.start();
  }

  void UDPSend() {
    ros_udp.Send();
  }

  void UDPRecv() {
    ros_udp.Recv();
  }

  std::vector<float> getUnitreeUDP() {
    std::vector<float> state;
    ros_udp.Recv();
    ros_udp.GetRecv(state_udp);
    // quaternion
    state.push_back(state_udp.imu.quaternion[0]);
    state.push_back(state_udp.imu.quaternion[1]);
    state.push_back(state_udp.imu.quaternion[2]);
    state.push_back(state_udp.imu.quaternion[3]);
    // gyroscope
    state.push_back(state_udp.imu.gyroscope[0]);
    state.push_back(state_udp.imu.gyroscope[1]);
    state.push_back(state_udp.imu.gyroscope[2]);
    // motor
    for (int i = 0; i < 12; i++) {
      state.push_back(state_udp.motorState[i].q);
      state.push_back(state_udp.motorState[i].dq);
      state.push_back(state_udp.motorState[i].tauEst);
    }
    // foot force
    state.push_back(state_udp.footForce[0]);
    state.push_back(state_udp.footForce[1]);
    state.push_back(state_udp.footForce[2]);
    state.push_back(state_udp.footForce[3]);
    return state;
  }

  void sendUnitreeUDP(std::vector<float> &cmd, int mode) {
    int len = cmd.size();
    assert(len==12);
    if (cmd[1] == 0 && cmd[2] == 0) {
      for(int i = 0; i<12; i++) {
        cmd_udp.motorCmd[i].q = PosStopF;        // 禁止位置环
        cmd_udp.motorCmd[i].dq = VelStopF;        // 禁止速度环
        cmd_udp.motorCmd[i].Kp = 0;
        cmd_udp.motorCmd[i].Kd = 0;
        cmd_udp.motorCmd[i].tau = 0;
      }
      // ros_udp.udpState.SendCount++;
      // ros_udp.SetSend(cmd_udp);
      // ros_udp.Send();
      return;
    }
    for (int i = 0; i < 12; i++) {
      if (mode == 1) {
        // position mode
        cmd_udp.motorCmd[i].q = cmd[i];
        cmd_udp.motorCmd[i].dq = 0.0;
        cmd_udp.motorCmd[i].Kp = 150.0;
        cmd_udp.motorCmd[i].Kd = 4.0;
        cmd_udp.motorCmd[i].tau = 0.0;
      }
      else if (mode == 2) {
        // torque mode
        cmd_udp.motorCmd[i].q = PosStopF;
        cmd_udp.motorCmd[i].dq = VelStopF;
        cmd_udp.motorCmd[i].Kp = 0.0;
        cmd_udp.motorCmd[i].Kd = 0.0;
        cmd_udp.motorCmd[i].tau = cmd[i];
      }
    }
    safe.PowerProtect(cmd_udp, state_udp, 5);
    ros_udp.SetSend(cmd_udp);
    ros_udp.Send();
  }

  void setStepAction(std::vector<float> &action) {
    step_action = action;
    std::cout << "----------" << std::endl;
    std::cout << "set step action: " << step_action[2] << std::endl;
  }

  void RobotControl() {
    if (!step_action.empty()) {
      last_action = update_action;
      update_action = step_action;
      step_action.clear();
      substep_count = 0;
      std::cout << "update action: " << update_action[2] << std::endl;
    }
    if (update_action.empty() || last_action.empty()
        || substep_count > 10) {
      motor_torques = std::vector<float>(12,0);
    }
    else {
      // std::cout << "substep_count = " << substep_count << std::endl;
      ProcessAction(update_action, substep_count, &proc_action);
      // std::cout << "proc action: " << proc_action[2] << std::endl;
      ApplyAction(proc_action, &motor_torques);
      // std::cout << "motor torque: " << motor_torques[2] << std::endl;
      substep_count++;
    }

    for (int i = 0; i < 12; i++) {
      cmd_udp.motorCmd[i].q = PosStopF;
      cmd_udp.motorCmd[i].dq = VelStopF;
      cmd_udp.motorCmd[i].Kp = 0.0;
      cmd_udp.motorCmd[i].Kd = 0.0;
      cmd_udp.motorCmd[i].tau = 0.0;
      cmd_udp.motorCmd[i].tau = motor_torques[i];
    }
    safe.PowerProtect(cmd_udp, state_udp, 5);
    ros_udp.SetSend(cmd_udp);
    ros_udp.Send();
    ros_udp.Recv();
    ros_udp.GetRecv(state_udp);
  }

 private:
  LoopFunc loop_control;
  UNITREE_LEGGED_SDK::UDP ros_udp;
  UNITREE_LEGGED_SDK::Safety safe;
  UNITREE_LEGGED_SDK::LowCmd cmd_udp;
  UNITREE_LEGGED_SDK::LowState state_udp;
  std::vector<float> step_action;
  std::vector<float> last_action;
  std::vector<float> update_action;
  std::vector<float> proc_action;
  std::vector<float> last_motor_commands;
  std::vector<float> motor_torques;
  int substep_count;

  void ProcessAction(std::vector<float>& action, int &substep_count, std::vector<float>* proc_action) {
    if (!last_action.empty()) {
      float lerp = float(substep_count + 1) / 5;
      for (int i = 0; i < 12 ; i++) {
        (*proc_action)[i] = last_action[i] + lerp * (action[i] - last_action[i]);
      }
    }
  }

  void ApplyAction(std::vector<float>& motor_commands, std::vector<float>* motor_torques) {
    float kp = 180.0;
    float kd = 1.0;
    float t = 0.0;
    std::cout << "motor_state: " << state_udp.motorState[2].q << " " << state_udp.motorState[2].dq << std::endl;
    for (int i = 0; i < 12 ; i++) {
      float motor_angle = state_udp.motorState[i].q, motor_velocity = state_udp.motorState[i].dq;
      float desired_motor_velocity = 0.0;
      if (!last_motor_commands.empty()) {
        desired_motor_velocity = (motor_commands[i] - last_motor_commands[i]) / 0.002;
      }
      t = kp * (motor_commands[i] - motor_angle) + kd * (desired_motor_velocity - motor_velocity);
      if (t > 30.0)
        (*motor_torques)[i] = 30.0;
      else if (t < -30.0)
        (*motor_torques)[i] = -30.0;
      else
        (*motor_torques)[i] = t;
    }
    // last_motor_commands.assign(motor_commands.begin(), motor_commands.end());
    last_motor_commands = motor_commands;
  }
};

PYBIND11_MODULE(unitree_udp, m) {
  // py::class_<std::vector<int>>(m, "IntVector")
  //     .def(py::init<>())
  //     .def("clear", &std::vector<int>::clear)
  //     .def("pop_back", &std::vector<int>::pop_back)
  //     .def("__len__", [](const std::vector<int> &v) { return v.size(); })
  //     .def("__iter__", [](std::vector<int> &v) {
  //       return py::make_iterator(v.begin(), v.end());
  //     }, py::keep_alive<0, 1>());
  py::class_<UnitreeUDPWrapper>(m, "UnitreeUDPWrapper")
    .def(py::init<>())
    .def("send", &UnitreeUDPWrapper::sendUnitreeUDP)
    .def("recv", &UnitreeUDPWrapper::getUnitreeUDP)
    .def("stepAction", &UnitreeUDPWrapper::setStepAction);
}
