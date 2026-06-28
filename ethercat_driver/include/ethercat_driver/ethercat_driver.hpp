// Copyright 2022 ICUBE Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Modified by wang.xinfei  2026-06-27
// modify the code to support: one slave has multiple components
// but not support Transfer ...  


#ifndef ETHERCAT_DRIVER__ETHERCAT_DRIVER_HPP_
#define ETHERCAT_DRIVER__ETHERCAT_DRIVER_HPP_

#include <unordered_map>
#include <memory>
#include <string>
#include <vector>
#include <pluginlib/class_loader.hpp>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ethercat_driver/visibility_control.h"
#include "ethercat_interface/ec_slave.hpp"
#include "ethercat_interface/ec_master.hpp"
#include "yaml-cpp/yaml.h"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ethercat_driver
{

class EthercatDriver : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EthercatDriver)

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  ETHERCAT_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ETHERCAT_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ETHERCAT_DRIVER_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  ETHERCAT_DRIVER_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

protected:
  std::vector<std::unordered_map<std::string, std::string>> getEcModuleParam(
    const std::string & urdf,
    const std::string & component_name,
    const std::string & component_type);

  uint16_t getAliasOrDefaultAlias(
    const std::unordered_map<std::string,
    std::string> & slave_parameters);

  virtual CallbackReturn setupMaster();

  CallbackReturn configNetwork();

  /** @brief Load transfer config YAML file
   * One use case is to load transfers for FailSafe Over EtherCAT Safety
   * @param[out] node YAML node containing the transfer configuration root
   * @param[in] path Path to the YAML file, if empty, the file is loaded from the *fsoe_config*
   * or *transfer_config* of the YAML document
   */
  void loadTransferConfigYamlFile(YAML::Node & node, const std::string & path = "");

  /** @brief Get transfer module parameters from YAML file
   * @param[in] config YAML node containing the transfer configuration root
   * @return Vector of maps containing transfer module parameters, each map corresponds to a module
   * involved in a transfer
   */
  std::vector<std::unordered_map<std::string, std::string>> getEcTransferModuleParam(
    const YAML::Node & config);

  /** @brief Get transfer nets from YAML file
   * @param[in] config YAML node containing the transfer configuration root
   * @return Vector of transfer nets
   */
  std::vector<ethercat_interface::EcTransferNet> getEcTransferNets(const YAML::Node & config);

  /** @brief Configure the transfer networks
   */
  void configTransferNetwork();

protected:
  std::vector<std::shared_ptr<ethercat_interface::EcSlave>> ec_modules_;
  std::vector<std::unordered_map<std::string, std::string>> ec_module_parameters_;

  std::vector<std::vector<double>> hw_joint_commands_;
  std::vector<std::vector<double>> hw_sensor_commands_;
  std::vector<std::vector<double>> hw_gpio_commands_;
  std::vector<std::vector<double>> hw_commands_all_;

  std::vector<std::vector<std::string>> hw_command_prefix_names_;
  std::vector<std::vector<std::string>> hw_command_interface_names_;

  std::vector<std::vector<double>> hw_joint_states_;
  std::vector<std::vector<double>> hw_sensor_states_;
  std::vector<std::vector<double>> hw_gpio_states_;
  std::vector<std::vector<double>> hw_states_all_;

  std::vector<std::vector<std::string>> hw_state_prefix_names_;
  std::vector<std::vector<std::string>> hw_state_interface_names_;


  std::vector<std::unordered_map<std::string, std::string>> module_param_map_;

  // pluginlib class loader for loading EcSlave plugins
  pluginlib::ClassLoader<ethercat_interface::EcSlave> ec_loader_{
    "ethercat_interface", "ethercat_interface::EcSlave"};

  double control_frequency_;

  std::shared_ptr<ethercat_interface::EcMaster> master_;
  std::mutex ec_mutex_;
  bool activated_;

  /** Transfer nets */
  std::vector<ethercat_interface::EcTransferNet> ec_transfer_nets_;

  /** Indexes of modules inside ec_modules_ vector that are transfer masters */
  std::vector<size_t> ec_transfer_masters_;
  /** Indexes of modules inside ec_modules_ vector that are transfer slaves only */
  std::vector<size_t> ec_transfer_slaves_;

  /** Empty interfaces */
  std::vector<double> empty_interface_;

  uint32_t state_interface_offsite_;
  uint32_t command_interface_offsite_;

  enum IndexType : uint32_t{
    TYPE_JOINT = 0,
    TYPE_SENSOR = 1,
    TYPE_GPIO = 2
  };
  enum SlaveType : uint32_t{
    RESUSE_SLAVE = 0,
    NEW_SLAVE = 1
  };
  struct IndexItem{
    IndexType type;
    uint32_t sub_index;
    std::vector<std::unordered_map<uint,SlaveType>> slave_type_map;
    // uint slave_position;
    // SlaveType slave_type;
  };
  std::vector<IndexItem> total_index_;
};
}  // namespace ethercat_driver

#endif  // ETHERCAT_DRIVER__ETHERCAT_DRIVER_HPP_
