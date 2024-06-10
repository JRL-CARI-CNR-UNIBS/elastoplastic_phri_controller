#ifndef ELASTOPLASTIC_LUGRE_CONTROLLER_HPP
#define ELASTOPLASTIC_LUGRE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <controller_interface/chainable_controller_interface.hpp>

namespace elastoplastic {

class ElastoplasticController : public controller_interface::ChainableControllerInterface
{
public:
    ElastoplasticController();


    controller_interface::InterfaceConfiguration command_interface_configuration() const override;


    controller_interface::InterfaceConfiguration state_interface_configuration() const override;


    controller_interface::return_type update_and_write_commands(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;


    controller_interface::CallbackReturn on_init() override;


    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;


    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;


    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;


    controller_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;


    controller_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State & previous_state) override;


    controller_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State & previous_state) override;
protected:
    std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

    controller_interface::return_type update_reference_from_subscribers() override;
};
}

#endif // ELASTOPLASTIC_LUGRE_CONTROLLER_HPP
