/*
 * actuator_node: consumes Robot.ActuatorCommand, publishes Robot.ActuatorStatus.
 *
 * Usage: actuator_node <actuatorId> <statusPeriodMs>
 *   e.g.  actuator_node motor-left 100
 */
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <thread>

#include <dds/dds.hpp>

#include "robot.hpp"

int main(int argc, char* argv[])
{
    const std::string actuator_id = (argc > 1) ? argv[1] : "motor-left";
    const unsigned status_period_ms = (argc > 2) ? std::atoi(argv[2]) : 100;

    try {
        dds::core::QosProvider command_qos(
            "QoS.xml", "RobotQosLibrary::ActuatorCommandEvent");
        dds::core::QosProvider status_qos(
            "QoS.xml", "RobotQosLibrary::ActuatorStatusState");

        dds::domain::DomainParticipant participant(0);
        dds::pub::Publisher publisher(participant);
        dds::sub::Subscriber subscriber(participant);

        dds::topic::Topic<ActuatorCommand> cmd_topic(participant, "Robot.ActuatorCommand");
        dds::topic::Topic<ActuatorStatus> status_topic(participant, "Robot.ActuatorStatus");

        dds::sub::DataReader<ActuatorCommand> cmd_reader(
            subscriber, cmd_topic, command_qos.datareader_qos());
        dds::pub::DataWriter<ActuatorStatus> status_writer(
            publisher, status_topic, status_qos.datawriter_qos());

        std::mutex mtx;
        ActuatorState state = ActuatorState::IDLE;
        double current_value = 0.0;
        std::string unit = "rev/s";

        std::cout << "Actuator node [" << actuator_id
                  << "] listening for Robot.ActuatorCommand" << std::endl;

        /* Status publisher loop (20 Hz default). */
        std::thread status_loop([&]() {
            while (true) {
                ActuatorStatus status;
                status.actuatorId = actuator_id;
                status.faultCode = 0;
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    status.state = state;
                    status.currentValue = current_value;
                    status.unit = unit;
                }
                status.timestamp =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count();
                status_writer.write(status);
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(status_period_ms));
            }
        });
        status_loop.detach();

        while (true) {
            dds::sub::LoanedSamples<ActuatorCommand> samples = cmd_reader.take();
            for (const auto& sample : samples) {
                if (sample.info().valid()) {
                    const ActuatorCommand& cmd = sample.data();
                    if (cmd.actuatorId != actuator_id) {
                        continue;
                    }
                    std::lock_guard<std::mutex> lock(mtx);
                    switch (cmd.command) {
                        case ActuatorCommandKind::ENABLE:
                            state = ActuatorState::IDLE;
                            std::cout << "[" << actuator_id << "] ENABLE received" << std::endl;
                            break;
                        case ActuatorCommandKind::DISABLE:
                            state = ActuatorState::DISABLED;
                            std::cout << "[" << actuator_id << "] DISABLE received" << std::endl;
                            break;
                        case ActuatorCommandKind::START:
                            state = ActuatorState::ACTIVE;
                            std::cout << "[" << actuator_id << "] START received" << std::endl;
                            break;
                        case ActuatorCommandKind::STOP:
                            state = ActuatorState::IDLE;
                            std::cout << "[" << actuator_id << "] STOP received" << std::endl;
                            break;
                        case ActuatorCommandKind::SET_VALUE:
                            current_value = cmd.targetValue;
                            unit = cmd.unit;
                            std::cout << "[" << actuator_id << "] SET_VALUE to "
                                      << current_value << " " << unit << std::endl;
                            break;
                        default:
                            std::cout << "[" << actuator_id << "] unsupported command" << std::endl;
                            break;
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    } catch (const std::exception& ex) {
        std::cerr << "actuator_node exception: " << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
