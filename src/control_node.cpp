/*
 * control_node: consumes Robot.SensorData + Robot.ActuatorStatus,
 *               publishes Robot.ActuatorCommand.
 *
 * Simple closed-loop example: every 100 ms it issues SET_VALUE to an actuator,
 * steering the actuator target toward the latest sensor reading.
 *
 * Usage: control_node <actuatorId> <sensorId>
 */
#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

#include <dds/dds.hpp>

#include "robot.hpp"

int main(int argc, char* argv[])
{
    const std::string actuator_id = (argc > 1) ? argv[1] : "motor-left";
    const std::string sensor_id = (argc > 2) ? argv[2] : "lidar-01";

    try {
        dds::core::QosProvider sensor_qos(
            "QoS.xml", "RobotQosLibrary::SensorPeriodic");
        dds::core::QosProvider cmd_qos(
            "QoS.xml", "RobotQosLibrary::ActuatorCommandEvent");
        dds::core::QosProvider status_qos(
            "QoS.xml", "RobotQosLibrary::ActuatorStatusState");

        dds::domain::DomainParticipant participant(0);
        dds::pub::Publisher publisher(participant);
        dds::sub::Subscriber subscriber(participant);

        dds::topic::Topic<SensorData> sensor_topic(participant, "Robot.SensorData");
        dds::topic::Topic<ActuatorCommand> cmd_topic(participant, "Robot.ActuatorCommand");
        dds::topic::Topic<ActuatorStatus> status_topic(participant, "Robot.ActuatorStatus");

        dds::sub::DataReader<SensorData> sensor_reader(
            subscriber, sensor_topic, sensor_qos.datareader_qos());
        dds::sub::DataReader<ActuatorStatus> status_reader(
            subscriber, status_topic, status_qos.datareader_qos());
        dds::pub::DataWriter<ActuatorCommand> cmd_writer(
            publisher, cmd_topic, cmd_qos.datawriter_qos());

        std::atomic<double> latest_sensor{0.0};
        std::atomic<bool> sensor_seen{false};
        std::atomic<ActuatorState> actuator_state{ActuatorState::UNKNOWN};

        /* Consumer thread: drains sensor and status samples. */
        std::thread consume_loop([&]() {
            while (true) {
                auto sensor_samples = sensor_reader.take();
                for (const auto& s : sensor_samples) {
                    if (s.info().valid() && s.data().sensorId == sensor_id) {
                        latest_sensor.store(s.data().value);
                        sensor_seen.store(true);
                    }
                }

                auto status_samples = status_reader.take();
                for (const auto& st : status_samples) {
                    if (st.info().valid() && st.data().actuatorId == actuator_id) {
                        actuator_state.store(st.data().state);
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
        consume_loop.detach();

        std::cout << "Control node steering [" << actuator_id << "] from sensor ["
                  << sensor_id << "]" << std::endl;

        uint64_t cmd_id = 0;
        while (true) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            if (!sensor_seen.load()) {
                std::cout << "No sensor data yet; skipping control tick" << std::endl;
                continue;
            }

            ActuatorCommand cmd;
            cmd.actuatorId = actuator_id;
            cmd.commandId = ++cmd_id;
            cmd.command = ActuatorCommandKind::SET_VALUE;
            cmd.targetValue = latest_sensor.load();
            cmd.unit = "m";
            cmd.timestamp =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                    .count();
            cmd_writer.write(cmd);
        }
    } catch (const std::exception& ex) {
        std::cerr << "control_node exception: " << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
