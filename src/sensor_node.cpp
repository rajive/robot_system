/*
 * sensor_node: publishes periodic Robot.SensorData samples.
 *
 * Usage: sensor_node <sensorId> <kind> <unit> <periodMs>
 *   e.g.  sensor_node lidar-01 POSITION m 20
 *         sensor_node imu-01  ACCELERATION m/s^2 10
 */
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

#include <dds/dds.hpp>

#include "robot.hpp"

int main(int argc, char* argv[])
{
    const std::string sensor_id = (argc > 1) ? argv[1] : "lidar-01";
    const std::string kind_str = (argc > 2) ? argv[2] : "POSITION";
    const std::string unit = (argc > 3) ? argv[3] : "m";
    const unsigned period_ms = (argc > 4) ? std::atoi(argv[4]) : 20;

    try {
        dds::core::QosProvider qos_provider(
            "QoS.xml", "RobotQosLibrary::SensorPeriodic");

        dds::domain::DomainParticipant participant(0);
        dds::pub::Publisher publisher(participant);

        dds::topic::Topic<SensorData> sensor_topic(participant, "Robot.SensorData");

        dds::pub::DataWriter<SensorData> writer(
            publisher, sensor_topic, qos_provider.datawriter_qos());

        SensorData sample;
        sample.sensorId = sensor_id;
        sample.kind = SensorKind::POSITION;
        sample.unit = unit;

        if (kind_str == "TEMPERATURE") {
            sample.kind = SensorKind::TEMPERATURE;
        } else if (kind_str == "VELOCITY") {
            sample.kind = SensorKind::VELOCITY;
        } else if (kind_str == "ACCELERATION") {
            sample.kind = SensorKind::ACCELERATION;
        }

        std::cout << "Sensor node [" << sensor_id << "] publishing Robot.SensorData "
                  << "every " << period_ms << " ms" << std::endl;

        while (true) {
            sample.value = 50.0 + static_cast<double>(std::rand() % 100) / 10.0;
            sample.timestamp =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                    .count();
            writer.write(sample);
            std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
        }
    } catch (const std::exception& ex) {
        std::cerr << "sensor_node exception: " << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
