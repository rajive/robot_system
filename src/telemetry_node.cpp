/*
 * telemetry_node: aggregates Robot.SensorData into a UI-friendly
 *                 Robot.TelemetryData snapshot.
 *
 * This keeps the monitoring UI out of the real-time control loop.
 */
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include <dds/dds.hpp>

#include "robot.hpp"

int main()
{
    try {
        dds::core::QosProvider sensor_qos(
            "QoS.xml", "RobotQosLibrary::SensorPeriodic");
        dds::core::QosProvider telemetry_qos(
            "QoS.xml", "RobotQosLibrary::TelemetrySnapshot");

        dds::domain::DomainParticipant participant(0);
        dds::pub::Publisher publisher(participant);
        dds::sub::Subscriber subscriber(participant);

        dds::topic::Topic<SensorData> sensor_topic(participant, "Robot.SensorData");
        dds::topic::Topic<TelemetryData> telemetry_topic(participant, "Robot.TelemetryData");

        dds::sub::DataReader<SensorData> sensor_reader(
            subscriber, sensor_topic, sensor_qos.datareader_qos());
        dds::pub::DataWriter<TelemetryData> telemetry_writer(
            publisher, telemetry_topic, telemetry_qos.datawriter_qos());

        std::mutex mtx;
        std::vector<SensorData> sensor_snapshot;

        /* Drain sensor data into a small ring buffer (keep last 16). */
        std::thread consume_loop([&]() {
            while (true) {
                auto samples = sensor_reader.take();
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    for (const auto& s : samples) {
                        if (s.info().valid()) {
                            sensor_snapshot.push_back(s.data());
                        }
                    }
                    if (sensor_snapshot.size() > 16) {
                        sensor_snapshot.erase(
                            sensor_snapshot.begin(),
                            sensor_snapshot.begin() + (sensor_snapshot.size() - 16));
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
        consume_loop.detach();

        /* Publish aggregated snapshot at 10 Hz for the dashboard. */
        std::cout << "Telemetry aggregator publishing Robot.TelemetryData at 10 Hz"
                  << std::endl;
        while (true) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            TelemetryData snapshot;
            snapshot.entityId = "robot-01";
            snapshot.timestamp =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                    .count();
            {
                std::lock_guard<std::mutex> lock(mtx);
                snapshot.sensorReadings = sensor_snapshot;
            }
            telemetry_writer.write(snapshot);
        }
    } catch (const std::exception& ex) {
        std::cerr << "telemetry_node exception: " << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
