/*
 * telemetry_ui: subscribes to Robot.TelemetryData and prints the latest
 *               dashboard snapshot every second.
 */
#include <chrono>
#include <iostream>
#include <thread>

#include <dds/dds.hpp>

#include "robot.hpp"

int main()
{
    try {
        dds::core::QosProvider telemetry_qos(
            "QoS.xml", "RobotQosLibrary::TelemetrySnapshot");

        dds::domain::DomainParticipant participant(0);
        dds::sub::Subscriber subscriber(participant);

        dds::topic::Topic<TelemetryData> telemetry_topic(participant, "Robot.TelemetryData");

        dds::sub::DataReader<TelemetryData> telemetry_reader(
            subscriber, telemetry_topic, telemetry_qos.datareader_qos());

        std::cout << "Telemetry UI listening on Robot.TelemetryData" << std::endl;

        while (true) {
            dds::sub::LoanedSamples<TelemetryData> samples = telemetry_reader.take();
            for (const auto& s : samples) {
                if (s.info().valid()) {
                    const TelemetryData& t = s.data();
                    std::cout << "[" << t.entityId << "] snapshot @ "
                              << t.timestamp << " with "
                              << t.sensorReadings.size() << " readings";
                    if (!t.sensorReadings.empty()) {
                        std::cout << " (last value "
                                  << t.sensorReadings.back().value << ")";
                    }
                    std::cout << std::endl;
                }
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    } catch (const std::exception& ex) {
        std::cerr << "telemetry_ui exception: " << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
