/* MIT License
 *
 * Copyright (c) 2016 Matthew Barulic
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <ros/ros.h>
#include <bassbot/SerialPort.h>
#include <bassbot/Note.h>
#include <sensor_msgs/BatteryState.h>
#include <queue>

constexpr double communication_delay = 1'500e-6; // using 115200 baud rate
constexpr double servo_delay = 0e-6;
constexpr double fret_rate = 0'500'000e-6;

struct gesture_request {
    ros::Time stamp;
    virtual void execute(SerialPort &port) const = 0;
};

struct fret_gesture_request : public gesture_request {
    int fret;
    virtual void execute(SerialPort &port) const override {
        port.write("f" + std::to_string(fret) + "\n");
    }
};

struct string_gesture_request : public gesture_request {
    int string;
    virtual void execute(SerialPort &port) const override {
        port.write("s" + std::to_string(string) + "\n");
    }
};

std::map<int, int> string_map = {
        {41, 1}, {42, 1}, {43, 1}, {44, 1}, {45, 1},
        {46, 2}, {47, 2}, {48, 2}, {49, 2}, {50, 2},
        {51, 3}, {52, 3}, {53, 3}, {54, 3}, {55, 3},
        {56, 4}, {57, 4}, {58, 4}, {59, 4}, {60, 4}
};

std::map<int, int> fret_map = {
        {41, 1}, {42, 2}, {43, 3}, {44, 4}, {45, 5},
        {46, 1}, {47, 2}, {48, 3}, {49, 4}, {50, 5},
        {51, 1}, {52, 2}, {53, 3}, {54, 4}, {55, 5},
        {56, 1}, {57, 2}, {58, 3}, {59, 4}, {60, 5}
};

template<typename T>
bool pq_comp(const T &a, const T &b)
{
    return a->stamp > b->stamp;
}

std::priority_queue<gesture_request*, std::vector<gesture_request*>, decltype(&pq_comp<gesture_request *>)> gestureQueue{
        &pq_comp};
int lastFretInQueue = 0;

void noteCallback(const bassbot::NoteConstPtr &msg)
{
    fret_gesture_request *fretGestureRequest = new fret_gesture_request;
    fretGestureRequest->fret = fret_map[msg->note];
    fretGestureRequest->stamp = msg->stamp - ros::Duration{communication_delay + (fret_rate * abs(fretGestureRequest->fret - lastFretInQueue))};
    gestureQueue.push(dynamic_cast<gesture_request*>(fretGestureRequest));
    lastFretInQueue = fretGestureRequest->fret;

    string_gesture_request *stringGestureRequest = new string_gesture_request;
    stringGestureRequest->string = string_map[msg->note];
    stringGestureRequest->stamp = msg->stamp - ros::Duration{communication_delay + servo_delay};
    gestureQueue.push(dynamic_cast<gesture_request*>(stringGestureRequest));
}

float getBatteryVoltage(SerialPort &port) {
    port.write("b\n");
    usleep(2'800);
    try {
        auto response = port.readln();
        return std::stof(response);
    } catch(const std::runtime_error &err) {
        return 0.0f;
    } catch(const std::invalid_argument &err) {
        return 0.0f;
    }
}

void publishBatteryVoltage(float voltage, ros::Publisher &publisher) {
    sensor_msgs::BatteryState batteryStateMsg;
    batteryStateMsg.current = NAN;
    batteryStateMsg.charge = NAN;
    batteryStateMsg.design_capacity = NAN;
    batteryStateMsg.percentage = NAN;
    batteryStateMsg.power_supply_status = batteryStateMsg.POWER_SUPPLY_STATUS_UNKNOWN;
    batteryStateMsg.power_supply_health = batteryStateMsg.POWER_SUPPLY_HEALTH_UNKNOWN;
    batteryStateMsg.power_supply_technology = batteryStateMsg.POWER_SUPPLY_TECHNOLOGY_NICD;
    batteryStateMsg.present = 1;
    batteryStateMsg.voltage = voltage;

    publisher.publish(batteryStateMsg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_interface");

    ros::NodeHandle handle;
    ros::NodeHandle pHandle{"~"};

    auto devPath = pHandle.param("dev", std::string{"/dev/ttyACM0"});

    SerialPort port{devPath, static_cast<unsigned int>(pHandle.param("baud", 115200))};

    if(!port.isOpen()) {
        ROS_ERROR_STREAM("Unable to open port " << devPath);
        return 1;
    }

    // Give the Arduino some time to boot up before we talk to it
    usleep(1'500'000);

    auto noteSubscriber = handle.subscribe("/play", 16, &noteCallback);

    auto batteryPublisher = handle.advertise<sensor_msgs::BatteryState>("/battery_state", 1, true);

    ros::Time t_now;

    ros::Time lastBattRequest = ros::Time::now();
    ros::Rate rate{60};
    while(ros::ok()) {
        ros::spinOnce();

        t_now = ros::Time::now();

        while( !gestureQueue.empty() && (gestureQueue.top()->stamp - t_now).toSec() <= 0) {
            auto gesture = gestureQueue.top();
            gesture->execute(port);
            gestureQueue.pop();
            delete gesture;
        }

        if( (t_now - lastBattRequest).toSec() >= 10.0 ) {
            publishBatteryVoltage(getBatteryVoltage(port), batteryPublisher);
            lastBattRequest = t_now;
        }

        rate.sleep();
    }

    return 0;
}