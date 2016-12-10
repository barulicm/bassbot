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
#include <bassbot/CallFoundNotification.h>
#include <bassbot/MidiNote.h>
#include <bassbot/PatternUtilities.h>

Pattern call_pattern;

size_t downbeat_index;

ros::Time firstNoteStamp;
ros::Time lastNoteStamp;
ros::Time downbeatNoteStamp;

size_t currentMatchIndex = 0;

ros::Publisher patternPublisher;

unsigned int seq = 0;

float call_duration;

void midi_callback(const bassbot::MidiNoteConstPtr &msg)
{
    if(msg->velocity == 0) {
        // Ignore note off messages
        return;
    }

    if(msg->note == call_pattern[currentMatchIndex].first) {
        if(currentMatchIndex == 0) {
            firstNoteStamp = msg->header.stamp;
        }
        if(currentMatchIndex == downbeat_index) {
            downbeatNoteStamp = msg->header.stamp;
        }

        currentMatchIndex++;

        if(currentMatchIndex == call_pattern.size()) {
            lastNoteStamp = msg->header.stamp;
            bassbot::CallFoundNotification notification;
            notification.header.stamp = ros::Time::now();
            notification.header.seq = seq++;
            auto call_time = static_cast<float>((lastNoteStamp - firstNoteStamp).toSec() / 60.0f);
            notification.tempo = call_duration / call_time;
            notification.downbeat = downbeatNoteStamp;
            patternPublisher.publish(notification);
            currentMatchIndex = 0;
        }
    } else {
        currentMatchIndex = 0;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "CallFinder");

    ros::NodeHandle handle{};
    ros::NodeHandle pHandle{"~"};

    std::string patternPath = pHandle.param("patternPath", std::string{"call_pattern.csv"});

    downbeat_index = static_cast<size_t>(pHandle.param<int>("downbeatIndex", 0));

    call_pattern = loadPattern(patternPath);

    call_duration = std::accumulate(call_pattern.begin(), call_pattern.end()-1, 0.0f, [](float total, std::pair<int, double> x){
        return total + x.second;
    });

    auto subscriber = handle.subscribe("/midi", 10, &midi_callback);

    patternPublisher = handle.advertise<bassbot::CallFoundNotification>("/callFound", 1);

    ros::spin();

    return 0;
}