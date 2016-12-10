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
#include <bassbot/PatternUtilities.h>
#include <bassbot/Note.h>

Pattern response_pattern;

double callPatternBeatsDuration;

ros::Publisher notePublisher;

enum MODE {
    STRAIGHT,
    SLOW,
    RAMP
} mode;

double tempoForBar(int bar, double tempoReceived) {
    switch(mode) {
        default:
        case STRAIGHT:
            return tempoReceived;
        case SLOW:
            return tempoReceived - 20;
        case RAMP:
            return tempoReceived - (5 * bar);
    }
}

void callFoundCallback(const bassbot::CallFoundNotificationConstPtr &msg)
{
    auto tempoReceived = msg->tempo;
    auto downbeatStamp = msg->downbeat;

    ros::Duration callPatternDuration{(callPatternBeatsDuration / tempoReceived) * 60.0};
    ros::Time responsePatternDownbeat = downbeatStamp + callPatternDuration;

    ros::Time stamp = responsePatternDownbeat;
    double beats = 0.0;
    for (const auto &note : response_pattern) {
        auto note_number = note.first;
        auto duration = note.second;
        auto duration_secs = ( duration / tempoForBar((int)(beats / 4), tempoReceived) ) * 60.0;
        bassbot::Note noteMsg;
        noteMsg.header.stamp = ros::Time::now();
        noteMsg.note = note_number;
        noteMsg.stamp = stamp;
        notePublisher.publish(noteMsg);
        beats += duration;
        stamp += ros::Duration{duration_secs};
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ResponsePlayer");

    ros::NodeHandle handle;
    ros::NodeHandle pHandle{"~"};

    auto callPattern = loadPattern(pHandle.param("call_pattern_path", std::string{"call_pattern.csv"}));
    auto callPatternDownbeatInd = pHandle.param("call_pattern_downbeat_index", 0);
    response_pattern = loadPattern(pHandle.param("response_pattern_path", std::string{"response_pattern.csv"}));

    ROS_INFO_STREAM("Response pattern length: " << response_pattern.size());

    auto modeString = pHandle.param("mode", std::string{"straight"});
    if(modeString == "straight")
        mode = STRAIGHT;
    else if(modeString == "slow")
        mode = SLOW;
    else if(modeString == "ramp")
        mode = RAMP;

    callPatternBeatsDuration = std::accumulate(callPattern.begin()+callPatternDownbeatInd, callPattern.end(), 0.0f,
                                               [](const float &total, const PatternBeat &beat) {
                                                   return total + beat.second;
                                               });

    /* Call pattern duration should be a whole number of
     * beats so response timestamp happens on a beat.
     */
    callPatternBeatsDuration = ceil(callPatternBeatsDuration);

    auto subscriber = handle.subscribe("/callFound", 1, &callFoundCallback);

    notePublisher = handle.advertise<bassbot::Note>("/play", static_cast<uint32_t>(response_pattern.size()));

    ros::spin();

    return 0;
}