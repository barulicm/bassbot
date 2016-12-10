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
#include <sound_play/sound_play.h>
#include <queue>
#include <bassbot/Note.h>

using sound_request = std::pair<uint8_t, ros::Time>;

std::map<uint8_t, sound_play::Sound*> sounds;

bool pq_comp(const sound_request &a, const sound_request &b)
{
    return a.second > b.second;
};

std::priority_queue<sound_request, std::vector<sound_request>, decltype(&pq_comp)> soundQueue{&pq_comp};

constexpr size_t minNoteValue = 41;
constexpr size_t maxNoteValue = 60;

void noteCallback(const bassbot::NoteConstPtr &msg)
{
    soundQueue.push(sound_request{msg->note, msg->stamp});
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "synth_interface");

    ros::NodeHandle handle;
    ros::NodeHandle pHandle{"~"};

    sound_play::SoundClient soundClient;

    for(size_t i = minNoteValue; i < maxNoteValue; i++) {
        sounds[i] = new sound_play::Sound(soundClient.waveSoundFromPkg("bassbot", "sounds/synthbass_"+std::to_string(i)+".wav"));
    }

    auto subscriber = handle.subscribe("/play", 16, &noteCallback);

    ros::Time t_now;

    ros::Rate rate{60};
    while(ros::ok()) {
        ros::spinOnce();

        t_now = ros::Time::now();

        while( !soundQueue.empty() && (soundQueue.top().second - t_now).toSec() <= 0.0 ) {
            sounds[soundQueue.top().first]->play();
            soundQueue.pop();
        }

        rate.sleep();
    }

    return 0;
}