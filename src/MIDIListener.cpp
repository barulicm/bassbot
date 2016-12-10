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
#include <RtMidi.h>
#include <bassbot/MidiNote.h>

std::array<std::string, 12> midiNoteNames = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};

ros::Publisher midiNotePublisher;

unsigned int sequence_number = 0;

ros::Time midiStamp;

void midiCallback(double deltatime, std::vector<unsigned char> *message, void *)
{
    if(midiStamp.isZero()) {
        // Initialize time stamp to current time for first message
        midiStamp = ros::Time::now();
    }
    midiStamp += ros::Duration(deltatime);

    switch(message->at(0)) {
        case 144: {
            unsigned char note_number = message->at(1);
            unsigned char note_velocity = message->at(2);
            std::string note_name = midiNoteNames[note_number % midiNoteNames.size()];
            unsigned char note_octave = note_number / (unsigned char)12;
            ROS_DEBUG_STREAM("MIDI Note: " << note_name << " " << note_octave << "\tVelocity: " << note_velocity);
            bassbot::MidiNote msg;
            msg.header.stamp = midiStamp;
            msg.header.seq = sequence_number;
            sequence_number++;
            msg.note = note_number;
            msg.velocity = note_velocity;
            midiNotePublisher.publish(msg);
            break;
        }
        default: {
            size_t nBytes = message->size();
            std::stringstream ss;
            ss << "Unhandled MIDI Message: ";
            for ( unsigned int i=0; i<nBytes; i++ )
                ss << "Byte " << i << " = " << (int)message->at(i) << ", ";
            if ( nBytes > 0 )
                ss << "stamp = " << deltatime;
            ROS_INFO_STREAM(ss.str());
            break;
        }
    }
}

std::unique_ptr<RtMidiIn> openMIDIInput(unsigned int port_number, RtMidiIn::RtMidiCallback &&callback) {
    auto device = std::make_unique<RtMidiIn>();

    ROS_DEBUG_STREAM("Opening port # " << port_number << "\tname: " << device->getPortName(0));

    device->openPort(port_number);

    device->setCallback(std::move(callback));

    device->ignoreTypes(false, false, true);

    return device;
}

int getMIDIPortNumberFromName(std::string port_name)
{
    std::unique_ptr<RtMidiIn> midiin = std::make_unique<RtMidiIn>();

    unsigned int nPorts = midiin->getPortCount();
    for ( unsigned int i=0; i<nPorts; i++ ) {
        if(midiin->getPortName(i) == port_name) {
            return i;
        }
    }
    return -1;
}

bool isMIDIInputAvailable() {
    try {
        std::unique_ptr<RtMidiIn> midiin = std::make_unique<RtMidiIn>();

        unsigned int nPorts = midiin->getPortCount();
        ROS_DEBUG_STREAM("There are " << nPorts << " MIDI input sources available.");
        std::string portName;
        for ( unsigned int i=0; i<nPorts; i++ ) {
            portName = midiin->getPortName(i);
            ROS_DEBUG_STREAM("Input Port #" << i+1 << ": " << portName);
        }
        return (nPorts > 0);
    }
    catch(RtMidiError &error) {
        error.printMessage();
        return false;
    }
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "MIDIListener");

    if(!isMIDIInputAvailable()) {
        ROS_ERROR("No MIDI input device detected.");
        return 1;
    }

    auto port_number = getMIDIPortNumberFromName("Digital Piano 24:0");

    if(port_number < 0) {
        ROS_ERROR("Could not find port number for device \"Digital Piano 24:0\"");
        return -1;
    }

    auto midiDevice = openMIDIInput(static_cast<unsigned int>(port_number), &midiCallback);

    ros::NodeHandle handle;

    midiNotePublisher = handle.advertise<bassbot::MidiNote>("/midi", 1);

    ros::spin();

    return 0;
}