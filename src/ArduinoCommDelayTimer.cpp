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

#include <bassbot/SerialPort.h>
#include <iostream>
#include <sys/time.h>

__useconds_t communication_delay = 1'500;
__useconds_t servo_delay = 0;

using timestamp_t = unsigned long long;

static timestamp_t get_timestamp()
{
    struct timeval now;
    gettimeofday(&now, nullptr);
    return now.tv_usec + static_cast<timestamp_t>(now.tv_sec * 1'000'000);
}


int main()
{
    SerialPort port{"/dev/ttyUSB0", 115200};

    if(!port.isOpen()) {
        std::cerr << "Unable to open port " << "/dev/ttyUSB0" << std::endl;
        return 1;
    }

    usleep(1'500'000);

    constexpr size_t trials = 1000;

    std::array<double, trials> data;

    for(size_t i = 0; i < trials && port.isOpen(); i++) {
        auto t0 = get_timestamp();
        port.write("Longer test message oh no!\n");
        usleep(2800);
        std::string response;
        try {
            port.readln();
        } catch(const std::runtime_error &err) {
            if(err.what() != std::string{"End of file"}){
                std::cerr << err.what() << std::endl;
            }
        }
        auto t1 = get_timestamp();

        auto triptime = (t1 - t0) / 2.0L;
        data[i] = triptime;
    }

    auto total = std::accumulate(data.begin(), data.end(), 0.0L);
    auto average = total / trials;

    std::cout << "Average trip time: " << average << " microseconds" << std::endl;

    return 0;
}