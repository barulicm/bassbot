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

using namespace std;
using namespace boost::asio;

SerialPort::SerialPort(string device, unsigned int baud)
    : port(ioservice),
      path(device)
{
    try
    {
        port.open(device);
    } catch(...){}
    
    if( !port.is_open() ) {
        cerr << "Could not open serial port " << device << endl;
        return;
    }
    
    try
    {
        port.set_option(boost::asio::serial_port_base::baud_rate(baud));
        port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    } catch(...) {
        cerr << "Could not set options on serial port " << device << endl;
    }
}

SerialPort::~SerialPort()
{
    port.close();
}

bool SerialPort::isOpen()
{
    return port.is_open();
}

void SerialPort::write(string msg)
{
    if(port.is_open())
        boost::asio::write(port, boost::asio::buffer(msg.c_str(),msg.length()));
}

void SerialPort::write(char *buffer, size_t length)
{
    boost::asio::write(port, boost::asio::buffer(buffer, length));
}

void SerialPort::write(unsigned char *buffer, size_t length)
{
    if(port.is_open())
        boost::asio::write(port, boost::asio::buffer(buffer, length));
}

char SerialPort::read()
{
    if(!port.is_open()) return -1;
    
    char in;
    try
    {
        boost::asio::read(port, buffer(&in, 1));
    } catch (boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::system::system_error> >& err) {
        throw std::runtime_error{"End of file"};
    }
    return in;
}

char* SerialPort::read(int numBytes)
{
    if(!port.is_open())
        return (char*)"";
    char* bytes = new char[numBytes];
    for(int i = 0; i < numBytes; i++)
        bytes[i] = read();
    return bytes;
}

string SerialPort::readln()
{
    string line = "";
    while(true)
    {
        char in = read();
        if(in == '\n')
            return line;
        if(in == '\r')
            return line;
        line = line + in;
    }
}

string SerialPort::devicePath()
{
    return path;
}
