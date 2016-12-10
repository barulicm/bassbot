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

#pragma once
#include <string>
#include <boost/asio.hpp>

/*!
 * \brief Helper class to simplify interfacing with serial port hardware.
 * \headerfile SerialPort.h <igvc/SerialPort.h>
 */
class SerialPort
{
public:
    /*! \brief The constructor takes in the path to the port (eg. "/dev/ttyUSB0") and a baud rate for the connection and opens the connection. */
    SerialPort(std::string device, unsigned int baud);
    
    ~SerialPort();
    
    /*! \brief Returns true if the serial port is connected and open */
    bool isOpen();
    
    /*! \brief Writes the given string to the serial port. */
    void write(std::string msg);

    /*! \brief Writes the given array of chars to the serial port. */
    void write(char *buffer, size_t length);

    /*! \brief Writes the given array of unsigned chars to the serial port. */
    void write(unsigned char *buffer, size_t length);
    
    /*! \brief Reads a single byte from the serial port.
     * \return The byte read.
     */
    char read();

    /*! \brief Reads numBytes bytes from the serial port.
     * \return An array containing the read bytes.
     */
    char* read(int numBytes);
    
    /*! \brief Reads bytes from the serial port until \n or \r is found.
     * \return String containing the bytes read excluding the newline.
     */
    std::string readln();

    /*! \brief Returns the path to the device this port is connected to.
     * \return String containing path to device.
     */
    std::string devicePath();
    
private:
    boost::asio::io_service ioservice;
    boost::asio::serial_port port;
    std::string path;

};
