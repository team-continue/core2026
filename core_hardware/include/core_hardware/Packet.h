#pragma once

#include <boost/asio.hpp>

#define BUFFER_SIZE 32768

class Packet {
    boost::asio::io_service io;
    boost::asio::serial_port serial_;
    uint8_t encodeBuffer_[BUFFER_SIZE];

   public:
    Packet() : io(), serial_(io) {
    }
    ~Packet() {
        serial_.close();
    }

    // Boost.Asioでシリアル通信してみる
    // https://myon.info/blog/2015/04/19/boost-asio-serial/
    void connect(const char* port) {
        serial_.close();
        serial_.open(port);
        // 速度は 9600bps
        serial_.set_option(boost::asio::serial_port_base::baud_rate(9600));
    }

    void send(const uint8_t* buffer, size_t size) {
        int numEncoded = cobsEncode(buffer, size, encodeBuffer_);
        // 一番最後にパケットマーカーを追加
        encodeBuffer_[numEncoded] = 0;
        // 送信
        boost::system::error_code ec;
        boost::asio::write(serial_, boost::asio::buffer(encodeBuffer_, numEncoded + 1));
    }

    int read(uint8_t* decodeBuffer_) {
        // serial から response に 0 まで読み込む
        boost::asio::streambuf response;
        boost::asio::read_until(serial_, response, 0);

        const uint8_t* _receiveBuffer = boost::asio::buffer_cast<const uint8_t*>(response.data());
        size_t _receiveBufferIndex = response.size() - 1;

        size_t numDecoded = cobsDecode(_receiveBuffer, _receiveBufferIndex, decodeBuffer_);
        return numDecoded;
    }
  private:
    // copy from https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
    size_t cobsEncode(const uint8_t *data, size_t length, uint8_t *buffer) {
        // assert(data && buffer);

        uint8_t *encode = buffer;   // Encoded byte pointer
        uint8_t *codep = encode++;  // Output code pointer
        uint8_t code = 1;           // Code value

        for (const uint8_t *byte = data; length--; ++byte) {
            if (*byte)  // Byte not zero, write it
                *encode++ = *byte, ++code;

            if (!*byte || code == 0xff)  // Input is zero or block completed, restart
            {
                *codep = code, code = 1, codep = encode;
                if (!*byte || length)
                    ++encode;
            }
        }
        *codep = code;  // Write final code value

        return (size_t)(encode - buffer);
    }

    /** COBS decode data from buffer
            @param buffer Pointer to encoded input bytes
            @param length Number of bytes to decode
            @param data Pointer to decoded output data
            @return Number of bytes successfully decoded
            @note Stops decoding if delimiter byte is found
    */
    size_t cobsDecode(const uint8_t *buffer, size_t length, uint8_t *data) {
        // assert(buffer && data);

        const uint8_t *byte = buffer;       // Encoded input byte pointer
        uint8_t *decode = data;  // Decoded output byte pointer

        for (uint8_t code = 0xff, block = 0; byte < buffer + length; --block) {
            if (block)  // Decode block byte
                *decode++ = *byte++;
            else {
                block = *byte++;              // Fetch the next block length
                if (block && (code != 0xff))  // Encoded zero, write it unless it's delimiter.
                    *decode++ = 0;
                code = block;
                if (!code)  // Delimiter code found
                    break;
            }
        }

        return (size_t)(decode - data);
    }
};