#pragma once

#include <Arduino.h>

class WirelessModule {
public:
    static constexpr uint32_t BAUD_RATE = 115200;
    static constexpr size_t DATA_LENGTH = 7;     // ':' の次を無視し、その後ろの 7 byte を使う

    explicit WirelessModule()
        : linePos_(0) {
        clearLineBuffer();
    }

    void init() {
        PORT_WIRELESS.begin(BAUD_RATE);
        PORT_WIRELESS.addMemoryForRead(wirelessbuffer_, sizeof(wirelessbuffer_));
        clearLineBuffer();
    }

    // 1行ぶん受信して正しくパースできたら true
    // data[0..6] に格納する
    bool update(uint8_t* data) {
        while (PORT_WIRELESS.available() > 0) {
            char c = static_cast<char>(PORT_WIRELESS.read());

            // CR は無視
            if (c == '\r') {
                continue;
            }

            // LF で1行完成
            if (c == '\n') {
                lineBuffer_[linePos_] = '\0';

                bool ok = parseLine(lineBuffer_, data);

                clearLineBuffer();
                return ok;
            }

            // バッファに追記
            if (linePos_ < LINE_BUFFER_SIZE - 1) {
                lineBuffer_[linePos_++] = c;
            } else {
                // オーバーフローしたら捨てる
                clearLineBuffer();
            }
        }

        return false;
    }

private:
    static constexpr size_t LINE_BUFFER_SIZE = 128;
    char lineBuffer_[LINE_BUFFER_SIZE];
    size_t linePos_;
    uint8_t wirelessbuffer_[1024];

    void clearLineBuffer() {
        linePos_ = 0;
        lineBuffer_[0] = '\0';
    }

    static bool parseHexByte(const char* token, uint8_t& out) {
        if (token == nullptr) {
            return false;
        }

        char* endPtr = nullptr;
        long value = strtol(token, &endPtr, 16);

        if (*token == '\0' || *endPtr != '\0' || value < 0x00 || value > 0xFF) {
            return false;
        }

        out = static_cast<uint8_t>(value);
        return true;
    }

    // 例:
    // 00,0401,D0:01,91,83,91,89,00,00,00
    //            ^ この 01 は無視し、その後ろの 7 byte を data に入れる
    static bool parseLine(char* line, uint8_t* data) {
        if (line == nullptr || data == nullptr) {
            return false;
        }

        char* colon = strchr(line, ':');
        if (colon == nullptr) {
            return false;
        }

        // ':' の後ろを読む
        char* payload = colon + 1;
        char* token = strtok(payload, ",");
        if (token == nullptr) {
            return false;
        }

        // ':' の次にある先頭 1 要素は使わない
        token = strtok(nullptr, ",");

        size_t index = 0;
        while (token != nullptr && index < DATA_LENGTH) {
            if (!parseHexByte(token, data[index])) {
                return false;
            }

            ++index;
            token = strtok(nullptr, ",");
        }

        // 7 byte ちょうど取れたか確認
        if (index != DATA_LENGTH) {
            return false;
        }

        return true;
    }
};
