/* Noah Klein */

#include <cstddef>
#include <cstdint>

#include "ProtocolDefinitions.hpp"

enum State {
    STATE_IDLE,
    STATE_UUID,
    STATE_LENGTH,
    STATE_COMMAND,
    STATE_PAYLOAD,
    STATE_CHECKSUM,
    STATE_COMPLETE
};

class MessageStateMachine {
  protected:
    State currentState = STATE_IDLE;

    size_t bufferSize;
    uint8_t *payloadBuffer; // Buffer to hold incoming message
    size_t bufferIndex = 0;

    uint8_t UUID = 0;
    uint8_t commandID = 0;
    uint8_t payloadLength = 0;

    uint8_t checksum = 0;
    uint8_t calculatedChecksum = 0;

    // ! These should be implemented by the extending state machine
    virtual void onChecksumFail() const = 0;
    virtual void onMissingStartMarker(uint8_t byte) const = 0;
    virtual void onMissingEndMarker(uint8_t byte) const = 0;
    virtual void onMessageComplete() const = 0;

  public:
    MessageStateMachine(size_t bufferSize) : bufferSize(bufferSize) {
        payloadBuffer = new uint8_t[bufferSize];
    }
    ~MessageStateMachine() { delete[] payloadBuffer; }

    inline State getCurrentState() { return currentState; }

    inline size_t getBufferSize() { return bufferSize; }

    void reset() {
        currentState = STATE_IDLE;
        UUID = 0;
        bufferIndex = 0;
        payloadLength = 0;
        checksum = 0;
        calculatedChecksum = 0;
    }

    void loop(uint8_t byte) {
        switch (currentState) {
        case STATE_IDLE:
            if (byte == START_MARKER) {
                currentState = STATE_UUID;
            } else {
                onMissingStartMarker(byte);
            }
            break;

        case STATE_UUID:
            UUID = byte;
            calculatedChecksum = byte;
            currentState = STATE_COMMAND;
            break;

        case STATE_COMMAND:
            commandID = byte; // Store command ID
            calculatedChecksum += byte;
            currentState = STATE_LENGTH;
            break;

        case STATE_LENGTH:
            payloadLength = byte;
            calculatedChecksum += byte;
            currentState = (payloadLength > 0) ? STATE_PAYLOAD : STATE_CHECKSUM;
            break;

        case STATE_PAYLOAD:
            payloadBuffer[bufferIndex++] = byte;
            calculatedChecksum += byte;

            if (bufferIndex == payloadLength) {
                currentState = STATE_CHECKSUM;
            }
            break;

        case STATE_CHECKSUM:
            checksum = byte;

            if (checksum == calculatedChecksum) {
                currentState = STATE_COMPLETE;
            } else {
                onChecksumFail();
                this->reset();
                return;
            }
            break;

        case STATE_COMPLETE:
            if (byte != END_MARKER) {
                onMissingEndMarker(byte);
                this->reset();
                return;
            }
            onMessageComplete();
            this->reset();
            break;
        }
    }
};
