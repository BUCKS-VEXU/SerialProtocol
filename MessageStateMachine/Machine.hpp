/* Noah Klein */

#include <cstddef>
#include <cstdint>

enum State {
    STATE_IDLE,
    STATE_UUID,
    STATE_LENGTH,
    STATE_COMMAND,
    STATE_PAYLOAD,
    STATE_CHECKSUM,
    STATE_COMPLETE
};

class StateMachine {
  private:
    State currentState = STATE_IDLE;

    size_t bufferSize;
    uint8_t *buffer; // Buffer to hold incoming message
    size_t bufferIndex = 0;

    uint8_t UUID = 0;
    uint8_t commandID = 0;
    uint8_t payloadLength = 0;
    uint8_t checksum = 0;

    uint8_t calculatedChecksum = 0;

  public:
    StateMachine(size_t bufferSize) : bufferSize(bufferSize) {
        buffer = new uint8_t[bufferSize];
    }
    ~StateMachine() { delete[] buffer; }

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

    void loop(uint8_t byte);
};
