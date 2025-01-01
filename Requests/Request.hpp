/* Noah Klein */

#ifndef REQUEST_HPP
#define REQUEST_HPP

#include "../ProtocolDefinitions.hpp"

#include <cstdint>
#include <numeric>
#include <vector>

class Request {
  protected:
    uint8_t UUID;

    // TODO Untested, measure performance as well, latency is very critical
    std::vector<uint8_t> createSerializedMessage(
        uint8_t commandID,
        const std::vector<uint8_t> &serializedPayload) const {

        uint8_t checksum = UUID + commandID + serializedPayload.size() +
                           std::accumulate(serializedPayload.begin(),
                                           serializedPayload.end(),
                                           0);

        std::vector<uint8_t> request;
        request.reserve(6 + serializedPayload.size());

        request.push_back(START_MARKER);
        request.push_back(UUID);
        request.push_back(commandID);
        request.push_back(static_cast<uint8_t>(serializedPayload.size()));
        request.insert(
            request.end(), serializedPayload.begin(), serializedPayload.end());
        request.push_back(checksum);
        request.push_back(END_MARKER);

        return request;
    }

    std::vector<uint8_t>
    createSerializedMessage(uint8_t commandID,
                            const uint8_t *serializedPayload,
                            uint8_t payloadLength) const {

        uint8_t checksum = UUID + commandID + payloadLength;

        std::vector<uint8_t> request;
        request.reserve(6 + payloadLength);

        request.push_back(START_MARKER);
        request.push_back(UUID);
        request.push_back(commandID);
        request.push_back(payloadLength);
        for (uint8_t i = 0; i < payloadLength; i++) {
            request.push_back(serializedPayload[i]);
            checksum += serializedPayload[i];
        }
        request.push_back(checksum);
        request.push_back(END_MARKER);

        return request;
    }

    std::vector<uint8_t> createSerializedMessage(uint8_t commandID) const {
        uint8_t checksum = UUID + commandID;
        return {START_MARKER, UUID, commandID, 0, checksum, END_MARKER};
    }

  public:
    explicit Request(uint8_t uuid) : UUID(uuid) {}

    uint8_t getUUID() const { return UUID; }

    // Each command overrides this
    virtual uint8_t getCommandID() const = 0;

    // Serialize parameters into a payload
    virtual std::vector<uint8_t> serializeRequest() const = 0;

    // Populate fields from a payload
    virtual void
    deserializeResponsePayload(const std::vector<uint8_t> &data) = 0;

    virtual ~Request() = default;
};

#endif // REQUEST_HPP
