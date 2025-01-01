/* Noah Klein */

#ifndef REQUESTS_HPP
#define REQUESTS_HPP

#include "Request.hpp"
#include <cstdint>
#include <stdexcept>
#include <vector>
#include <string.h>

#include "../ProtocolDefinitions.hpp"

#ifdef OTOS_DEFINITIONS_PRESENT
#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#endif // OTOS_DEFINITIONS_PRESENT

class PingRequest : public Request {
  private:
    // Request-specific parameters (input or response fields)
    uint8_t pingByte;

    // Response fields
    uint8_t pingResponse;

  public:
    PingRequest(uint8_t uuid, uint8_t pingByte) :
        Request(uuid), pingByte(pingByte), pingResponse(0) {}

    uint8_t getCommandID() const override { return PING; }

    std::vector<uint8_t> serializeRequest() const override {
        return createSerializedMessage(PING, &pingByte, 1);
    }

    void
    deserializeResponsePayload(const std::vector<uint8_t> &payload) override {
        if (payload.size() < sizeof(uint8_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize the ping response
        pingResponse = payload[0];
    }

    uint8_t getPingResponse() const { return pingResponse; }
};

class SerialPrintPayloadRequest : public Request {
  private:
    // Request-specific parameters (input or response fields)
    std::vector<uint8_t> payload;

    // Response fields
    uint8_t bytesPrinted;

  public:
    SerialPrintPayloadRequest(uint8_t uuid,
                              const std::vector<uint8_t> &payload) :
        Request(uuid), payload(payload) {}

    uint8_t getCommandID() const override { return SERIAL_PRINT_PAYLOAD; }

    std::vector<uint8_t> serializeRequest() const override {
        return createSerializedMessage(SERIAL_PRINT_PAYLOAD, payload);
    }

    void
    deserializeResponsePayload(const std::vector<uint8_t> &payload) override {
        if (payload.size() < sizeof(uint8_t))
            throw std::runtime_error("Invalid payload length");

        // "Deserialize" the payload
        bytesPrinted = payload[0];
    }

    uint8_t getBytesPrinted() const { return bytesPrinted; }
};

class IsConnectedRequest : public Request {
  private:
    // Request-specific parameters (input or response fields)

    // Response
    sfeTkError_t errorResponse;

  public:
    IsConnectedRequest(uint8_t uuid) : Request(uuid), errorResponse(0) {}

    uint8_t getCommandID() const override { return IS_CONNECTED; }

    std::vector<uint8_t> serializeRequest() const override {
        return createSerializedMessage(IS_CONNECTED);
    }

    void
    deserializeResponsePayload(const std::vector<uint8_t> &payload) override {
        if (payload.size() < sizeof(sfeTkError_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize the error field
        memcpy(&errorResponse, &payload[0], sizeof(sfeTkError_t));
    }

    sfeTkError_t getResponse() const { return errorResponse; }
};

class CalibrateIMURequest : public Request {
  private:
    // Request-specific parameters (input or response fields)
    uint8_t numSamples;
    bool waitUntilDone;

    // Response
    sfeTkError_t errorResponse;

  public:
    CalibrateIMURequest(uint8_t uuid, uint8_t numSamples, bool waitUntilDone) :
        Request(uuid), errorResponse(0) {}

    uint8_t getCommandID() const override { return CALIBRATE_IMU; }

    std::vector<uint8_t> serializeRequest() const override {
        std::vector<uint8_t> payload = {numSamples, waitUntilDone};
        return createSerializedMessage(CALIBRATE_IMU, payload);
    }

    void
    deserializeResponsePayload(const std::vector<uint8_t> &payload) override {
        if (payload.size() < sizeof(sfeTkError_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize the error field
        memcpy(&errorResponse, &payload[0], sizeof(sfeTkError_t));
    }

    sfeTkError_t getError() const { return errorResponse; }
};

class GetIMUCalibrationProgressRequest : public Request {
  private:
    // Request-specific parameters (input or response fields)

    // Response
    sfeTkError_t errorResponse;
    uint8_t progress;

  public:
    GetIMUCalibrationProgressRequest(uint8_t uuid) :
        Request(uuid), errorResponse(0), progress(0) {}

    uint8_t getCommandID() const override {
        return GET_IMU_CALIBRATION_PROGRESS;
    }

    std::vector<uint8_t> serializeRequest() const override {
        return createSerializedMessage(GET_IMU_CALIBRATION_PROGRESS);
    }

    void
    deserializeResponsePayload(const std::vector<uint8_t> &payload) override {
        if (payload.size() < sizeof(sfeTkError_t) + sizeof(uint8_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize the error and progress fields
        memcpy(&errorResponse, &payload[0], sizeof(sfeTkError_t));
        progress = payload[4];
    }

    sfeTkError_t getError() const { return errorResponse; }
    uint8_t getProgress() const { return progress; }
};

class ResetTrackingRequest : public Request {
  private:
    // Request-specific parameters (input or response fields)

    // Response
    sfeTkError_t errorResponse;

  public:
    ResetTrackingRequest(uint8_t uuid) : Request(uuid), errorResponse(0) {}

    uint8_t getCommandID() const override { return RESET_TRACKING; }

    std::vector<uint8_t> serializeRequest() const override {
        return createSerializedMessage(RESET_TRACKING);
    }

    void
    deserializeResponsePayload(const std::vector<uint8_t> &payload) override {
        if (payload.size() < sizeof(sfeTkError_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize the error field
        memcpy(&errorResponse, &payload[0], sizeof(sfeTkError_t));
    }

    sfeTkError_t getError() const { return errorResponse; }
};

class GetStatusRequest : public Request {
  private:
    // Request-specific parameters (input or response fields)

    // Response
    sfeTkError_t errorResponse;
    sfe_otos_status_t status;

  public:
    GetStatusRequest(uint8_t uuid) :
        Request(uuid), errorResponse(0), status({0}) {}

    uint8_t getCommandID() const override { return GET_STATUS; }

    std::vector<uint8_t> serializeRequest() const override {
        return createSerializedMessage(GET_STATUS);
    }

    void
    deserializeResponsePayload(const std::vector<uint8_t> &payload) override {
        if (payload.size() < sizeof(sfeTkError_t) + sizeof(sfe_otos_status_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize the error and status fields
        memcpy(&errorResponse, &payload[0], sizeof(sfeTkError_t));
        memcpy(&status, &payload[4], sizeof(sfe_otos_status_t));
    }

    sfeTkError_t getError() const { return errorResponse; }
    sfe_otos_status_t getStatus() const { return status; }
};

class GetOffsetRequest : public Request {
  private:
    // Request-specific parameters (input or response fields)

    // Response
    sfeTkError_t errorResponse;
    pose2d_t offset;

  public:
    GetOffsetRequest(uint8_t uuid) :
        Request(uuid), errorResponse(0), offset({0, 0, 0}) {}

    uint8_t getCommandID() const override { return GET_OFFSET; }

    std::vector<uint8_t> serializeRequest() const override {
        return createSerializedMessage(GET_OFFSET);
    }

    void
    deserializeResponsePayload(const std::vector<uint8_t> &payload) override {
        if (payload.size() < sizeof(sfeTkError_t) + sizeof(pose2d_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize the error and offset fields
        memcpy(&errorResponse, &payload[0], sizeof(sfeTkError_t));
        memcpy(&offset, &payload[4], sizeof(pose2d_t));
    }

    sfeTkError_t getError() const { return errorResponse; }
    pose2d_t getOffset() const { return offset; }
};

class SetOffsetRequest : public Request {
  private:
    // Request-specific parameters (input or response fields)
    pose2d_t offset;

    // Response
    sfeTkError_t errorResponse;

  public:
    SetOffsetRequest(uint8_t uuid, pose2d_t offset) :
        Request(uuid), offset(offset), errorResponse(0) {}

    uint8_t getCommandID() const override { return SET_OFFSET; }

    std::vector<uint8_t> serializeRequest() const override {
        return createSerializedMessage(
            SET_OFFSET,
            reinterpret_cast<const uint8_t *>(&offset),
            sizeof(pose2d_t));
    }

    void
    deserializeResponsePayload(const std::vector<uint8_t> &payload) override {
        if (payload.size() < sizeof(sfeTkError_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize the error field
        memcpy(&errorResponse, &payload[0], sizeof(sfeTkError_t));
    }

    sfeTkError_t getError() const { return errorResponse; }
};

class GetPoseRequest : public Request {
  private:
    // Request-specific parameters (input or response fields)

    // Response fields
    sfeTkError_t errorResponse;
    pose2d_t pose;

  public:
    GetPoseRequest(uint8_t uuid) :
        Request(uuid), errorResponse(0), pose({0, 0, 0}) {}

    uint8_t getCommandID() const override { return GET_POSE; }

    std::vector<uint8_t> serializeRequest() const override {
        return createSerializedMessage(GET_POSE);
    }

    void
    deserializeResponsePayload(const std::vector<uint8_t> &payload) override {
        if (payload.size() < sizeof(sfeTkError_t) + sizeof(pose2d_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize error and pose from the payload
        memcpy(&errorResponse, &payload[0], sizeof(sfeTkError_t));
        memcpy(&pose, &payload[4], sizeof(pose2d_t));
    }

    sfeTkError_t getError() const { return errorResponse; }
    pose2d_t getPose() const { return pose; }
};

class GetPoseStdRequest : public Request {
  private:
    // Request-specific parameters (input or response fields)

    // Response fields
    sfeTkError_t errorResponse;
    pose2d_t poseStd;

  public:
    GetPoseStdRequest(uint8_t uuid) :
        Request(uuid), errorResponse(0), poseStd({0, 0, 0}) {}

    uint8_t getCommandID() const override { return GET_POSE_STD; }

    std::vector<uint8_t> serializeRequest() const override {
        return createSerializedMessage(GET_POSE_STD);
    }

    void
    deserializeResponsePayload(const std::vector<uint8_t> &payload) override {
        if (payload.size() < sizeof(sfeTkError_t) + sizeof(pose2d_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize error and poseStd from the payload
        memcpy(&errorResponse, &payload[0], sizeof(sfeTkError_t));
        memcpy(&poseStd, &payload[4], sizeof(pose2d_t));
    }

    sfeTkError_t getError() const { return errorResponse; }
    pose2d_t getPoseStd() const { return poseStd; }
};

class GetVelocityRequest : public Request {
  private:
    // Request-specific parameters (input or response fields)

    // Response fields
    sfeTkError_t errorResponse;
    pose2d_t velocity;

  public:
    GetVelocityRequest(uint8_t uuid) :
        Request(uuid), errorResponse(0), velocity({0, 0, 0}) {}

    uint8_t getCommandID() const override { return GET_VELOCITY; }

    std::vector<uint8_t> serializeRequest() const override {
        return createSerializedMessage(GET_VELOCITY);
    }

    void
    deserializeResponsePayload(const std::vector<uint8_t> &payload) override {
        if (payload.size() < sizeof(sfeTkError_t) + sizeof(pose2d_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize error and velocity from the payload
        memcpy(&errorResponse, &payload[0], sizeof(sfeTkError_t));
        memcpy(&velocity, &payload[4], sizeof(pose2d_t));
    }

    sfeTkError_t getError() const { return errorResponse; }
    pose2d_t getVelocity() const { return velocity; }
};

class GetVelocityStdRequest : public Request {
  private:
    // Request-specific parameters (input or response fields)

    // Response fields
    sfeTkError_t errorResponse;
    pose2d_t velocityStd;

  public:
    GetVelocityStdRequest(uint8_t uuid) :
        Request(uuid), errorResponse(0), velocityStd({0, 0, 0}) {}

    uint8_t getCommandID() const override { return GET_VELOCITY_STD; }

    std::vector<uint8_t> serializeRequest() const override {
        return createSerializedMessage(GET_VELOCITY_STD);
    }

    void
    deserializeResponsePayload(const std::vector<uint8_t> &payload) override {
        if (payload.size() < sizeof(sfeTkError_t) + sizeof(pose2d_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize error and velocityStd from the payload
        memcpy(&errorResponse, &payload[0], sizeof(sfeTkError_t));
        memcpy(&velocityStd, &payload[4], sizeof(pose2d_t));
    }

    sfeTkError_t getError() const { return errorResponse; }
    pose2d_t getVelocityStd() const { return velocityStd; }
};

class GetAccelerationRequest : public Request {
  private:
    // Request-specific parameters (input or response fields)

    // Response fields
    sfeTkError_t errorResponse;
    pose2d_t acceleration;

  public:
    GetAccelerationRequest(uint8_t uuid) :
        Request(uuid), errorResponse(0), acceleration({0, 0, 0}) {}

    uint8_t getCommandID() const override { return GET_ACCELERATION; }

    std::vector<uint8_t> serializeRequest() const override {
        return createSerializedMessage(GET_ACCELERATION);
    }

    void
    deserializeResponsePayload(const std::vector<uint8_t> &payload) override {
        if (payload.size() < sizeof(sfeTkError_t) + sizeof(pose2d_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize error and acceleration from the payload
        memcpy(&errorResponse, &payload[0], sizeof(sfeTkError_t));
        memcpy(&acceleration, &payload[4], sizeof(pose2d_t));
    }

    sfeTkError_t getError() const { return errorResponse; }
    pose2d_t getAcceleration() const { return acceleration; }
};

class GetAccelerationStdRequest : public Request {
  private:
    // Request-specific parameters (input or response fields)

    // Response fields
    sfeTkError_t errorResponse;
    pose2d_t accelerationStd;

  public:
    GetAccelerationStdRequest(uint8_t uuid) :
        Request(uuid), errorResponse(0), accelerationStd({0, 0, 0}) {}

    uint8_t getCommandID() const override { return GET_ACCELERATION_STD; }

    std::vector<uint8_t> serializeRequest() const override {
        return createSerializedMessage(GET_ACCELERATION_STD);
    }

    void
    deserializeResponsePayload(const std::vector<uint8_t> &payload) override {
        if (payload.size() < sizeof(sfeTkError_t) + sizeof(pose2d_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize error and accelerationStd from the payload
        memcpy(&errorResponse, &payload[0], sizeof(sfeTkError_t));
        memcpy(&accelerationStd, &payload[4], sizeof(pose2d_t));
    }

    sfeTkError_t getError() const { return errorResponse; }
    pose2d_t getAccelerationStd() const { return accelerationStd; }
};

class GetPosVelAccRequest : public Request {
  private:
    // Request-specific parameters (input or response fields)

    // Response fields
    sfeTkError_t errorResponse;
    pose2d_t pose;
    pose2d_t velocity;
    pose2d_t acceleration;

  public:
    GetPosVelAccRequest(uint8_t uuid) :
        Request(uuid),
        errorResponse(0),
        pose({0, 0, 0}),
        velocity({0, 0, 0}),
        acceleration({0, 0, 0}) {}

    uint8_t getCommandID() const override { return GET_POS_VEL_ACC; }

    std::vector<uint8_t> serializeRequest() const override {
        return createSerializedMessage(GET_POS_VEL_ACC);
    }

    void
    deserializeResponsePayload(const std::vector<uint8_t> &payload) override {
        if (payload.size() < sizeof(sfeTkError_t) + 3 * sizeof(pose2d_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize error, pose, velocity, and acceleration from the payload
        memcpy(&errorResponse, &payload[0], sizeof(sfeTkError_t));
        memcpy(&pose, &payload[4], sizeof(pose2d_t));
        memcpy(&velocity, &payload[16], sizeof(pose2d_t));
        memcpy(&acceleration, &payload[28], sizeof(pose2d_t));
    }

    sfeTkError_t getError() const { return errorResponse; }
    pose2d_t getPose() const { return pose; }
    pose2d_t getVelocity() const { return velocity; }
    pose2d_t getAcceleration() const { return acceleration; }
};

class GetPosVelAccStdRequest : public Request {
  private:
    // Request-specific parameters (input or response fields)

    // Response fields
    sfeTkError_t errorResponse;
    pose2d_t poseStd;
    pose2d_t velocityStd;
    pose2d_t accelerationStd;

  public:
    GetPosVelAccStdRequest(uint8_t uuid) :
        Request(uuid),
        errorResponse(0),
        poseStd({0, 0, 0}),
        velocityStd({0, 0, 0}),
        accelerationStd({0, 0, 0}) {}

    uint8_t getCommandID() const override { return GET_POS_VEL_ACC_STD; }

    std::vector<uint8_t> serializeRequest() const override {
        return createSerializedMessage(GET_POS_VEL_ACC_STD);
    }

    void
    deserializeResponsePayload(const std::vector<uint8_t> &payload) override {
        if (payload.size() < sizeof(sfeTkError_t) + 3 * sizeof(pose2d_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize error, poseStd, velocityStd, and accelerationStd from the
        // payload
        memcpy(&errorResponse, &payload[0], sizeof(sfeTkError_t));
        memcpy(&poseStd, &payload[4], sizeof(pose2d_t));
        memcpy(&velocityStd, &payload[16], sizeof(pose2d_t));
        memcpy(&accelerationStd, &payload[28], sizeof(pose2d_t));
    }

    sfeTkError_t getError() const { return errorResponse; }
    pose2d_t getPoseStd() const { return poseStd; }
    pose2d_t getVelocityStd() const { return velocityStd; }
    pose2d_t getAccelerationStd() const { return accelerationStd; }
};

class GetPosVelAccAndStdRequest : public Request {
  private:
    // Request-specific parameters (input or response fields)

    // Response fields
    sfeTkError_t errorResponse;
    pose2d_t pose;
    pose2d_t velocity;
    pose2d_t acceleration;
    pose2d_t poseStd;
    pose2d_t velocityStd;
    pose2d_t accelerationStd;

  public:
    GetPosVelAccAndStdRequest(uint8_t uuid) :
        Request(uuid),
        errorResponse(0),
        pose({0, 0, 0}),
        velocity({0, 0, 0}),
        acceleration({0, 0, 0}),
        poseStd({0, 0, 0}),
        velocityStd({0, 0, 0}),
        accelerationStd({0, 0, 0}) {}

    uint8_t getCommandID() const override { return GET_POS_VEL_ACC_AND_STD; }

    std::vector<uint8_t> serializeRequest() const override {
        return createSerializedMessage(GET_POS_VEL_ACC_AND_STD);
    }

    void
    deserializeResponsePayload(const std::vector<uint8_t> &payload) override {
        if (payload.size() < sizeof(sfeTkError_t) + 6 * sizeof(pose2d_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize error, pose, velocity, acceleration, poseStd,
        // velocityStd, and accelerationStd from the payload
        memcpy(&errorResponse, &payload[0], sizeof(sfeTkError_t));
        memcpy(&pose, &payload[4], sizeof(pose2d_t));
        memcpy(&velocity, &payload[16], sizeof(pose2d_t));
        memcpy(&acceleration, &payload[28], sizeof(pose2d_t));
        memcpy(&poseStd, &payload[40], sizeof(pose2d_t));
        memcpy(&velocityStd, &payload[52], sizeof(pose2d_t));
        memcpy(&accelerationStd, &payload[64], sizeof(pose2d_t));
    }

    sfeTkError_t getError() const { return errorResponse; }
    pose2d_t getPose() const { return pose; }
    pose2d_t getVelocity() const { return velocity; }
    pose2d_t getAcceleration() const { return acceleration; }
    pose2d_t getPoseStd() const { return poseStd; }
    pose2d_t getVelocityStd() const { return velocityStd; }
    pose2d_t getAccelerationStd() const { return accelerationStd; }
};

#endif // REQUESTS_HPP
