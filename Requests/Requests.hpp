/* Noah Klein */

#ifndef REQUESTS_HPP
#define REQUESTS_HPP

#include "Request.hpp"
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "../ProtocolDefinitions.hpp"

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

    void deserializeResponse(const std::vector<uint8_t> &data) override {
        if (data.size() < sizeof(uint8_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize the payload
        bytesPrinted = data[0];
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

    void deserializeResponse(const std::vector<uint8_t> &data) override {
        if (data.size() < sizeof(sfeTkError_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize the connected field
        memcpy(&errorResponse, &data[0], sizeof(sfeTkError_t));
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

    void deserializeResponse(const std::vector<uint8_t> &data) override {
        if (data.size() < sizeof(sfeTkError_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize the error field
        memcpy(&errorResponse, &data[0], sizeof(sfeTkError_t));
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

    void deserializeResponse(const std::vector<uint8_t> &data) override {
        if (data.size() < sizeof(sfeTkError_t) + sizeof(uint8_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize the error and progress fields
        memcpy(&errorResponse, &data[0], sizeof(sfeTkError_t));
        progress = data[4];
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

    void deserializeResponse(const std::vector<uint8_t> &data) override {
        if (data.size() < sizeof(sfeTkError_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize the error field
        memcpy(&errorResponse, &data[0], sizeof(sfeTkError_t));
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

    void deserializeResponse(const std::vector<uint8_t> &data) override {
        if (data.size() < sizeof(sfeTkError_t) + sizeof(sfe_otos_status_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize the error and status fields
        memcpy(&errorResponse, &data[0], sizeof(sfeTkError_t));
        memcpy(&status, &data[4], sizeof(sfe_otos_status_t));
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

    void deserializeResponse(const std::vector<uint8_t> &data) override {
        if (data.size() < sizeof(sfeTkError_t) + sizeof(pose2d_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize the error and offset fields
        memcpy(&errorResponse, &data[0], sizeof(sfeTkError_t));
        memcpy(&offset, &data[4], sizeof(pose2d_t));
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

    void deserializeResponse(const std::vector<uint8_t> &data) override {
        if (data.size() < sizeof(sfeTkError_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize the error field
        memcpy(&errorResponse, &data[0], sizeof(sfeTkError_t));
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

    void deserializeResponse(const std::vector<uint8_t> &data) override {
        if (data.size() < sizeof(sfeTkError_t) + sizeof(pose2d_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize error and pose from the payload
        std::memcpy(&errorResponse, &data[0], sizeof(sfeTkError_t));
        std::memcpy(&pose, &data[4], sizeof(pose2d_t));
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

    void deserializeResponse(const std::vector<uint8_t> &data) override {
        if (data.size() < sizeof(sfeTkError_t) + sizeof(pose2d_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize error and poseStd from the payload
        std::memcpy(&errorResponse, &data[0], sizeof(sfeTkError_t));
        std::memcpy(&poseStd, &data[4], sizeof(pose2d_t));
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

    void deserializeResponse(const std::vector<uint8_t> &data) override {
        if (data.size() < sizeof(sfeTkError_t) + sizeof(pose2d_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize error and velocity from the payload
        std::memcpy(&errorResponse, &data[0], sizeof(sfeTkError_t));
        std::memcpy(&velocity, &data[4], sizeof(pose2d_t));
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

    void deserializeResponse(const std::vector<uint8_t> &data) override {
        if (data.size() < sizeof(sfeTkError_t) + sizeof(pose2d_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize error and velocityStd from the payload
        std::memcpy(&errorResponse, &data[0], sizeof(sfeTkError_t));
        std::memcpy(&velocityStd, &data[4], sizeof(pose2d_t));
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

    void deserializeResponse(const std::vector<uint8_t> &data) override {
        if (data.size() < sizeof(sfeTkError_t) + sizeof(pose2d_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize error and acceleration from the payload
        std::memcpy(&errorResponse, &data[0], sizeof(sfeTkError_t));
        std::memcpy(&acceleration, &data[4], sizeof(pose2d_t));
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

    void deserializeResponse(const std::vector<uint8_t> &data) override {
        if (data.size() < sizeof(sfeTkError_t) + sizeof(pose2d_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize error and accelerationStd from the payload
        std::memcpy(&errorResponse, &data[0], sizeof(sfeTkError_t));
        std::memcpy(&accelerationStd, &data[4], sizeof(pose2d_t));
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

    void deserializeResponse(const std::vector<uint8_t> &data) override {
        if (data.size() < sizeof(sfeTkError_t) + 3 * sizeof(pose2d_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize error, pose, velocity, and acceleration from the payload
        std::memcpy(&errorResponse, &data[0], sizeof(sfeTkError_t));
        std::memcpy(&pose, &data[4], sizeof(pose2d_t));
        std::memcpy(&velocity, &data[16], sizeof(pose2d_t));
        std::memcpy(&acceleration, &data[28], sizeof(pose2d_t));
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

    void deserializeResponse(const std::vector<uint8_t> &data) override {
        if (data.size() < sizeof(sfeTkError_t) + 3 * sizeof(pose2d_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize error, poseStd, velocityStd, and accelerationStd from the
        // payload
        std::memcpy(&errorResponse, &data[0], sizeof(sfeTkError_t));
        std::memcpy(&poseStd, &data[4], sizeof(pose2d_t));
        std::memcpy(&velocityStd, &data[16], sizeof(pose2d_t));
        std::memcpy(&accelerationStd, &data[28], sizeof(pose2d_t));
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

    void deserializeResponse(const std::vector<uint8_t> &data) override {
        if (data.size() < sizeof(sfeTkError_t) + 6 * sizeof(pose2d_t))
            throw std::runtime_error("Invalid payload length");

        // Deserialize error, pose, velocity, acceleration, poseStd,
        // velocityStd, and accelerationStd from the payload
        std::memcpy(&errorResponse, &data[0], sizeof(sfeTkError_t));
        std::memcpy(&pose, &data[4], sizeof(pose2d_t));
        std::memcpy(&velocity, &data[16], sizeof(pose2d_t));
        std::memcpy(&acceleration, &data[28], sizeof(pose2d_t));
        std::memcpy(&poseStd, &data[40], sizeof(pose2d_t));
        std::memcpy(&velocityStd, &data[52], sizeof(pose2d_t));
        std::memcpy(&accelerationStd, &data[64], sizeof(pose2d_t));
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
