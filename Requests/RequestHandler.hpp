/* Noah Klein */

#ifndef REQUEST_HANDLER_HPP
#define REQUEST_HANDLER_HPP

#ifdef OTOS_DEFINITIONS_PRESENT

// Abstract Base Class for RPC Commands
class RPCCommand {
  public:
    virtual ~RPCCommand() = default;

    virtual int
    execute(uint8_t UUID, const uint8_t *payload, uint8_t payloadLength) = 0;

    uint8_t getCommandID() const { return commandID; }

  protected:
    RPCCommand(uint8_t id, int argLen, int retLen) :
        commandID(id), argLength(argLen), returnLength(retLen) {}

    bool validatePayload(uint8_t payloadLength) const {
        return payloadLength == argLength;
    }

    // 12U
    const size_t pose2dSize = sizeof(sfe_otos_pose2d_t);

    uint8_t commandID;
    int argLength;
    int returnLength;
};

/******************************* OTOS Commands *******************************/

class IsConnectedCommand : public RPCCommand {
  public:
    IsConnectedCommand() : RPCCommand(IS_CONNECTED, 0, 4) {}

    int execute(uint8_t UUID,
                const uint8_t *payload,
                uint8_t payloadLength) override {
        if (!validatePayload(payloadLength)) {
            Serial.printf("Incorrect argument length (%d) for `IsConnected`\n",
                          payloadLength);
            return -1;
        }

        sfeTkError_t error = myOtos.isConnected();

        uint8_t *response = reinterpret_cast<uint8_t *>(&error);
        sendResponse(UUID, commandID, response, sizeof(response));

        return 0;
    }
};

class CalibrateIMUCommand : public RPCCommand {
  public:
    CalibrateIMUCommand() : RPCCommand(CALIBRATE_IMU, 2, 4) {}

    int execute(uint8_t UUID,
                const uint8_t *payload,
                uint8_t payloadLength) override {
        if (!validatePayload(payloadLength)) {
            Serial.printf("Incorrect argument length (%d) for `CalibrateIMU`\n",
                          payloadLength);
            return -1;
        }

        uint8_t numSamples = payload[0];
        bool waitUntilDone = payload[1];
        sfeTkError_t error = myOtos.calibrateImu(numSamples, waitUntilDone);

        uint8_t *response = reinterpret_cast<uint8_t *>(&error);
        sendResponse(UUID, commandID, response, sizeof(response));

        return 0;
    }
};

class GetIMUCalibrationProgressCommand : public RPCCommand {
  public:
    GetIMUCalibrationProgressCommand() :
        RPCCommand(GET_IMU_CALIBRATION_PROGRESS, 0, 5) {}

    int execute(uint8_t UUID,
                const uint8_t *payload,
                uint8_t payloadLength) override {
        if (!validatePayload(payloadLength)) {
            Serial.printf("Incorrect argument length (%d) for "
                          "`GetIMUCalibrationProgress`\n",
                          payloadLength);
            return -1;
        }

        uint8_t numSamples;
        sfeTkError_t error = myOtos.getImuCalibrationProgress(numSamples);

        uint8_t response[5];
        memcpy(response, &error, sizeof(error));
        response[4] = numSamples;

        sendResponse(UUID, commandID, response, sizeof(response));

        return 0;
    }
};

class ResetTrackingCommand : public RPCCommand {
  public:
    ResetTrackingCommand() : RPCCommand(RESET_TRACKING, 0, 4) {}

    int execute(uint8_t UUID,
                const uint8_t *payload,
                uint8_t payloadLength) override {
        if (!validatePayload(payloadLength)) {
            Serial.printf("Incorrect argument length (%d) for "
                          "`ResetTracking`\n",
                          payloadLength);
            return -1;
        }

        sfeTkError_t error = myOtos.resetTracking();

        uint8_t *response = reinterpret_cast<uint8_t *>(&error);
        sendResponse(UUID, commandID, response, sizeof(response));

        return 0;
    }
};

class GetStatusCommand : public RPCCommand {
  public:
    GetStatusCommand() : RPCCommand(GET_STATUS, 0, 5) {}

    int execute(uint8_t UUID,
                const uint8_t *payload,
                uint8_t payloadLength) override {
        if (!validatePayload(payloadLength)) {
            Serial.printf("Incorrect argument length (%d) for `GetStatus`\n",
                          payloadLength);
            return -1;
        }

        sfe_otos_status_t status;
        sfeTkError_t error = myOtos.getStatus(status);

        uint8_t response[5];
        memcpy(response, &error, sizeof(error));
        memcpy(response + sizeof(error), &status, sizeof(status));

        sendResponse(UUID, commandID, response, sizeof(response));

        return 0;
    }
};

class GetOffsetCommand : public RPCCommand {
  public:
    GetOffsetCommand() : RPCCommand(GET_OFFSET, 0, 4 + pose2dSize) {}

    int execute(uint8_t UUID,
                const uint8_t *payload,
                uint8_t payloadLength) override {
        if (!validatePayload(payloadLength)) {
            Serial.printf("Incorrect argument length (%d) for `GetOffset`\n",
                          payloadLength);
            return -1;
        }

        sfe_otos_pose2d_t offset;
        sfeTkError_t error = myOtos.getOffset(offset);

        uint8_t response[16];
        memcpy(response, &error, sizeof(error));
        memcpy(response + sizeof(error), &offset, sizeof(offset));

        sendResponse(UUID, commandID, response, sizeof(response));

        return 0;
    }
};

class SetOffsetCommand : public RPCCommand {
  public:
    SetOffsetCommand() : RPCCommand(SET_OFFSET, pose2dSize, 4) {}

    int execute(uint8_t UUID,
                const uint8_t *payload,
                uint8_t payloadLength) override {
        if (!validatePayload(payloadLength)) {
            Serial.printf("Incorrect argument length (%d) for `SetOffset`\n",
                          payloadLength);
            return -1;
        }

        /*
         * 4 bytes (as float) for x
         * 4 bytes (as float) for y
         * 4 bytes (as float) for h
         * */
        const float *data = reinterpret_cast<const float *>(payload);
        float x = data[0];
        float y = data[1];
        float h = data[2];
        sfe_otos_pose2d_t offset = {x, y, h};
        sfeTkError_t error = myOtos.setOffset(offset);

        uint8_t *response = reinterpret_cast<uint8_t *>(&error);
        sendResponse(UUID, commandID, response, sizeof(response));

        return 0;
    }
};

class GetPoseCommand : public RPCCommand {
  public:
    GetPoseCommand() : RPCCommand(GET_POSE, 0, 4 + pose2dSize) {}

    int execute(uint8_t UUID,
                const uint8_t *payload,
                uint8_t payloadLength) override {
        if (!validatePayload(payloadLength)) {
            Serial.printf("Incorrect argument length (%d) for `GetPose`\n",
                          payloadLength);
            return -1;
        }

        sfe_otos_pose2d_t pose;
        sfeTkError_t error = myOtos.getPosition(pose);

        uint8_t response[16];
        memcpy(response, &error, sizeof(error));
        memcpy(response + sizeof(error), &pose, sizeof(pose));

        sendResponse(UUID, commandID, response, sizeof(response));

        return 0;
    }
};

class GetPoseStdCommand : public RPCCommand {
  public:
    GetPoseStdCommand() : RPCCommand(GET_POSE_STD, 0, 4 + pose2dSize) {}

    int execute(uint8_t UUID,
                const uint8_t *payload,
                uint8_t payloadLength) override {
        if (!validatePayload(payloadLength)) {
            Serial.printf("Incorrect argument length (%d) for `GetPoseStd`\n",
                          payloadLength);
            return -1;
        }

        sfe_otos_pose2d_t poseStdDev;
        sfeTkError_t error = myOtos.getPositionStdDev(poseStdDev);

        uint8_t response[16];
        memcpy(response, &error, sizeof(error));
        memcpy(response + sizeof(error), &poseStdDev, sizeof(poseStdDev));

        sendResponse(UUID, commandID, response, sizeof(response));

        return 0;
    }
};

class GetVelocityCommand : public RPCCommand {
  public:
    GetVelocityCommand() : RPCCommand(GET_VELOCITY, 0, 4 + pose2dSize) {}

    int execute(uint8_t UUID,
                const uint8_t *payload,
                uint8_t payloadLength) override {
        if (!validatePayload(payloadLength)) {
            Serial.printf("Incorrect argument length (%d) for `GetVelocity`\n",
                          payloadLength);
            return -1;
        }

        sfe_otos_pose2d_t velocity;
        sfeTkError_t error = myOtos.getVelocity(velocity);

        uint8_t response[16];
        memcpy(response, &error, sizeof(error));
        memcpy(response + sizeof(error), &velocity, sizeof(velocity));

        sendResponse(UUID, commandID, response, sizeof(response));

        return 0;
    }
};

class GetVelocityStdCommand : public RPCCommand {
  public:
    GetVelocityStdCommand() : RPCCommand(GET_VELOCITY_STD, 0, 4 + pose2dSize) {}

    int execute(uint8_t UUID,
                const uint8_t *payload,
                uint8_t payloadLength) override {
        if (!validatePayload(payloadLength)) {
            Serial.printf("Incorrect argument length (%d) for "
                          "`GetVelocityStd`\n",
                          payloadLength);
            return -1;
        }

        sfe_otos_pose2d_t velocityStdDev;
        sfeTkError_t error = myOtos.getVelocityStdDev(velocityStdDev);

        uint8_t response[16];
        memcpy(response, &error, sizeof(error));
        memcpy(
            response + sizeof(error), &velocityStdDev, sizeof(velocityStdDev));

        sendResponse(UUID, commandID, response, sizeof(response));

        return 0;
    }
};

class GetAccelerationCommand : public RPCCommand {
  public:
    GetAccelerationCommand() :
        RPCCommand(GET_ACCELERATION, 0, 4 + pose2dSize) {}

    int execute(uint8_t UUID,
                const uint8_t *payload,
                uint8_t payloadLength) override {
        if (!validatePayload(payloadLength)) {
            Serial.printf("Incorrect argument length (%d) for "
                          "`GetAcceleration`\n",
                          payloadLength);
            return -1;
        }

        sfe_otos_pose2d_t acceleration;
        sfeTkError_t error = myOtos.getAcceleration(acceleration);

        uint8_t response[16];
        memcpy(response, &error, sizeof(error));
        memcpy(response + sizeof(error), &acceleration, sizeof(acceleration));

        sendResponse(UUID, commandID, response, sizeof(response));

        return 0;
    }
};

class GetAccelerationStdCommand : public RPCCommand {
  public:
    GetAccelerationStdCommand() :
        RPCCommand(GET_ACCELERATION_STD, 0, 4 + pose2dSize) {}

    int execute(uint8_t UUID,
                const uint8_t *payload,
                uint8_t payloadLength) override {
        if (!validatePayload(payloadLength)) {
            Serial.printf("Incorrect argument length (%d) for "
                          "`GetAccelerationStd`\n",
                          payloadLength);
            return -1;
        }

        sfe_otos_pose2d_t accelerationStdDev;
        sfeTkError_t error = myOtos.getAccelerationStdDev(accelerationStdDev);

        uint8_t response[16];
        memcpy(response, &error, sizeof(error));
        memcpy(response + sizeof(error),
               &accelerationStdDev,
               sizeof(accelerationStdDev));

        sendResponse(UUID, commandID, response, sizeof(response));

        return 0;
    }
};

class GetPosVelAccCommand : public RPCCommand {
  public:
    GetPosVelAccCommand() :
        RPCCommand(GET_POS_VEL_ACC, 0, 4 + 3 * pose2dSize) {}

    int execute(uint8_t UUID,
                const uint8_t *payload,
                uint8_t payloadLength) override {
        if (!validatePayload(payloadLength)) {
            Serial.printf("Incorrect argument length (%d) for `GetPosVelAcc`\n",
                          payloadLength);
            return -1;
        }

        sfe_otos_pose2d_t pose;
        sfe_otos_pose2d_t velocity;
        sfe_otos_pose2d_t acceleration;
        sfeTkError_t error = myOtos.getPosVelAcc(pose, velocity, acceleration);

        uint8_t response[40];
        memcpy(response, &error, sizeof(error));
        memcpy(response + 4, &pose, sizeof(pose));
        memcpy(response + 16, &velocity, sizeof(velocity));
        memcpy(response + 28, &acceleration, sizeof(acceleration));

        sendResponse(UUID, commandID, response, sizeof(response));

        return 0;
    }
};

class GetPosVelAccStdCommand : public RPCCommand {
  public:
    GetPosVelAccStdCommand() :
        RPCCommand(GET_POS_VEL_ACC_STD, 0, 4 + 3 * pose2dSize) {}

    int execute(uint8_t UUID,
                const uint8_t *payload,
                uint8_t payloadLength) override {
        if (!validatePayload(payloadLength)) {
            Serial.printf("Incorrect argument length (%d) for "
                          "`GetPosVelAccStd`\n",
                          payloadLength);
            return -1;
        }

        sfe_otos_pose2d_t poseStdDev;
        sfe_otos_pose2d_t velocityStdDev;
        sfe_otos_pose2d_t accelerationStdDev;
        sfeTkError_t error = myOtos.getPosVelAccStdDev(
            poseStdDev, velocityStdDev, accelerationStdDev);

        uint8_t response[40];
        memcpy(response, &error, sizeof(error));
        memcpy(response + 4, &poseStdDev, sizeof(poseStdDev));
        memcpy(response + 16, &velocityStdDev, sizeof(velocityStdDev));
        memcpy(response + 28, &accelerationStdDev, sizeof(accelerationStdDev));

        sendResponse(UUID, commandID, response, sizeof(response));

        return 0;
    }
};

class GetPosVelAccAndStdCommand : public RPCCommand {
  public:
    GetPosVelAccAndStdCommand() :
        RPCCommand(GET_POS_VEL_ACC_AND_STD, 0, 4 + 6 * pose2dSize) {}

    int execute(uint8_t UUID,
                const uint8_t *payload,
                uint8_t payloadLength) override {
        if (!validatePayload(payloadLength)) {
            Serial.printf("Incorrect argument length (%d) for "
                          "`GetPosVelAccAndStd`\n",
                          payloadLength);
            return -1;
        }

        sfe_otos_pose2d_t pose;
        sfe_otos_pose2d_t velocity;
        sfe_otos_pose2d_t acceleration;

        sfe_otos_pose2d_t poseStd;
        sfe_otos_pose2d_t velocityStd;
        sfe_otos_pose2d_t accelerationStd;

        sfeTkError_t error = myOtos.getPosVelAccAndStdDev(pose,
                                                          velocity,
                                                          acceleration,
                                                          poseStd,
                                                          velocityStd,
                                                          accelerationStd);

        uint8_t response[76];
        memcpy(response, &error, sizeof(error));
        memcpy(response + 4, &pose, sizeof(pose));
        memcpy(response + 16, &velocity, sizeof(velocity));
        memcpy(response + 28, &acceleration, sizeof(acceleration));
        memcpy(response + 40, &poseStd, sizeof(poseStd));
        memcpy(response + 52, &velocityStd, sizeof(velocityStd));
        memcpy(response + 64, &accelerationStd, sizeof(accelerationStd));

        sendResponse(UUID, commandID, response, sizeof(response));

        return 0;
    }
};

/***************************** Non-OTOS Commands *****************************/

class PingCommand : public RPCCommand {
  public:
    PingCommand() : RPCCommand(PING, 1, 1) {}

    int execute(uint8_t UUID,
                const uint8_t *payload,
                uint8_t payloadLength) override {
        if (!validatePayload(payloadLength)) {
            Serial.printf("Incorrect argument length (%d) for `Ping`\n",
                          payloadLength);
            return -1;
        }

        sendResponse(UUID, commandID, payload, 1);

        return 0;
    }
};

class SerialPrintPayloadCommand : public RPCCommand {
  public:
    SerialPrintPayloadCommand() :
        RPCCommand(SERIAL_PRINT_PAYLOAD, UKNOWN_ARG_LEN, UNKNOWN_RET_LEN) {}

    int execute(uint8_t UUID,
                const uint8_t *payload,
                uint8_t payloadLength) override {
        Serial.print("Payload: ");
        for (uint8_t i = 0; i < payloadLength; i++) {
            Serial.print(payload[i], HEX);
            Serial.print(" ");
        }
        Serial.println();

        // Sends back the length of the payload
        sendResponse(UUID,
                     commandID,
                     reinterpret_cast<uint8_t *>(&payloadLength),
                     sizeof(payloadLength));

        return 0;
    }
};

#endif // OTOS_DEFINITIONS_PRESENT

#endif // REQUEST_HANDLER_HPP