/* Noah Klein */

#ifndef PROTOCOL_DEFINITIONS_HPP
#define PROTOCOL_DEFINITIONS_HPP

#include <cstdint>
#include <memory>
#include <vector>

#define START_MARKER 0x7E
#define END_MARKER 0x7F

typedef std::shared_ptr<std::vector<uint8_t>> payload_t;

struct SerialRequest {
    uint8_t UUID;
    uint8_t commandID;
    uint8_t payloadLength;
    payload_t payload;
    uint8_t checksum;
};

struct SerialResponse {
    uint8_t UUID;
    uint8_t commandID;
    uint8_t payloadLength;
    payload_t payload;
    uint8_t checksum;
};

enum RPCCommandID {
    PING = 0x00,
    SERIAL_PRINT_PAYLOAD = 0xFF,

    IS_CONNECTED = 0x01,

    CALIBRATE_IMU = 0x02,
    GET_IMU_CALIBRATION_PROGRESS = 0x03,
    RESET_TRACKING = 0x04,
    GET_STATUS = 0x05,

    GET_OFFSET = 0x06,
    SET_OFFSET = 0x07,

    GET_POSE = 0x08,
    GET_POSE_STD = 0x09,

    GET_VELOCITY = 0x0A,
    GET_VELOCITY_STD = 0x0B,

    GET_ACCELERATION = 0x0C,
    GET_ACCELERATION_STD = 0x0D,

    GET_POS_VEL_ACC = 0x0E,
    GET_POS_VEL_ACC_STD = 0x0F,
    GET_POS_VEL_ACC_AND_STD = 0x10,
};

/**
 * @brief Prints the payload to the ESP32 debug serial
 *
 * @param string the payload to print
 * @return uint8_t the number of bytes printed
 */
// uint8_t SerialPrintPayload(uint8_t *string);

/// @brief 2D pose structure, compatible with Lem and OTOS
struct pose2d_t {
    float x;
    float y;
    float theta;
};

#ifndef OTOS_DEFINITIONS_PRESENT

/**
 * General Concept
 *    A SparkFun Toolkit error system. The goal is to keep this simple.
 *
 *    This mimics a variety of systems, using an int type for error codes,
 *    where:
 *   		0   = okay
 *         -1   = general failure
 *         >0   = an informative error
 *
 *    Since *subsystems* in the toolkit can have their own errors,
 *    A start range for these errors are defined. Values > than this value
 *    define the errors for the set subsystem. These start ranges are set
 *    in this file, with actual error values defined in the the respective
 *    subsystem header files.
 *
 */
typedef int32_t sfeTkError_t;

/// @union sfe_otos_status_t
/// @brief Status register bit fields
typedef union {
    struct {
        /// @brief Returns 1 if the tilt angle threshold has been exceeded.
        /// While set, the accelerometer data is ignored
        uint8_t warnTiltAngle : 1;

        /// @brief Returns 1 if the optical tracking is unreliable. While set,
        /// only the IMU data is used for tracking unless warnTiltAngle is set
        uint8_t warnOpticalTracking : 1;

        /// @brief Reserved bits, do not use
        uint8_t reserved : 4;

        /// @brief Returns 1 if the optical sensor has a fatal error
        uint8_t errorPaa : 1;

        /// @brief Returns 1 if the IMU has a fatal error
        uint8_t errorLsm : 1;
    };

    /// @brief Raw register value
    uint8_t value;
} sfe_otos_status_t;

#endif // OTOS_DEFINITIONS_PRESENT

#endif // PROTOCOL_DEFINITIONS_HPP
