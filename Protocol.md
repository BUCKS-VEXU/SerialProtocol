# V5 Brain ESP32 Serial Protocol
# Message Structure

## Request

| **Index** | **Length**  | **Description**                                |
|-----------|-------------|-----------------------------------------------|
| 0         | 1           | **Start Marker (0x7E):** Begins a request     |
| 1         | 1           | **UUID:** Unique identifier used to distinguish returned responses |
| 2         | 1           | **Command ID:** The ID of the command to execute |
| 3         | 1           | **Payload Length:** Specifies how long the payload is, since its length varies |
| 4         | n           | **Payload:** The parameters of the request    |
| 4+n       | 1           | **Checksum:** Used for transmission error catching |
| 5+n       | 1           | **End Marker (0x7E):** Ends a request         |

## Response

| **Index** | **Length**  | **Description**                                |
|-----------|-------------|-----------------------------------------------|
| 0         | 1           | **Start Marker (0x7E):** Begins a response    |
| 1         | 1           | **UUID:** Unique identifier used to distinguish this response |
| 2         | 1           | **Command ID:** The ID of the command executed that generated this response |
| 3         | 1           | **Payload Length:** Specifies how long the payload is, since its length varies |
| 4         | n           | **Payload:** The data returned by the executed command |
| 4+n       | 1           | **Checksum:** Used for transmission error catching |
| 5+n       | 1           | **End Marker (0x7E):** Ends a response        |

# Errors
A Command ID of 0xFF indicates that an error has occurred. An error code will be specified in the payload

<!-- TODO need some way to indicate a checksum error on the Client, currently it just sends nothing back  -->
