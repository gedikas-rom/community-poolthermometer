/**
 * sensor_message.h
 *
 * Must be identical in all projects:
 *   - ESP-NOW nodes   (know only sensor_message)
 *   - ESP32 bridge    (knows both structs)
 *   - Poolcontrol     (knows both structs)
 *
 * Two structs, two transport paths:
 *
 *   sensor_message  -> transmitted via ESP-NOW (nodes <-> bridge)
 *   uart_message    -> transmitted via UART    (bridge <-> poolcontrol)
 *                     Wrapper: sensor_message + node MAC
 */

#pragma once
#include <stdint.h>
#include <stddef.h>

// sensor_message
// ESP-NOW payload. Nodes only know this struct.
//
//   id:      Identifier or command:
//              "pool/temp"      -> Node sends temperature data
//              "pool/chemistry" -> Node sends chemistry data
//              "pool/level"     -> Node sends fill level
//              "get-values"     -> Node requests values from Poolcontrol
//                                 payload then contains which values are requested
//                                 e.g. {"req":["water_temp","solar_temp"]} or empty ("") for all values
//
//   payload: JSON string with data or request parameters. Empty ("") is valid.
//
typedef struct sensor_message {
    char id[20];        // max 19 chars + '\0'
    char payload[180];  // max 179 chars + '\0'
} sensor_message;

static_assert(sizeof(sensor_message) == 200, "sensor_message must be exactly 200 bytes");


// uart_message
// UART wrapper. Used only on the bridge <-> poolcontrol link.
// Nodes do NOT know this struct.
//
//   mac:  6-byte MAC address of the ESP-NOW node.
//           Direction bridge -> poolcontrol:  MAC of the sending node
//           Direction poolcontrol -> bridge:  MAC of the target node (response receiver)
//
//   msg:  The actual sensor_message.
//
typedef struct uart_message {
    uint8_t        mac[6];  // Node MAC address (6 bytes)
    sensor_message msg;     // Message          (200 bytes)
} uart_message;             // Total:            206 bytes

static_assert(sizeof(uart_message) == 206, "uart_message must be exactly 206 bytes");
