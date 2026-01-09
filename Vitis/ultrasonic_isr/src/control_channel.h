/*
 * Control Channel Header
 * TCP server on port 6000 for receiving control packets
 * Protocol: 12-byte packets [Signature:4][ParamID:4][ParamValue:4]
 */

#ifndef CONTROL_CHANNEL_H
#define CONTROL_CHANNEL_H

#include <stdint.h>

// Number of configurable parameters (IDs 0–9)
#define NUM_PARAMETERS 10

// Initialize control channel TCP server on port 6000
// Returns 0 on success, -1 on error
int start_control_channel(void);

// Check if control client is connected
// Returns 1 if connected, 0 otherwise
int is_control_connected(void);

// Get parameter value by ID (0–9)
// Returns parameter value, or 0 if invalid ID
uint32_t get_parameter(uint32_t param_id);

// Set parameter value by ID (0–9)
// Returns 0 on success, -1 if invalid ID
int set_parameter(uint32_t param_id, uint32_t value);

#endif // CONTROL_CHANNEL_H

