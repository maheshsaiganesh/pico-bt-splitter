// BTstack configuration for pico-car-audio (Classic Bluetooth / A2DP only).
// Based on the Pico SDK's test/kitchen_sink/btstack_config.h, minus LE.
#ifndef BTSTACK_CONFIG_H
#define BTSTACK_CONFIG_H

#define ENABLE_LOG_ERROR
#define ENABLE_PRINTF_HEXDUMP

// Buffers and sizes
#define HCI_OUTGOING_PRE_BUFFER_SIZE 4
#define HCI_ACL_PAYLOAD_SIZE (1691 + 4)
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT 4
#define MAX_NR_AVDTP_CONNECTIONS 3      // 2 Booms + iPhone
#define MAX_NR_AVDTP_STREAM_ENDPOINTS 3 // 2 sources (Booms) + 1 sink (iPhone)
#define MAX_NR_AVRCP_CONNECTIONS 3
#define MAX_NR_BTSTACK_LINK_KEY_DB_MEMORY_ENTRIES 2
#define MAX_NR_HCI_CONNECTIONS 3
#define MAX_NR_L2CAP_CHANNELS 12
#define MAX_NR_L2CAP_SERVICES 4
#define MAX_NR_SERVICE_RECORD_ITEMS 4

// Limit ACL/SCO buffers to avoid overrunning the cyw43 shared bus
#define MAX_NR_CONTROLLER_ACL_BUFFERS 6
#define MAX_NR_CONTROLLER_SCO_PACKETS 3

// Controller-to-host flow control, also to protect the cyw43 shared bus
#define ENABLE_HCI_CONTROLLER_TO_HOST_FLOW_CONTROL
#define HCI_HOST_ACL_PACKET_LEN 1024
#define HCI_HOST_ACL_PACKET_NUM 3
#define HCI_HOST_SCO_PACKET_LEN 120
#define HCI_HOST_SCO_PACKET_NUM 3

// Link keys live in flash (TLV on the SDK's flash bank)
#define NVM_NUM_LINK_KEYS 16
#define NVM_NUM_DEVICE_DB_ENTRIES 16

#define HAVE_EMBEDDED_TIME_MS
#define HAVE_ASSERT
#define HCI_RESET_RESEND_TIMEOUT_MS 1000
#define HAVE_BTSTACK_STDIN

#endif
