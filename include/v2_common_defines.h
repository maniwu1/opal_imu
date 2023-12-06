#ifndef V2_COMMON_DEFINES_H_
#define V2_COMMON_DEFINES_H_


#if defined(MONITOR_FIRMWARE) || defined(SMART_SOX)
//setting USE_SMART_SOX_BUFFERS to 1 here enables smartsox monitor functions, and requires the smartsox sensor head
#define USE_SMART_SOX_BUFFERS               0
#else
//setting USE_SMART_SOX_BUFFERS to 1 here impacts the host libraries
#define USE_SMART_SOX_BUFFERS               0
#endif

#define USE_SMART_SOX_BUFFERS_CAL           USE_SMART_SOX_BUFFERS

#define USE_SMART_SOX_BUFFERS_TEMPERATURE   0

//enables carrier board functions like carrier LEDs
#define SOX_CARRIER_V2_18                   USE_SMART_SOX_BUFFERS

#if USE_SMART_SOX_BUFFERS
#define USE_SENSOR_IRQ_LATENCIES            0
#define USE_HIGH_SAMPLE_RATE                0
#else
#define USE_SENSOR_IRQ_LATENCIES            1
#define USE_HIGH_SAMPLE_RATE                1
#endif

#define APDM_USB_VENDOR_ID                  0x224F
#define APDM_V2_MONITOR_CDC_PID             0x0004
#define APDM_V2_MONITOR_PID                 0x0005
#define APDM_V2_DOCK_PID                    0x0006
#define APDM_V2_ACCESS_POINT_CDC_PID        0x0007
#define APDM_V2_ACCESS_POINT_PID            0x0008

#define SSOX_HEARTBEAT_MAX_AGE              500
#define SSOX_WARMUP_TIME                    1000


#endif /* V2_COMMON_DEFINES_H_ */
