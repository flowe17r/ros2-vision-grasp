# MCU 周期状态帧（cmd=0x11）解析结果
uint8 seq
uint32 uptime_ms
float32 temperature_c
uint16 error_code
float32[6] encoders
uint32 mcu_crc_errors
uint32 bridge_crc_errors
