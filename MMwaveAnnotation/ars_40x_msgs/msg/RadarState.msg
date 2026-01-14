bool rcs_threshold
uint32 invalid_clusters
bool ctrl_relay
uint32 output_type
bool send_quality
bool send_ex_info
uint32 motion_rx_state
uint32 sensor_id
uint32 sort_index
uint32 radar_power
bool voltage_error
bool temporary_error
bool temperature_error
bool interference
bool persistent_error
float64 max_distance
bool nvm_read_status
bool nvm_write_status


# `rcs_threshold`: 一个布尔值，可能表示一个雷达系统中的反射截面（RCS）阈值，用于判断目标的大小。
# `invalid_clusters`: 一个无符号整型，可能表示无效聚类的数量，聚类可能是一组相邻的目标点。
# `ctrl_relay`: 一个布尔值，可能表示控制继电器的状态，可能用于控制雷达系统的开关。
# `output_type`: 一个无符号整型，可能表示输出数据的类型，可能有不同的数据格式。
# `send_quality`: 一个布尔值，可能表示是否发送目标质量信息，用于描述目标的可靠性。
# `send_ex_info`: 一个布尔值，可能表示是否发送额外的信息，可能是关于目标的附加信息。
# `motion_rx_state`: 一个无符号整型，可能表示雷达接收到的运动状态信息。
# `sensor_id`: 一个无符号整型，可能表示传感器的唯一标识符。
# `sort_index`: 一个无符号整型，可能表示目标排序索引，用于确定目标的优先级。
#- `radar_power`: 一个无符号整型，可能表示雷达的功率水平。
#- `voltage_error`: 一个布尔值，可能表示是否存在电压错误。
#- `temporary_error`: 一个布尔值，可能表示是否存在临时错误。
#- `temperature_error`: 一个布尔值，可能表示是否存在温度错误。
#- `interference`: 一个布尔值，可能表示是否存在干扰。
#- `persistent_error`: 一个布尔值，可能表示是否存在持续错误。
#- `max_distance`: 一个双精度浮点数，可能表示最大检测距离。
#- `nvm_read_status`: 一个布尔值，可能表示非易失性内存（NVM）读取状态。
#- `nvm_write_status`: 一个布尔值，可能表示非易失性内存（NVM）写入状态。
#  这些只是猜测，实际含义可能取决于具体的应用和上下文。


