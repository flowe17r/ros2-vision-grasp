# 单个目标的二维检测结果（像素坐标，坐标系由 header.frame_id 给出）
std_msgs/Header header

# 目标颜色标签，如 red / green / blue
string color_name

# 轮廓质心（像素）
float32 u
float32 v

# 轮廓面积（像素）
float32 area_px

# 外接框左上角与宽高（像素）
float32 bbox_x
float32 bbox_y
float32 bbox_w
float32 bbox_h

# 轮廓面积 / 外接框面积，1.0 表示完全填满；用来过滤碎片
float32 fill_ratio
