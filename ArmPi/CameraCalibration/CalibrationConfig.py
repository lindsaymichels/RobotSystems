from pathlib import Path

#相邻两个角点间的实际距离，单位cm
corners_length = 2.1

#木块边长3cm
square_length = 3

#标定棋盘大小, 列， 行, 指内角点个数，非棋盘格
calibration_size = (7, 7)

# 标定目录（使用当前文件位置，避免硬编码路径）
_BASE_DIR = Path(__file__).resolve().parent

#采集标定图像存储路径
save_path = str(_BASE_DIR / 'calibration_images') + '/'

#标定参数存储路径
calibration_param_path = str(_BASE_DIR / 'calibration_param')

#映射参数存储路径
map_param_path = str(_BASE_DIR / 'map_param')
