import sys
from PIL import Image

def generate_empty_map(file_name, width_meters, height_meters, resolution):
    """
    生成空白地图图片并保存为 PNG 文件。

    Args:
        file_name (str): 输出图片的文件名。
        width_meters (float): 地图的物理宽度（米）。
        height_meters (float): 地图的物理高度（米）。
        resolution (float): 地图分辨率（米/像素）。
    """
    # 计算图片的像素尺寸
    width_px = int(width_meters / resolution)
    height_px = int(height_meters / resolution)

    print(f"生成空白地图: {width_meters}m x {height_meters}m")
    print(f"分辨率: {resolution}m/px")
    print(f"图片尺寸: {width_px}px x {height_px}px")

    # 创建一张全白的灰度图，白色(255)代表自由空间
    img = Image.new('L', (width_px, height_px), 255)

    # 保存图片
    img.save(file_name)
    print(f"地图图片已保存为: {file_name}")

if __name__ == "__main__":
    # 参数校验
    if len(sys.argv) != 5:
        print("用法: python generate_empty_map.py <输出图片名> <宽度(米)> <高度(米)> <分辨率>")
        print("示例: python generate_empty_map.py empty_map.png 200.0 200.0 0.05")
        sys.exit(1)

    # 解析命令行参数
    output_file = sys.argv[1]
    width_m = float(sys.argv[2])
    height_m = float(sys.argv[3])
    resolution_m = float(sys.argv[4])

    generate_empty_map(output_file, width_m, height_m, resolution_m)