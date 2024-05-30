import os
from datetime import datetime

# 设置数据文件的目录
directory = "./data"

# 获取所有csv文件
files = [f for f in os.listdir(directory) if f.startswith("carla_") and f.endswith(".csv")]

# 如果没有找到文件，就退出
if not files:
    print("No data files found.")
    exit()

# 解析文件名中的日期时间信息
def parse_timestamp(file_name):
    try:
        # 尝试解析标准文件名
        return datetime.strptime(file_name, "carla_%Y%m%d%H%M%S.csv")
    except ValueError:
        # 尝试解析已标注为invalid的文件名
        try:
            return datetime.strptime(file_name, "carla_%Y%m%d%H%M%S_invalid.csv")
        except ValueError:
            # 如果两种格式都无法解析，则返回None
            return None

# 过滤并排序文件，确保所有文件名都能正确解析
valid_files = [file for file in files if parse_timestamp(file)]
valid_files.sort(reverse=True, key=lambda x: parse_timestamp(x))

# 检查最新的文件是否已标注为无效
latest_file = valid_files[0] if valid_files else None

if latest_file and "_invalid" in latest_file:
    print("The most recent file has already been marked as invalid.")
elif latest_file:
    # 构建新的文件名，加上 'invalid' 标记
    new_filename = f"{latest_file[:-4]}_invalid.csv"

    # 重命名文件
    os.rename(os.path.join(directory, latest_file), os.path.join(directory, new_filename))

    print(f"Renamed '{latest_file}' to '{new_filename}'")
else:
    print("No file to rename.")
