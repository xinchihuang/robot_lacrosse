
import os.path
import shutil

# 定义源文件路径和目标文件路径


# 复制文件

model_error=[]
for number in range(100):
    if os.path.isfile('saved_ball_data/' + str(number) + '.npy'):
        source_path_b = 'saved_ball_data/' + str(number) + '.npy'
        destination_path_b = 'selected_data/ball_data/' + str(number) + '.npy'
        source_path_a = 'saved_arm_data/' + str(number) + '.npy'
        destination_path_a = 'selected_data/arm_data/' + str(number) + '.npy'
        # 复制文件
        shutil.copy(source_path_b, destination_path_b)
        shutil.copy(source_path_a, destination_path_a)

