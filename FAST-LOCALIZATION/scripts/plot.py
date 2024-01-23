import pandas as pd
import matplotlib.pyplot as plt

# CSV 파일 읽기
file_path1 = '/home/lee/fitness_scores1(0.4,0.1).csv'
file_path2 = '/home/lee/fitness_scores2(0.4,0.1).csv'

data1 = pd.read_csv(file_path1)
data2 = pd.read_csv(file_path2)

# 데이터 확인
print(data1.head())
print(data2.head())

# Plot 생성
plt.figure(figsize=(10, 5))

# 첫 번째 파일의 데이터 플로팅
plt.plot(data1['timestamp'], data1['fitness_score'], label='File 1')

# 두 번째 파일의 데이터 플로팅
plt.plot(data2['timestamp'], data2['fitness_score'], label='File 2')

# 레이블 및 타이틀 추가
plt.xlabel('Timestamp')
plt.ylabel('Fitness Score')
plt.title('Fitness Score over Time')
plt.legend()

# Plot 표시
plt.show()