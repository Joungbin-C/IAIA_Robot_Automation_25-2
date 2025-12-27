import cv2
import os

# 입력 비디오 경로
video_path = r"C:\Users\joung\Downloads\spinnaker_python-4.2.0.88-cp310-cp310-win_amd64\Examples\Python3\SaveToVideo-Uncompressed-18475771-0000.avi"

# 출력 폴더
output_folder = "frames"
os.makedirs(output_folder, exist_ok=True)

# 비디오 열기
cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print("비디오 파일을 열 수 없습니다.")
    exit()

frame_index = 0
save_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 10프레임마다 저장
    if frame_index % 5 == 0:
        output_path = os.path.join(output_folder, f"frame_{frame_index:06d}.jpg")
        cv2.imwrite(output_path, frame)
        save_count += 1

    frame_index += 1

cap.release()
print(f"총 {save_count}개의 프레임 저장 완료!")
