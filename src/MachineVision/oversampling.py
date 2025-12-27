import os
import glob
import shutil
import random
from collections import defaultdict
from tqdm import tqdm

LABEL_DIR = "surface-detecting-4/test/labels"
IMAGE_DIR = "surface-detecting-4/test/images"

IMG_EXTENSIONS = ['.jpg', '.jpeg', '.png', '.bmp']


def get_class_counts(label_files):
    class_counts = defaultdict(int)
    files_by_class = defaultdict(list)

    print("데이터 분석 중...")
    for label_file in tqdm(label_files):
        with open(label_file, 'r') as f:
            lines = f.readlines()

        existing_classes_in_file = set()

        for line in lines:
            parts = line.strip().split()
            if len(parts) > 0:
                class_id = int(parts[0])
                class_counts[class_id] += 1
                existing_classes_in_file.add(class_id)

        for cls in existing_classes_in_file:
            files_by_class[cls].append(label_file)

    return class_counts, files_by_class


def find_image_file(label_file, image_dir):
    base_name = os.path.splitext(os.path.basename(label_file))[0]
    for ext in IMG_EXTENSIONS:
        img_path = os.path.join(image_dir, base_name + ext)
        if os.path.exists(img_path):
            return img_path
    return None


def oversample_dataset():
    label_files = glob.glob(os.path.join(LABEL_DIR, "*.txt"))
    if not label_files:
        print("라벨 파일을 찾을 수 없습니다. 경로를 확인해주세요.")
        return

    # 현재 클래스별 개수 파악
    class_counts, files_by_class = get_class_counts(label_files)

    if not class_counts:
        print("데이터셋이 비어있거나 형식이 잘못되었습니다.")
        return

    # 목표 수량 설정
    target_count = max(class_counts.values())
    print(f"\n[분석 결과]")
    for cls, count in sorted(class_counts.items()):
        print(f"Class {cls}: {count} instances")
    print(f"-> 목표 인스턴스 수 (Target): {target_count}")

    # 오버샘플링
    print("\nOversampling 시작...")

    # 0부터 최대 클래스 ID까지 순회
    max_class_id = max(class_counts.keys())

    for cls in range(max_class_id + 1):
        current_count = class_counts[cls]
        diff = target_count - current_count

        if diff <= 0:
            continue

        print(f"Class {cls} 보강 중... ({current_count} -> {target_count}, 부족분: {diff})")

        # 해당 클래스가 포함된 파일 후보군
        candidates = files_by_class[cls]
        if not candidates:
            print(f"Warning: Class {cls}를 포함한 원본 이미지가 없습니다. 건너뜁니다.")
            continue

        added_count = 0
        while added_count < diff:
            src_label_path = random.choice(candidates)
            src_img_path = find_image_file(src_label_path, IMAGE_DIR)

            if src_img_path is None:
                continue

            with open(src_label_path, 'r') as f:
                lines = f.readlines()
                instance_in_img = sum(1 for line in lines if int(line.split()[0]) == cls)

            # 새로운 파일명 생성
            base_name = os.path.splitext(os.path.basename(src_label_path))[0]
            new_name = f"{base_name}_aug_{random.randint(1000, 99999)}"

            dst_label_path = os.path.join(LABEL_DIR, new_name + ".txt")
            dst_img_path = os.path.join(IMAGE_DIR, new_name + os.path.splitext(src_img_path)[1])

            # 파일 복사
            shutil.copy(src_img_path, dst_img_path)
            shutil.copy(src_label_path, dst_label_path)

            added_count += instance_in_img

    print("\n모든 작업이 완료되었습니다!")


if __name__ == "__main__":
    oversample_dataset()