from ultralytics import YOLO
import torch


def train_yolo_model():

    if torch.cuda.is_available():
        device = 0
        print("CUDA is available. Using GPU.")
    else:
        device = 'cpu'
        print("⚠CUDA is not available. Using CPU.")

    model = YOLO('yolov8s.pt')

    try:
        results = model.train(
            data=r'F:\한동대\8학기\IAIA\IAIA_robot_project_machine_vision\train\surface-detecting-4\data.yaml',

            # 학습 기본 설정
            epochs=100,
            imgsz=640,
            batch=8,
            device=device,
            patience=30,
            workers=0,
            cache=True,

            optimizer='AdamW',
            lr0=0.001,
            lrf=0.01,
            weight_decay=0.0005,

            mosaic=0.0,
            mixup=0.0,
            flipud=0.0,

            project=r'runs',
            name='yolov8s_surface_defect_v1',
        )

        print("\n학습이 성공적으로 완료되었습니다!")
        print(f"결과 저장 위치: {results.save_dir}")

    except Exception as e:
        print(f"\n학습 중 오류 발생: {e}")


if __name__ == '__main__':
    train_yolo_model()
