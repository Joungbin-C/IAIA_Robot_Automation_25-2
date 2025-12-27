# Windows에서 PySpin 가상환경 설치 가이드



## 0. 사전 준비

1. **카메라 드라이버 & Spinnaker SDK 설치**
   - PySpin 버전과 **동일한 버전의 Spinnaker SDK**를 먼저 설치
   - 예: PySpin 3.0.0.0 → **Spinnaker SDK 3.0.0.0**
   - [PySpin 및 SDK 다운로드 링크](https://www.teledynevisionsolutions.com/ko-kr/support/support-center/software-firmware-downloads/iis/download-the-latest-spinnaker-software-development-kit/spinnaker-sdk--download-files/)
2. **Python 버전 확인**
   - 지원 버전: **3.5 ~ 3.8, 3.10**
   - 권장: **Python 3.10 (64-bit)** (우분투 20.04이면 3.8 (64-bit) 추천)
   - Python 설치 시 반드시 **64-bit**로 설치
   - 환경 변수에 파이썬이 등록 되어있어야함

------



## 1. Python 설치 확인

```
python --version
```

또는 Python 3.10 명시:

```
py -3.10 --version

# Ubuntu Version
py -3.8 --version
```

------



## 2. 콘다 설치

이 가이드에선 콘다가 미리 설치 되었다는 가정으로 진행

------



## 3. 가상환경 생성 (conda)

Python 3.10 기준 예시:

```
conda create -n pyspin python=3.10 -y

# Ubuntu Version
conda create -n pyspin python=3.8 -y
```

------



## 4. 가상환경 활성화

```
conda activate pyspin
```

성공하면 프롬프트 앞에 `(pyspin)` 표시됨:

```
(pyspin) C:\pyspin_project>
```

------



## 5. pip 최신화

```
python -m ensurepip
python -m pip install --upgrade pip
```

------



## 6. PySpin Wheel 설치

### PySpin wheel 파일 준비

Wheel 파일 위치로 이동

```
cd C:\path\to\spinnaker_python_wheel
```

whl파일 확인

예시:

```
spinnaker_python-3.0.0.0-cp310-cp310-win_amd64.whl
```

반드시 확인:

- `cp310` → Python 3.10
- `win_amd64` → 64-bit Windows

Unbuntu 20.04이면:
- `cp38` → Python 3.8
- 'linux_x86_64` → 64-bit Linux

### 설치

```
pip install spinnaker_python-3.x.x.x-cp310-cp310-win_amd64.whl
```

(실제 파일명으로 변경)

------



## 7. 설치 확인

```
python -c "import PySpin; print(PySpin.__version__)"
```

에러 없으면 성공

------



## 8. 예제 실행

Spinnaker SDK 또는 PySpin 압축 안의 예제 사용:

```
python Examples\Python3\Acquisition.py
```

또는 직접 테스트:

```
import PySpin

system = PySpin.System.GetInstance()
cam_list = system.GetCameras()
print("Number of cameras:", cam_list.GetSize())

cam_list.Clear()
system.ReleaseInstance()
```

------



## 9. 프로젝트를 위한 환경 설치

```
pip install numpy==2.2.6
pip install opencv-python-headless==4.10.0.84
pip install ultralytics==8.3.232
pip install PyQt5==5.15.11
pip install Pillow==9.2.0
```



## 10. 코드 실행 방법

깃허브에 `src/MachineVsion/AcquireAndDisplay_Final.py`을 spinnaker_python 파일에서 `Examples/Python3`에 옮긴 후 실행



## 11. 환경 비활성화

```
deactivate
```
