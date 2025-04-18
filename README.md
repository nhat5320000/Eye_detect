# AI_detect

Yolov11 User Manual for Jetson-Nano Jetpack 4.6

---

## **Features**
- **Real-time Detection**: Identify and classify in real time.  
- **Pretrained Models**: Includes YOLOv11 and MobileNetV2 for efficient performance.  
- **WebView Integration**: Visualize detection and classification results in real time via FastAPI.  
- **Flexible Training Pipeline**: Simplified end-to-end training for detection and classification models.  

---

## **Use Case**
Not enough money to buy Jetson-Orin ...  

> **Note:** This project is under development and requires further optimization for real-world deployment. Enhancements in accuracy, responsiveness, and robustness are ongoing.

---


## **Requirements**

### **Hardware**
- NVIDIA Jetson Nano  
- USB or Raspberry Pi Camera  

### **Software**
- **JetPack SDK 4.6**  
- **Python 3.8**  

---

## **Project Structure**
```plaintext
├── datasets                   # Datasets for detection and classification
│   ├── eyes-detect-dataset    # Eye detection dataset
│   └── eyes-state-dataset     # Eye state classification dataset
├── Dockerfile                 # Docker configuration
├── Readme.md                  # Documentation
├── requirements.txt           # Python dependencies
├── src                        # Application source code
│   └── main.py                # Main script (includes FastAPI WebView server)
├── training                   # Model training resources
│   ├── eyes-detect            # Eye detection model training
│   └── eye-state              # Eye state classification model training
```

---

## **Setup**

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/nhat5320000/Eye_Detect.git
   cd Eye_Detect/
   ```

---

## **Running the System**

### **Option 1: Using Docker (Recommended for camera USB)**

Ensure the camera is connected to `/dev/video0` and start the system using Docker:  
1. **Build the Docker Image**:
   If you haven't already built the Docker image, run the following command to build it:
   ```bash
   docker build -t eye-state-detection .
   ```
2. **Run the System Using Docker**: After the image is built, start the system using Docker(main.py with API):

   ```bash
   docker run --rm --device /dev/video0 --runtime nvidia -p 8000:8000 eye-state-detection
   ```
3. **Run the System Using Docker with USB cam**: Direct display on jetson-nano(main1.py with cv2.imshow):

   ```bash
   run --rm --device /dev/video0 --runtime nvidia   -e DISPLAY=:0   -v /tmp/.X11-unix:/tmp/.X11-unix:rw   eye-state-detection
   ```
If it cannot be displayed on cv2.imshows refer to Permisson
### **Option 2: Using vitural env 3.8(Recommended for camera CSI)**

1. Creating a python3.8 virtual environment:
   ```bash
   # Option 1 to download UV.
   wget -qO- https://astral.sh/uv/install.sh | sh
   # Option 2 to download UV.
   curl -LsSf https://astral.sh/uv/install.sh | less
   # Create virtual env 3.8
   uv venv -p 3.8
   ```

2. Activate ENV:
   ```bash
   source .venv/bin/activate
   ```
3. Download package:
   ```bash
   sudo apt install libopenmpi-dev libopenblas-base libomp-dev gcc
   sudo apt-get install cmake
   sudo apt-get install gcc g++
   sudo apt-get install python3-dev python3-numpy
   sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev
   sudo apt-get install libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev
   sudo apt-get install libgtk2.0-dev
   sudo apt-get install libgtk-3-dev
   ```
4. Download onxx and TRT:
   ```bash
   wget https://nvidia.box.com/shared/static/gjqofg7rkg97z3gc8jeyup6t8n9j8xjw.whl onnxruntime_gpu-1.8.0-cp38-cp38-linux_aarch64.whl
   wget https://forums.developer.nvidia.com/uploads/short-url/hASzFOm9YsJx6VVFrDW1g44CMmv.whl tensorrt-8.2.0.6-cp38-none-linux_aarch64.whl
   wget https://github.com/ultralytics/assets/releases/download/v0.0.0/torch-1.11.0a0+gitbc2c6ed-cp38-cp38-linux_aarch64.whl
   wget https://github.com/ultralytics/assets/releases/download/v0.0.0/torchvision-0.12.0a0+9b5a3fe-cp38-cp38-linux_aarch64.whl
   uv pip install *.whl
   ```
5. Download ultralytics and package:
   ```bash
   uv pip install ultralytics
   uv pip install --no-cache-dir "onnx>=1.12.0" "onnxslim"
   uv pip install numpy==1.23.5
   uv pip install pip
   uv pip install pymodbus
   ```
6. Download OpenCV for CSI:
   ```bash
   OPENCV_VER="master"
   TMPDIR=$(mktemp -d)
   # Build and install OpenCV from source.
   cd "${TMPDIR}"
   git clone --branch ${OPENCV_VER} --depth 1 --recurse-submodules --shallow-submodules https://github.com/opencv/opencv-python.git opencv-python-${OPENCV_VER}
   cd opencv-python-${OPENCV_VER}
   export ENABLE_CONTRIB=0
   export ENABLE_HEADLESS=0
   # We want GStreamer support enabled.
   export CMAKE_ARGS="-DWITH_QT=ON -DWITH_GTK=OFF -DWITH_GSTREAMER=ON"
   python3 -m pip wheel . --verbose
   # Install OpenCV
   python3 -m pip install opencv_python*.whl
   ```
If using a USB camera without opencv installed.LINK documents:(https://hub.docker.com/r/ultralytics/ultralytics)

---

## **Visualizing Results**

The FastAPI WebView for real-time visualization starts automatically with the main application.  

1. **Access the WebView**:  
   Open your browser and navigate to `http://<jetson-nano-ip>:8000` (replace `<jetson-nano-ip>` with your device's IP address).  

The WebView will display:  
- Live video feed with detection overlays.  
- Real-time logs of eye state and warnings.

---

## **Training**

### **Prepare Datasets**

Ensure datasets are ready before training:  
- **Eye Detection Dataset**: [datasets/eyes-detect-dataset/readme.md](datasets/eyes-detect-dataset/readme.md)  
- **Eye State Classification Dataset**: [datasets/eyes-state-dataset/readme.md](datasets/eyes-state-dataset/readme.md)  

### **Training Instructions**

Detailed training steps are available in the respective directories:  
- **Detection Model**: [training/eyes-detect/readme.md](training/eyes-detect/readme.md)  
- **Eye State Classifier**: [training/eye-state/readme.md](training/eye-state/readme.md)  

> **Training Recommendation:** Perform training on a system with an NVIDIA GPU, not the Jetson Nano.

---

## **Acknowledgments**

This project utilizes:  
- **NVIDIA Jetson Nano** for edge computing.  
- **YOLOv11** for eye detection.  
- **MobileNetV2** for efficient classification.  
- **FastAPI** for serving the WebView.

---

### **Permision**

1. Install the required dependencies:
   ```bash
   sudo apt update
   sudo apt install xorg openbox xauth
   echo $DISPLAY
   ps -e | grep X
   export DISPLAY=:0
   xhost +local:docker
   sudo docker run --rm --device /dev/video0 --runtime nvidia   -e DISPLAY=:0   -v /tmp/.X11-unix:/tmp/.X11-unix:rw   eye-detect
   ```
