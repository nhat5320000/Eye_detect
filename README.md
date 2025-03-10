# Eye_detect

A compact, real-time system designed for NVIDIA Jetson Nano to detect whether a person’s eyes are open or closed. This project integrates YOLO for eye detection and a PyTorch-based classifier for state classification, with a WebView to visualize real-time results using FastAPI.

---

## **Features**
- **Real-time Detection**: Identify and classify eye states in real time.  
- **Pretrained Models**: Includes YOLOv11 and MobileNetV2 for efficient performance.  
- **WebView Integration**: Visualize detection and classification results in real time via FastAPI.  
- **Flexible Training Pipeline**: Simplified end-to-end training for detection and classification models.  

---

## **Use Case**
This project is designed for **driver drowsiness detection**. The system monitors the driver’s eyes in real time, and if the driver’s eyes are detected as closed or not visible for a prolonged period, it triggers a warning to alert the driver.  

> **Note:** This project is under development and requires further optimization for real-world deployment. Enhancements in accuracy, responsiveness, and robustness are ongoing.

---

## **Workflow**

1. **Capture Video Frame**: The system continuously captures image frames from the video feed.  
2. **Eye Detection**:  
   - **If eyes are not detected**: Trigger a warning to the driver.  
   - **If eyes are detected**: Proceed to the next step.  
3. **Eye State Classification**:  
   - Check the state (open or closed) of each detected eye.  
   - **If any eye is detected as closed**: Trigger a warning to the driver.  
4. **WebView Visualization**: Display real-time detection and classification results in a web interface powered by FastAPI.  
5. **Real-time Processing**: This process is repeated continuously to ensure timely alerts.  

---

## **Limitations and Optimization Opportunities**

### **Limitations**
- **MobileNetV2 Accuracy**: While MobileNetV2 offers efficient classification, it sometimes misclassifies the eye state due to its lightweight architecture.  
- **Memory Constraints**: The NVIDIA Jetson Nano’s limited memory restricts the use of larger, more accurate models.
- **Single Client Stream Limitation**: FastAPI can only serve a video stream to one client at a time, which limits scalability for multiple viewers.

### **Planned Optimizations**
1. **Model Compression**: Optimize larger models (e.g., EfficientNet or ResNet) using techniques like quantization or pruning to make them compatible with Jetson Nano.
2. **RTSP Server Setup**: Integrate an RTSP server to handle multiple concurrent video streams, allowing scalability for real-time visualization across multiple clients.
> **Discussion Point:** If you have suggestions for other optimization strategies, please share your ideas.

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
   git clone https://github.com/vodinhvu/Eye_State_Detect.git
   cd Eye_State_Detect/
   ```

---

## **Running the System**

### **Option 1: Using Docker (Recommended)**

Ensure the camera is connected to `/dev/video0` and start the system using Docker:  
1. **Build the Docker Image**:
   If you haven't already built the Docker image, run the following command to build it:
   ```bash
   docker build -t eye-state-detection .
   ```
2. **Run the System Using Docker**: After the image is built, start the system using Docker:

   ```bash
   docker run --rm --device /dev/video0 --runtime nvidia -p 8000:8000 eye-state-detection
   ```

### **Option 2: Using Host Python**

1. Install the required dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Run the application:
   ```bash
   python src/main.py
   ```

The FastAPI WebView is automatically started alongside the main application.

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
- **MobileNetV2** for efficient eye state classification.  
- **FastAPI** for serving the WebView.

---

### **Option 2: Using Host Python**

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
