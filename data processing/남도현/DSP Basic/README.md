# Data Signal Processing (DSP) Project
확장 칼만 필터(EKF)를 이용한 궤적 추정 알고리즘을 구현, 로켓이 비행하는 동안 얻은 IMU, GPS, 기압 센서 데이터를 종합하여 처리하고 비행 궤적을 계산하는 하나로 전자팀 산하 프로젝트입니다.

This project processes and visualizes IMU (Inertial Measurement Unit), GPS, and pressure sensor data from a flight trajectory, implementing various trajectory estimation methods including Extended Kalman Filter (EKF).

## Project Structure
### Core Files
- `main.py`: Entry point of the application. Handles data loading and visualization of trajectory estimates.
- `data.py`: Contains the main `data_format` class that processes raw sensor data and implements the EKF algorithm.
- `plot_trajectory.py`: Implements various trajectory estimation methods:
  - `IMUTrajectory`: Dead reckoning using IMU data with RK4 integration
  - `GPSTrajectory`: Position estimation using GPS coordinates
  - `SIMPLETrajectory`: Basic trajectory estimation based on simple velocity integration
- `quaternion.py`: Quaternion mathematics utilities for 3D rotation calculations
- `config.py`: Configuration parameters for visualization and processing

### Supporting Files
- `imu.py`: IMU data processing utilities
- `label.py`: Data labeling utilities
- `preprocessing.py`: Data preprocessing functions
- `dsp.py`: Digital signal processing utilities

## Usage
`main.py` 스크립트를 실행하면 IMU, GPS, 그리고 기압 센서 데이터를 기록한 CSV 파일을 처리하여 비행 궤적을 구하고, 다음의 세 가지 방식으로 추정한 경로를 각각 시각화하여 나타냅니다.  
The main script processes data from a CSV file containing IMU, GPS, and pressure sensor readings. It visualizes three different trajectory estimates:
- Kalman filter-based trajectory (red)
- GPS-based trajectory (green)
- Simple integration-based trajectory estimation (blue)

## Dependencies
- pandas
- numpy
- matplotlib
- scipy
