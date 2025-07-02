# Pose Optimizer
최적화 변수로 pose를 사용하는 Levenberg-Marquardt 기반의 비선형 최적화 라이브러리

## 1. Installation
### Dependencies
- Eigen library : https://eigen.tuxfamily.org/

### Clone repository
```
git clone "https://github.com/ChanghyeonKim93/pose_optimizer.git"
cd pose_optimizer
mkdir build && cd build
cmake .. && make -j{$nproc}
```

## 2. Contents
#### Single pose optimizer
* 단일 포즈를 변수로하는 최적화 문제. 일반적인 frame-to-frame reprojection error minimization, scan matching 등의 문제를 풀 수 있음.