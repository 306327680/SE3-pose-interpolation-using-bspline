# SE(3) Pose Interpolation using B-Spline

A C++ implementation of B-spline-based pose interpolation on the SE(3) manifold for continuous-time trajectory estimation.

![Visualization](pic.png)

## Overview

This project implements cumulative B-spline interpolation for smooth pose estimation in SE(3) (Special Euclidean Group). Given four discrete poses (T_{i-1}, T_i, T_{i+1}, T_{i+2}), it enables continuous-time pose estimation at any time t between T_i and T_{i+1}.

This approach is particularly useful for:
- Continuous-time trajectory optimization
- Visual-Inertial odometry
- LiDAR odometry and mapping
- Motion planning and control
- Sensor fusion applications

## Theoretical Background

### SE(3) Manifold

SE(3) represents the group of rigid body transformations in 3D space, consisting of rotation and translation:

```
SE(3) = {T = [R t; 0 1] | R ∈ SO(3), t ∈ R³}
```

### B-Spline Interpolation on SE(3)

Traditional B-splines work in Euclidean spaces, but poses lie on the SE(3) manifold. This implementation uses the cumulative form of B-splines with Lie algebra operations:

```
T(u) = exp(ξ₁) · exp(B₁(u) · log(T₁⁻¹T₂)) · exp(B₂(u) · log(T₂⁻¹T₃)) · exp(B₃(u) · log(T₃⁻¹T₄))
```

Where:
- `u ∈ [0,1]` is the interpolation parameter
- `B₁(u) = (5 + 3u - 3u² + u³) / 6`
- `B₂(u) = (1 + 3u + 3u² - 2u³) / 6`
- `B₃(u) = u³ / 6`
- `log()` is the SE(3) logarithmic map
- `exp()` is the SE(3) exponential map

This formulation ensures:
- Smooth interpolation on the manifold
- Proper handling of rotational discontinuities
- C² continuity of the trajectory

## Features

- **Manifold-aware interpolation**: Proper handling of SE(3) geometry using Lie group operations
- **Cumulative B-spline formulation**: Numerically stable implementation
- **Continuous-time estimation**: Query poses at arbitrary timestamps
- **Visualization support**: Generate point cloud visualizations of trajectories
- **Efficient computation**: Optimized using Eigen and Sophus libraries

## Dependencies

The project requires the following libraries:

- **CMake** (>= 2.8)
- **Eigen3**: Linear algebra library
- **Sophus**: Lie group library for SE(3) operations
- **PCL** (Point Cloud Library): For point cloud processing and visualization
- **OpenCV**: Computer vision library
- **fmt**: Modern formatting library

### Installation on Ubuntu/Debian

```bash
# Install system dependencies
sudo apt-get update
sudo apt-get install cmake libeigen3-dev libopencv-dev libpcl-dev libfmt-dev

# Install Sophus (if not available in repositories)
git clone https://github.com/strasdat/Sophus.git
cd Sophus
mkdir build && cd build
cmake ..
sudo make install
```

## Building

```bash
# Clone the repository
git clone https://github.com/yourusername/SE3-pose-interpolation-using-bspline.git
cd SE3-pose-interpolation-using-bspline

# Create build directory
mkdir build && cd build

# Configure and build
cmake ..
make

# Run the example
./percent
```

## Usage

### Basic Example

```cpp
#include "SplineFusion.h"

// Create SplineFusion instance
SplineFusion sf;

// Define four poses
Eigen::Isometry3d T1, T2, T3, T4;
// ... initialize poses ...

// Interpolate at u = 0.5 (midpoint between T2 and T3)
double u = 0.5;
Eigen::Isometry3d T_interpolated = sf.cumulativeForm(T1, T2, T3, T4, u);

// Calculate interpolation parameter from timestamps
double t = 1.5;      // Query time
double ti = 1.0;     // T2 timestamp
double dt = 1.0;     // Time interval
double u = sf.getUt(t, ti, dt);  // u = 0.5
```

### API Reference

#### `SplineFusion` Class

**Constructor**
```cpp
SplineFusion()
```
Default constructor.

**Methods**

```cpp
Eigen::Isometry3d cumulativeForm(
    Eigen::Isometry3d T_1,
    Eigen::Isometry3d T_2,
    Eigen::Isometry3d T_3,
    Eigen::Isometry3d T_4,
    double u
)
```
Compute interpolated pose using cumulative B-spline formulation.
- **Parameters:**
  - `T_1, T_2, T_3, T_4`: Four control poses
  - `u`: Interpolation parameter in [0, 1]
- **Returns:** Interpolated pose between T_2 and T_3

```cpp
double getUt(double t, double ti, double dt)
```
Convert timestamp to interpolation parameter.
- **Parameters:**
  - `t`: Query timestamp
  - `ti`: Start timestamp (corresponding to T_2)
  - `dt`: Time interval between poses
- **Returns:** Normalized parameter u ∈ [0, 1]

```cpp
void test()
```
Run visualization demo generating point cloud trajectories.

## Output

The example program generates two PCD files:
- `temp1.pcd`: Trajectory poses visualization
- `pose.pcd`: Transformed coordinate frames along trajectory
- `cp_save.pcd`: Control pose coordinate frames

Visualize using PCL viewer:
```bash
pcl_viewer temp1.pcd pose.pcd
```

## Mathematical Details

### Lie Group Operations

The implementation uses Sophus library for SE(3) operations:

1. **Logarithmic map**: `log: SE(3) → se(3)`
   - Maps group element to Lie algebra
   - Represents relative transformation as twist

2. **Exponential map**: `exp: se(3) → SE(3)`
   - Maps Lie algebra to group element
   - Generates transformation from twist

3. **Relative transformation**: `T₁⁻¹ · T₂`
   - Transformation from frame 1 to frame 2

### Basis Functions

Cubic B-spline basis functions ensure C² continuity:

```
B₀(u) = (1-u)³ / 6
B₁(u) = (3u³ - 6u² + 4) / 6
B₂(u) = (-3u³ + 3u² + 3u + 1) / 6
B₃(u) = u³ / 6
```

Cumulative form combines these for numerical stability.

## References

1. Sommer, H., et al. "Why and How to Avoid the Flipped Quaternion Multiplication." Aerospace, 2018.

2. Lovegrove, S., et al. "Spline Fusion: A continuous-time representation for visual-inertial fusion with application to rolling shutter cameras." BMVC, 2013.

3. Mueggler, E., et al. "Continuous-Time Visual-Inertial Odometry for Event Cameras." IEEE Transactions on Robotics, 2018.

4. Furgale, P., et al. "Continuous-time batch estimation using temporal basis functions." ICRA, 2012.

## License

This project is provided as-is for research and educational purposes.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Citation

If you use this code in your research, please cite:

```bibtex
@misc{se3-bspline-interpolation,
  author = {Your Name},
  title = {SE(3) Pose Interpolation using B-Spline},
  year = {2024},
  publisher = {GitHub},
  url = {https://github.com/yourusername/SE3-pose-interpolation-using-bspline}
}
```

## Contact

For questions or issues, please open an issue on GitHub.

---

**English Version** | **[中文版](README_CN.md)**
