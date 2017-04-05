## Extended Kalman Filter Project Starter Code. [Rubric](https://review.udacity.com/#!/rubrics/748/view) Points
Self-Driving Car Engineer Nanodegree Program

---
### 1. Accuracy for data-1

> The px, py, vx, vy output coordinates have an RMSE <= [0.08, 0.08, 0.60, 0.60] when using the file: "sample-laser-radar-measurement-data-1.txt".

I got

| My Accuracy | Target Accuracy|
|:-------------:|:-------------:|
| 0.0651648     | 0.08 |
| 0.0605379     | 0.08 |
| 0.533212      | 0.60 |
| 0.544193      | 0.60 |

---
### 2. Accuracy for data-2

> The px, py, vx, vy output coordinates have an RMSE <= [0.20, 0.20, .50, .85] when using the file: "sample-laser-radar-measurement-data-2.txt".

I got

| My Accuracy | Target Accuracy|
|:-------------:|:-------------:|
| 0.185495      | 0.20 |
| 0.190302      | 0.20 |
| 0.487137      | 0.50 |
| 0.810657      | 0.85 |

---
### 3. Algorithm-1

> Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.

I followed the flow taught in the lesson. Most building blocks, including Predict(), UpdateEKF(), and Update(), are actually the codes in the lecture.


---
### 4. Algorithm-2

> Your Kalman Filter algorithm handles the first measurements appropriately.

I handled the first measurements as follows:

1. RADAR:
``` python
ekf_.x_ << cos(theta)*rho, sin(theta)*rho, cos(theta)*measurement_pack.raw_measurements_(2), sin(theta)*measurement_pack.raw_measurements_(2);
```

2. LASER
``` python
ekf_.x_ << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), 1, 1;
```

Before calling those assiangment, a checking on nonsence sensor data is done by:
``` python
// Avoid initializing nonsence data
if (ekf_.x_[0] < MIN_SENSOR_VALUE && ekf_.x_[1] < MIN_SENSOR_VALUE)
    return;
```

---
### 5. Algorithm-3

> Your Kalman Filter algorithm first predicts then updates.

Sure it is!
``` c++
  // If two measurements are closed to each other, then don't predict
  if ( dt > 0.001 )
  {
      ekf_.Predict();
  }
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
      ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
      ekf_.R_ = R_radar_;
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
      ekf_.H_ = H_laser_;
      ekf_.R_ = R_laser_;
      ekf_.Update(measurement_pack.raw_measurements_);
  }
```

---
### 6. Algorithm-4

> Your Kalman Filter can handle radar and lidar measurements.

Sure it is! As shown in the above code snippet, I update with `UpdateEKF()` and `Update()` according to what type of sensor is.

---
### 7. Code Efficiency

> Your algorithm should avoid unnecessary calculations.

1. I avoid running the same calculation in `Q_`, `Update()`, and `UpdateEKF()`.
2. No complex data structures are used. In fact, the template codes provided are well structured and most variables we need are defined already.
3. No Unnecessary control flow checks. Only two parts of checking and both are necessary:
  * `Update()`: check if 0 to avoid `sqrt(0)` and divid 0. Code: `if (px==0 && py==0){return;}`
  * `CalculateJacobian()`: check to avoid divid 0
  ```c++
  if(fabs(c1) < 0.0001){
          std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
          c1 = 0.0001;
  //        return Hj;
      }
  ```
