

# VL53L1X Scanner Code Documentation

UNDER CONSTRUCTION!

### 1. Adaptive Scan Resolution

Optimizes angle between scans based on the previous single measurement. 

**Pros:**

- Is good for real-time operations (i.e. localization)
- Gives results faster
- Gives less data but with similar information

**Cons:**

- Outputs PointCloud2. Needs resampling for LaserScan message.
- May miss some measurements on edges of an object.

**Math:**

(1) Finding theta. Assuming that two scans will have same measurements. Applying cosines theorem.

![adaptive_res_1](img/adaptive_res_1.png)

(2) In boundaries of the hardwares, `theta = f(dist)` has **reciprocal distribution** features. After removing the effect of this property, applying line fitting to find coefficients. See `/scripts/adaptive_scan_resolution_find_coefficients.py`

![adaptive_res_2](img/adaptive_res_2.png)

(3) Line fitting result of `theta = p(1/dist)`

![adaptive_res_3](img/adaptive_res_3.png)

**Note:**

Actually, I have tried predicting orientation of the wall to guess the direction of next measurement, but it failed. To explain it, firstly, rapid changes in the measurements caused fluctuations in guessed orientation of walls. Secondly, I have solved this issue with applying exponentially weighted averages on to the guess of wall orientation, but this caused overshoot at the end of walls. As a result, I have quit working with orientation predictions and came with this idea which is simpler and faster.



## 2. PointCloud Calculation

- Math calculation for measurement to pointcloud conversion:

![pointcloud_calculation](img/pointcloud_calculation.png)





--------------------------------------------------------------------------

## Notes

- Stepper motor delay set as 2.25ms instead of 2ms, because it was missing steps.
- Stepper motor phase set as 1 instead of 2, because torque was enough.
- (TODO) 28BYJ-48 steps per revolution is 2048 instead of 2038.


- Intensity value is equal to `signal_rate` (This may change in the future)



TODO:

- 3d best quality/speed parameters
- 2d best quality/speed parameters
- 2d select scan ring configuration



