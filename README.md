# Composite CBF for Collision Avoidance

This repository contains the ROS implementation of the **Composite CBF Safety Filter for Collision Avoidance for Multirotors**.  
It computes the optimally minimal modifications to an acceleration setpoint that prevent collisions based on velocity estimates and depth measurements.

See our [paper video on YouTube!](https://www.youtube.com/watch?v=8nnfFECLjxw)

The filter operates using real-time onboard range measurements and velocity estimation.
Specifically, the filter operates on a downsampled point cloud from the onboard sensor mounted on the robot, including, e.g., Intel RealSense D455 and alike, small-scale Time-of-Flight sensors, or LiDARs.
We release a point cloud selector node for generic depth / range sensors on [this dedicated repo](https://github.com/ntnu-arl/composite_cbf).

The node implements the unconstrained-FoV case.
It relies on computing an analytical solution to the underlying CBF-QP, which is computationally efficient.

We also release an [embedded implementation for PX4](https://github.com/ntnu-arl/PX4-CBF).

## Installation

> Note: A ROS2 implementation will be released soon. The current ROS1 version will remain as a separate branch.

### Requirements

* ROS Noetic
* Eigen3

### Installation

* Clone the repo in the `src` folder of your workspace and install it using Catkin.
* Install the [point cloud selector node](https://github.com/ntnu-arl/composite_cbf).

## Config

The CBF tuning is presented in [our paper, Section V.E](https://arxiv.org/html/2504.15850v1#S5).

We report here the table detailing the config files:

| Parameter     | Range     | Effect when increasing        |
| ------------- | --------- | ----------------------------- |
| `epsilon`     | >0        | Larger avoidance radius       |
| `kappa`       | [10, 100] | Less smooth approximation     |
| `gamma`       | [10, 100] | Reacts to farther obstacles   |
| `alpha`       | [1, 3]    | Increased filter sensitivity  |
| `pole_0`      | [-3, -1]  | Damped response               |
| `lp_gain_out` | [0, 1]    | Lesser accel. smoothing       |
| `clamp_xy`    | >0        | Lateral accel. clamping       |
| `clamp_z`     | >0        | Vertical accel. clamping      |
| `obs_to`      | >0        | Timeout [s] for old obstacles |
| `cmd_to`      | >0        | Timeout [s] for old reference |


## Cite

If you use this work in your research, please cite the following publication:

```bibtex
@INPROCEEDINGS{harms2025safe,
  AUTHOR={Marvin Harms and Martin Jacquet and Kostas Alexis},
  TITLE={Safe Quadrotor Navigation using Composite Control Barrier Functions},
  BOOKTITLE={2025 IEEE International Conference on Robotics and Automation (ICRA)},
  YEAR={2025},
  URL={https://arxiv.org/abs/2502.04101},
}
```

Or, if you use our embedded implementation, please cite:

```bibtex
@INPROCEEDINGS{misyats2025embedded,
  AUTHOR={Misyats, Nazar and Harms, Marvin and Nissov, Morten and Jacquet, Martin and Alexis, Kostas},
  TITLE={Embedded Safe Reactive Navigation for Multirotors Systems using Control Barrier Functions},
  BOOKTITLE={2025 International Conference on Unmanned Aircraft Systems (ICUAS)},
  pages={697--704},
  YEAR={2025},
  URL={https://arxiv.org/abs/2504.15850},
}
```

## Acknowledgements

This work was supported by the European Commission Horizon Europe grants DIGIFOREST (EC 101070405) and SPEAR (EC 101119774).

## Contact

* [Martin Jacquet](mailto:marvin.jacquet@ntnu.no)
* [Marvin Harms](mailto:marvin.c.harms@ntnu.no)
* [Kostas Alexis](mailto:konstantinos.alexis@ntnu.no)
