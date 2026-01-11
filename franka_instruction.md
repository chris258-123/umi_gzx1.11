# Deploy UMI Policy on Franka Robot

## System Overview
<img width="90%" src="assets/umi_franka.png">

* FrankaInterface (Server): [scripts_real/launch_franka_interface_server.py](scripts_real/launch_franka_interface_server.py) (L7)

* FrankaInterface (Client): [umi/real_world/franka_interpolation_controller.py](umi/real_world/franka_interpolation_controller.py) (L36)

* FrankaInterpolationController: [umi/real_world/franka_interpolation_controller.py](umi/real_world/franka_interpolation_controller.py) (L71)


## Instructions
* Follow the [documentation](https://facebookresearch.github.io/fairo/polymetis/prereq.html#franka-panda-hardware-setup) to install *Polymetis* on a computer with realtime kernel (i.e., NUC in our example).
* Launch FrankaInterface Server on NUC.
    
    `python scripts_real/launch_franka_interface_server.py`
* (optional) Now you should be able to control the Franka arm using a space mouse on another desktop, the one you are going to run the robot policy on.

    `python scripts_real/control_franka.py`
* Change [eval_robots_config.yaml](example/eval_robots_config.yaml)/robots/robot_type to `'franka'`. This will change the robot controller in [umi_env](umi/real_world/bimanual_umi_env.py) (L233).

* Obtain IP address and update [eval_robots_config.yaml](example/eval_robots_config.yaml)/robots/robot_ip.
* On the Franka interface website
    * Set mass to 1.8 kg
    * Set Flange to Center of Mass of Load Vector to (0.064, -0.06, 0.03)m.

## Gripper Configuration

### Using Franka Native Gripper (Franka Hand)

The system automatically uses Franka's native gripper when `robot_type: 'franka'` is set. The gripper is controlled through the same Polymetis server running on the NUC.

**Configuration** (`example/eval_robots_config.yaml`):
```yaml
grippers: [
  {
    gripper_type: "franka",  # Optional, auto-detected from robot_type
    gripper_obs_latency: 0.01,
    gripper_action_latency: 0.1,
    gripper_max_speed: 0.2,   # Max speed in m/s (Franka Hand: 0.2 m/s)
    gripper_max_force: 70.0   # Max force in N (Franka Hand: 70 N)
  }
]
```

**No additional hardware required** - gripper control is integrated with robot control through Polymetis.

**Testing the gripper**:
```bash
# On desktop (replace with your NUC IP)
python scripts_real/test_franka_gripper.py --nuc_ip 172.16.13.130
```

### Using WSG50 Gripper (Alternative)

If using an external WSG50 gripper with Franka robot:
```yaml
grippers: [
  {
    gripper_type: "wsg",
    gripper_ip: "192.168.0.18",
    gripper_port: 1000,
    gripper_obs_latency: 0.01,
    gripper_action_latency: 0.1
  }
]
```

Requires separate WSG50 gripper hardware and network connection.

## Deployment

* Then you should be able to launch the evaluation on the franka arm.

    `python eval_real.py --robot_config=example/eval_robots_config.yaml -i cup_wild_vit_l.ckpt -o data/eval_cup_wild_example`