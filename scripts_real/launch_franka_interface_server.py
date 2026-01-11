import zerorpc
from polymetis import RobotInterface, GripperInterface
import scipy.spatial.transform as st
import numpy as np
import torch
import threading
import time
import argparse

class FrankaInterface:
    def __init__(
        self,
        host='localhost',
        enable_gripper: bool = True,
        gripper_ip: str = '127.0.0.1',
        gripper_port: int = 50052,
    ):
        self._gripper_lock = threading.Lock()

        self.robot = RobotInterface(host)
        # 注释掉夹爪功能
        self.gripper = None

        if enable_gripper:
            try:
                self.gripper = GripperInterface(ip_address=gripper_ip, port=int(gripper_port))
            except Exception as e:
                print(f'[FrankaInterface] Warning: failed to initialize gripper interface: {e}')

        self._print_initial_state()
        # 注释掉初始夹爪状态打印
        self._print_initial_gripper_state()

    def _print_initial_state(self):
        try:
            joint_positions = self.robot.get_joint_positions().numpy().tolist()
            joint_velocities = self.robot.get_joint_velocities().numpy().tolist()

            ee = self.robot.get_ee_pose()
            pos = ee[0].numpy()
            quat_xyzw = ee[1].numpy()
            rot_vec = st.Rotation.from_quat(quat_xyzw).as_rotvec()
            ee_pose = np.concatenate([pos, rot_vec]).tolist()

            print('[FrankaInterface] Initial state:')
            print({
                'joint_positions': joint_positions,
                'joint_velocities': joint_velocities,
                'ee_pose': ee_pose,
            })
        except Exception as e:
            print(f'[FrankaInterface] Warning: failed to read initial state: {e}')

    @staticmethod
    def _timestamp_to_unix_seconds(ts) -> float:
        if ts is None:
            return float('nan')
        # protobuf Timestamp-like object (seconds/nanos)
        secs = getattr(ts, 'seconds', None)
        nanos = getattr(ts, 'nanos', None)
        if secs is None:
            return float('nan')
        try:
            return float(secs) + float(nanos or 0) * 1e-9
        except Exception:
            return float('nan')

    # 注释掉夹爪初始状态打印函数
    def _print_initial_gripper_state(self):
        if self.gripper is None:
            print('[FrankaInterface] Gripper: disabled or unavailable')
            return
        try:
            state = self.get_gripper_state()
            print('[FrankaInterface] Initial gripper state:')
            print(state)
        except Exception as e:
            print(f'[FrankaInterface] Warning: failed to read initial gripper state: {e}')

    def get_ee_pose(self):
        data = self.robot.get_ee_pose()
        pos = data[0].numpy()
        quat_xyzw = data[1].numpy()
        rot_vec = st.Rotation.from_quat(quat_xyzw).as_rotvec()
        return np.concatenate([pos, rot_vec]).tolist()
    
    def get_joint_positions(self):
        return self.robot.get_joint_positions().numpy().tolist()
    
    def get_joint_velocities(self):
        return self.robot.get_joint_velocities().numpy().tolist()
    
    def move_to_joint_positions(self, positions, time_to_go):
        self.robot.move_to_joint_positions(
            positions=torch.Tensor(positions),
            time_to_go=time_to_go,
        )
    
    def start_cartesian_impedance(self, Kx, Kxd):
        self.robot.start_cartesian_impedance(
            Kx=torch.Tensor(Kx),
            Kxd=torch.Tensor(Kxd),
        )

    def update_desired_ee_pose(self, pose):
        pose = np.asarray(pose)
        self.robot.update_desired_ee_pose(
            position=torch.Tensor(pose[:3]),
            orientation=torch.Tensor(st.Rotation.from_rotvec(pose[3:]).as_quat()),
        )

    def terminate_current_policy(self):
        self.robot.terminate_current_policy()

    # ---------------- Gripper (Franka Hand via Polymetis GripperInterface) ----------------

    def get_gripper_state(self):
        """Returns a JSON-serializable dict of gripper state, or None if unavailable."""
        if self.gripper is None:
            return None

        with self._gripper_lock:
            state = self.gripper.get_state()

        # `state.timestamp` is a protobuf Timestamp.
        state_ts = getattr(state, 'timestamp', None)

        # Metadata may be missing if server doesn't provide it.
        max_width = None
        metadata = getattr(self.gripper, 'metadata', None)
        if metadata is not None:
            max_width = getattr(metadata, 'max_width', None)

        return {
            'timestamp': self._timestamp_to_unix_seconds(state_ts),
            'server_receive_timestamp': time.time(),
            'width': float(getattr(state, 'width', float('nan'))),
            'max_width': None if max_width is None else float(max_width),
            'is_grasped': bool(getattr(state, 'is_grasped', False)),
            'is_moving': bool(getattr(state, 'is_moving', False)),
            'prev_command_successful': bool(getattr(state, 'prev_command_successful', False)),
            'prev_command_id': int(getattr(state, 'prev_command_id', 0)),
        }

    def get_gripper_width(self):
        state = self.get_gripper_state()
        if state is None:
            return None
        return state.get('width', None)

    def goto_gripper_width(self, width, speed, force, blocking=False):
        """Commands the gripper to a target width (meters).

        Args:
            width: target opening width (m)
            speed: speed (m/s)
            force: max force (N)
            blocking: whether to block until command has been sent via gRPC client queue
        """
        if self.gripper is None:
            return {'ok': False, 'error': 'gripper_unavailable'}

        with self._gripper_lock:
            try:
                self.gripper.goto(
                    width=float(width),
                    speed=float(speed),
                    force=float(force),
                    blocking=bool(blocking),
                )
                return {'ok': True, 'error': None}
            except Exception as e:
                return {'ok': False, 'error': str(e)}

    def grasp_gripper(self, speed, force, grasp_width=0.0, epsilon_inner=-1.0, epsilon_outer=-1.0, blocking=False):
        """Grasp command (useful for Franka Hand force-hold semantics)."""
        if self.gripper is None:
            return {'ok': False, 'error': 'gripper_unavailable'}

        with self._gripper_lock:
            try:
                self.gripper.grasp(
                    speed=float(speed),
                    force=float(force),
                    grasp_width=float(grasp_width),
                    epsilon_inner=float(epsilon_inner),
                    epsilon_outer=float(epsilon_outer),
                    blocking=bool(blocking),
                )
                return {'ok': True, 'error': None}
            except Exception as e:
                return {'ok': False, 'error': str(e)}

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Launch Franka Interface Server')
    parser.add_argument('--host', type=str, default='localhost',
                       help='Polymetis server host (default: localhost)')
    parser.add_argument('--enable-gripper', action='store_true', default=True,
                       help='Enable gripper interface (default: True)')
    parser.add_argument('--no-gripper', dest='enable_gripper', action='store_false',
                       help='Disable gripper interface')
    parser.add_argument('--gripper-ip', type=str, default='127.0.0.1',
                       help='Gripper server IP (default: 127.0.0.1)')
    parser.add_argument('--gripper-port', type=int, default=50052,
                       help='Gripper server port (default: 50052)')
    parser.add_argument('--bind-address', type=str, default='0.0.0.0',
                       help='zerorpc bind address (default: 0.0.0.0)')
    parser.add_argument('--bind-port', type=int, default=4242,
                       help='zerorpc bind port (default: 4242)')
    args = parser.parse_args()

    interface = FrankaInterface(
        host=args.host,
        enable_gripper=args.enable_gripper,
        gripper_ip=args.gripper_ip,
        gripper_port=args.gripper_port
    )

    s = zerorpc.Server(interface)
    bind_addr = f"tcp://{args.bind_address}:{args.bind_port}"
    s.bind(bind_addr)
    print(f"[FrankaInterface] Server listening on {bind_addr}")
    s.run()