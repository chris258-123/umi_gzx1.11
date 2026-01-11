import time
import enum
import multiprocessing as mp
from multiprocessing.managers import SharedMemoryManager
import zerorpc
from umi.shared_memory.shared_memory_queue import (
    SharedMemoryQueue, Empty)
from umi.shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer
from umi.common.precise_sleep import precise_wait
from umi.common.pose_trajectory_interpolator import PoseTrajectoryInterpolator


class Command(enum.Enum):
    SHUTDOWN = 0
    SCHEDULE_WAYPOINT = 1
    RESTART_PUT = 2


class FrankaGripperController(mp.Process):
    def __init__(self,
            shm_manager: SharedMemoryManager,
            robot_ip: str,
            robot_port: int = 4242,
            frequency: int = 30,
            move_max_speed: float = 0.2,
            move_max_force: float = 70.0,
            get_max_k=None,
            command_queue_size=1024,
            launch_timeout=3,
            receive_latency=0.0,
            verbose=False
            ):
        super().__init__(name="FrankaGripperController")
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.frequency = frequency
        self.move_max_speed = move_max_speed
        self.move_max_force = move_max_force
        self.launch_timeout = launch_timeout
        self.receive_latency = receive_latency
        self.verbose = verbose

        if get_max_k is None:
            get_max_k = int(frequency * 10)

        # build input queue
        example = {
            'cmd': Command.SCHEDULE_WAYPOINT.value,
            'target_pos': 0.0,
            'target_time': 0.0
        }
        input_queue = SharedMemoryQueue.create_from_examples(
            shm_manager=shm_manager,
            examples=example,
            buffer_size=command_queue_size
        )

        # build ring buffer
        example = {
            'gripper_state': 0,
            'gripper_position': 0.0,
            'gripper_velocity': 0.0,
            'gripper_force': 0.0,
            'gripper_measure_timestamp': time.time(),
            'gripper_receive_timestamp': time.time(),
            'gripper_timestamp': time.time()
        }
        ring_buffer = SharedMemoryRingBuffer.create_from_examples(
            shm_manager=shm_manager,
            examples=example,
            get_max_k=get_max_k,
            get_time_budget=0.2,
            put_desired_frequency=frequency
        )

        self.ready_event = mp.Event()
        self.input_queue = input_queue
        self.ring_buffer = ring_buffer

    # ========= launch method ===========
    def start(self, wait=True):
        super().start()
        if wait:
            self.start_wait()
        if self.verbose:
            print(f"[FrankaGripperController] Controller process spawned at {self.pid}")

    def stop(self, wait=True):
        message = {
            'cmd': Command.SHUTDOWN.value
        }
        self.input_queue.put(message)
        if wait:
            self.stop_wait()

    def start_wait(self):
        self.ready_event.wait(self.launch_timeout)
        assert self.is_alive()

    def stop_wait(self):
        self.join()

    @property
    def is_ready(self):
        return self.ready_event.is_set()

    # ========= context manager ===========
    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    # ========= command methods ============
    def schedule_waypoint(self, pos: float, target_time: float):
        message = {
            'cmd': Command.SCHEDULE_WAYPOINT.value,
            'target_pos': pos,
            'target_time': target_time
        }
        self.input_queue.put(message)

    def restart_put(self, start_time):
        self.input_queue.put({
            'cmd': Command.RESTART_PUT.value,
            'target_time': start_time
        })

    # ========= receive APIs =============
    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k,out=out)

    def get_all_state(self):
        return self.ring_buffer.get_all()

    # ========= main loop in process ============
    def run(self):
        # start connection
        try:
            endpoint = f"tcp://{self.robot_ip}:{self.robot_port}"
            client = zerorpc.Client(timeout=10, heartbeat=20)
            client.connect(endpoint)

            if self.verbose:
                print(f"[FrankaGripperController] Connected to {endpoint}")

            # get initial state
            initial_state = client.get_gripper_state()
            if initial_state is None or not isinstance(initial_state, dict):
                if self.verbose:
                    print("[FrankaGripperController] Warning: gripper not available")
                curr_pos = 0.04  # Default to mid-range
            else:
                curr_pos = initial_state.get('width', 0.04)
                if self.verbose:
                    print(f"[FrankaGripperController] Initial gripper width: {curr_pos:.4f}m")

            curr_t = time.monotonic()
            last_waypoint_time = curr_t
            pose_interp = PoseTrajectoryInterpolator(
                times=[curr_t],
                poses=[[curr_pos, 0, 0, 0, 0, 0]]
            )

            keep_running = True
            t_start = time.monotonic()
            iter_idx = 0
            last_target_width = curr_pos

            while keep_running:
                # command gripper
                t_now = time.monotonic()
                dt = 1 / self.frequency
                t_target = t_now
                target_width = pose_interp(t_target)[0]

                # compute velocity from trajectory
                prev_width = pose_interp(t_target - dt)[0]
                target_velocity = abs((target_width - prev_width) / dt)
                target_speed = min(target_velocity, self.move_max_speed)

                # only send command if width changed significantly
                if abs(target_width - last_target_width) > 1e-5:
                    result = client.goto_gripper_width(
                        float(target_width),
                        float(target_speed),
                        float(self.move_max_force),
                        False  # non-blocking
                    )
                    if self.verbose and not result.get('ok', False):
                        print(f"[FrankaGripperController] Warning: goto_gripper_width failed: {result.get('error')}")
                    last_target_width = target_width

                # get state from gripper
                state_dict = client.get_gripper_state()
                if state_dict is not None and isinstance(state_dict, dict):
                    # Map Franka state to WSG-compatible format
                    state = {
                        'gripper_state': 1 if state_dict.get('is_moving', False) else 0,
                        'gripper_position': float(state_dict.get('width', 0.0)),
                        'gripper_velocity': target_velocity,  # Use computed velocity
                        'gripper_force': 0.0,  # Not available from Franka
                        'gripper_measure_timestamp': float(state_dict.get('timestamp', time.time())),
                        'gripper_receive_timestamp': float(state_dict.get('server_receive_timestamp', time.time())),
                        'gripper_timestamp': time.time() - self.receive_latency
                    }
                else:
                    # Fallback state if gripper unavailable
                    state = {
                        'gripper_state': 0,
                        'gripper_position': target_width,
                        'gripper_velocity': 0.0,
                        'gripper_force': 0.0,
                        'gripper_measure_timestamp': time.time(),
                        'gripper_receive_timestamp': time.time(),
                        'gripper_timestamp': time.time() - self.receive_latency
                    }

                self.ring_buffer.put(state)

                # fetch command from queue
                try:
                    commands = self.input_queue.get_all()
                    n_cmd = len(commands['cmd'])
                except Empty:
                    n_cmd = 0

                # execute commands
                for i in range(n_cmd):
                    command = dict()
                    for key, value in commands.items():
                        command[key] = value[i]
                    cmd = command['cmd']

                    if cmd == Command.SHUTDOWN.value:
                        keep_running = False
                        break
                    elif cmd == Command.SCHEDULE_WAYPOINT.value:
                        target_pos = command['target_pos']
                        target_time = command['target_time']
                        # translate global time to monotonic time
                        target_time = time.monotonic() - time.time() + target_time
                        curr_time = t_now
                        pose_interp = pose_interp.schedule_waypoint(
                            pose=[target_pos, 0, 0, 0, 0, 0],
                            time=target_time,
                            max_pos_speed=self.move_max_speed,
                            max_rot_speed=self.move_max_speed,
                            curr_time=curr_time,
                            last_waypoint_time=last_waypoint_time
                        )
                        last_waypoint_time = target_time
                    elif cmd == Command.RESTART_PUT.value:
                        t_start = command['target_time'] - time.time() + time.monotonic()
                        iter_idx = 1
                    else:
                        keep_running = False
                        break

                # first loop successful, ready to receive command
                if iter_idx == 0:
                    self.ready_event.set()
                iter_idx += 1

                # regulate frequency
                dt = 1 / self.frequency
                t_end = t_start + dt * iter_idx
                precise_wait(t_end=t_end, time_func=time.monotonic)

            # cleanup
            client.close()

        except Exception as e:
            if self.verbose:
                print(f"[FrankaGripperController] Error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.ready_event.set()
            if self.verbose:
                print(f"[FrankaGripperController] Disconnected from {self.robot_ip}:{self.robot_port}")
