import argparse
import time

import zerorpc


def _print_state(state, prefix: str = ""):
	if state is None:
		print(f"{prefix}gripper_state=None")
		return
	keys = [
		"timestamp",
		"server_receive_timestamp",
		"width",
		"max_width",
		"is_moving",
		"is_grasped",
		"prev_command_successful",
		"prev_command_id",
	]
	summary = {k: state.get(k, None) for k in keys}
	print(f"{prefix}{summary}")


def main():
	parser = argparse.ArgumentParser(description="Test Franka gripper via ZeroRPC FrankaInterface server")
	parser.add_argument("--nuc_ip", type=str, required=True, help="NUC IP running launch_franka_interface_server.py")
	parser.add_argument("--rpc_port", type=int, default=4242, help="ZeroRPC port on NUC")
	parser.add_argument("--open_width", type=float, default=None, help="Open width (m). If omitted, uses max_width or 0.08")
	parser.add_argument("--close_width", type=float, default=0.0, help="Close width (m)")
	parser.add_argument("--speed", type=float, default=0.08, help="Gripper speed (m/s)")
	parser.add_argument("--force", type=float, default=20.0, help="Max gripper force (N)")
	parser.add_argument("--sleep", type=float, default=1.5, help="Seconds to wait between commands")
	parser.add_argument("--blocking", action="store_true", help="Use blocking=True when sending gRPC command")
	args = parser.parse_args()

	endpoint = f"tcp://{args.nuc_ip}:{args.rpc_port}"
	client = zerorpc.Client(timeout=10)
	client.connect(endpoint)
	print(f"Connected to {endpoint}")

	state0 = client.get_gripper_state()
	_print_state(state0, prefix="initial: ")

	max_width = None
	if isinstance(state0, dict):
		max_width = state0.get("max_width", None)

	open_width = args.open_width
	if open_width is None:
		open_width = float(max_width) if max_width is not None else 0.08

	# 1) Open
	print(f"goto open: width={open_width} speed={args.speed} force={args.force}")
	print(client.goto_gripper_width(open_width, args.speed, args.force, args.blocking))
	time.sleep(args.sleep)
	_print_state(client.get_gripper_state(), prefix="after open: ")

	# 2) Close
	print(f"goto close: width={args.close_width} speed={args.speed} force={args.force}")
	print(client.goto_gripper_width(args.close_width, args.speed, args.force, args.blocking))
	time.sleep(args.sleep)
	_print_state(client.get_gripper_state(), prefix="after close: ")

	# 3) Open again
	print(f"goto open2: width={open_width} speed={args.speed} force={args.force}")
	print(client.goto_gripper_width(open_width, args.speed, args.force, args.blocking))
	time.sleep(args.sleep)
	_print_state(client.get_gripper_state(), prefix="after open2: ")


if __name__ == "__main__":
	main()

