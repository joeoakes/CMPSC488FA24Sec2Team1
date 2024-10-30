import argparse, subprocess

def start(args):
    if args.mode == "battle":
        print("Starting in battle mode...")
        # Start systemd services with bot in battle mode
    elif args.mode == "disruptor":
        print("Starting in disruptor mode...")
        # Start systemd services with bot in disruptor mode
    else:
        print("Incorrect mode")
        return
    print("Bot started")

def stop(args):
    print("Stopping bot...")
    print("Bot stopped")

def logs(args):
    print("Bot started in battle mode")
    print("Entering attack mode")
    print("Searching for targets")
    print("Obstacle found, saving image at ./debug/images/obstacles/1.jpg")
    print("Navigating around obstacle")
    print("Disruptor spotted, saving image at ./debug/images/disruptor/1.jpg")
    print("Entering deffense mode")
    print("Disruptor gone, switching back to attack mode")
    print("Entering attack mode")
    print("Hit by disruptor, pausing for 3 seconds")
    print("Resuming")
    print("Target spotted, saving image at ./debug/images/targets/1.jpg")
    print("Aiming at target")
    print("Firing laser")
    print("Point verified")
    print("Stop command received, stopping")
    print()
    print("Bot started in disruptor mode")
    print("Entering disruptor mode")
    print("Searching for robots")
    print("Obstacle found, saving image at ./debug/images/obstacles/2.jpg")
    print("Navigating around obstacle")
    print("Robot spotted, saving image at ./debug/images/robot/1.jpg")
    print("Aiming at robot's target")
    print("Firing")
    print("Stop command received, stopping")


def check(args):
    print("Running systems check")
    subprocess.run(["./pi-check.sh"], cwd="./pi-check")

# parser = argparse.ArgumentParser(
#     prog="LaserTagBot",
#     description="Main controller for Laser Tag Robot")

# parser.add_argument("action", choices=["start", "stop", "logs", "check"])
# subparsers = parser.add_subparsers(description="command", required=True)
parser = argparse.ArgumentParser(description="Bot control program")
subparsers = parser.add_subparsers(dest="command", help="Available commands")
# start_parser = subparsers.add_parser("start", help="Start bot in either battle or disruptor mode")
# start_parser.add_argument("mode", choices=["battle", "disruptor"])
# start_parser.set_defaults(func=start)
#
# stop_parser = subparsers.add_parser("stop", help="Stop bot")
# stop_parser.set_defaults(func=stop)
#
# logs_parser = subparsers.add_parser("logs", help="View Logs")
# logs_parser.set_defaults(func=logs)
#
# check_parser = subparsers.add_parser("check", help="Run System Check")
# check_parser.set_defaults(func=check)
#
# parser.parse_args()

# Start command
start_parser = subparsers.add_parser("start", help="Start the bot")
start_parser.add_argument("mode", choices=["battle", "disruptor"], help="Mode to start in")

# Stop command
subparsers.add_parser("stop", help="Stop the bot")

# Logs command
subparsers.add_parser("logs", help="Print logs")

# Check command
subparsers.add_parser("check", help="Run systems check")

args = parser.parse_args()

if args.command == "start":
    start(args)
elif args.command == "stop":
    stop(args)
elif args.command == "logs":
    logs(args)
elif args.command == "check":
    check(args)
else:
    parser.print_help()
