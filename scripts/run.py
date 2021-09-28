#!/usr/bin/env python3.8

import argparse
import json

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument('--mode', required=True, type=str, help="Run the script as publisher or as a listener")
args, _ = arg_parser.parse_known_args()

if args.mode == "publisher":
    import publisher
    run_func = publisher.run_publisher
    cfg_file = "/tmp/scripts/publisher/publisher_cfg.json"


elif args.mode == "listener":
    import listener
    run_func = listener.run_listener
    cfg_file = "/tmp/scripts/listener/listener_cfg.json"
else:
    raise ValueError(f"Mode argument is {args.mode} and is not supported.")

if __name__ == "__main__":
    with open(cfg_file, 'r') as file:
        cfg = json.load(file)

    run_func(cfg)
