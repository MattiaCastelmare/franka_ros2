#!/usr/bin/env python3

import argparse
import os

import cv2
import numpy as np

import rclpy
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, CompressedImage


def image_msg_to_cv2(msg):
    encoding = msg.encoding.lower()

    if encoding in ["rgb8", "bgr8"]:
        img = np.frombuffer(msg.data, dtype=np.uint8)
        img = img.reshape((msg.height, msg.width, 3))

        if encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        return img

    if encoding in ["mono8", "8uc1"]:
        img = np.frombuffer(msg.data, dtype=np.uint8)
        img = img.reshape((msg.height, msg.width))
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    if encoding in ["16uc1", "mono16"]:
        img = np.frombuffer(msg.data, dtype=np.uint16)
        img = img.reshape((msg.height, msg.width))
        img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
        img = img.astype(np.uint8)
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    if encoding in ["32fc1"]:
        img = np.frombuffer(msg.data, dtype=np.float32)
        img = img.reshape((msg.height, msg.width))
        img = np.nan_to_num(img, nan=0.0, posinf=0.0, neginf=0.0)
        img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
        img = img.astype(np.uint8)
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    raise RuntimeError(f"Encoding not supported: {msg.encoding}")

def get_latest_bag(base_dir="experiment_bags"):
    """Search the latest rosbag saved based on alphabetical order (date format: YYYY-MM-DD_HH-MM-SS)"""
    if not os.path.isdir(base_dir):
        return None
    # Find all subdirectories in experiment_bags
    subdirs = [os.path.join(base_dir, d) for d in os.listdir(base_dir) if os.path.isdir(os.path.join(base_dir, d))]
    if not subdirs:
        return None
    # Sort in alphabetical order and take the last one
    subdirs.sort()
    return subdirs[-1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--bag",
        default="latest",
        help="Directory of the rosbag. If 'latest', takes the latest one from 'experiment_bags/'"
    )
    parser.add_argument(
        "--topic",
        default="/camera/camera/color/image_raw",
        help="Image topic to convert"
    )
    parser.add_argument(
        "--out",
        default="output.mp4",
        help="Name of the output video. If it is only a filename, it will be saved in the rosbag folder"
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=30.0,
        help="FPS of the output video"
    )

    args = parser.parse_args()

    # Logic to find the latest bag automatically
    if args.bag == "latest":
        latest_bag = get_latest_bag("experiment_bags")
        if latest_bag is None:
            print("No bag found in 'experiment_bags/'. Please specify --bag manually.")
            return
        args.bag = latest_bag

    # Absolute path to the bag directory
    bag_dir = os.path.abspath(args.bag)

    # If --out is only a filename, save the video inside the bag directory
    # If --out is an absolute path, use it as is
    if not os.path.isabs(args.out):
        args.out = os.path.join(bag_dir, args.out)

    print(f"Rosbag: {bag_dir}")
    print(f"Topic: {args.topic}")
    print(f"Output video: {args.out}")
    print(f"FPS: {args.fps}")

    rclpy.init()

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_dir, storage_id="sqlite3"),
        ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"
        )
    )

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    if args.topic not in type_map:
        print(f"Topic not found: {args.topic}")
        print("\nAvailable topics:")
        for name, typ in type_map.items():
            print(f"  {name}: {typ}")
        rclpy.shutdown()
        return

    msg_type = get_message(type_map[args.topic])

    writer = None
    frame_count = 0

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic != args.topic:
            continue

        msg = deserialize_message(data, msg_type)

        if isinstance(msg, Image):
            frame = image_msg_to_cv2(msg)

        elif isinstance(msg, CompressedImage):
            arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

        else:
            continue

        if frame is None:
            continue

        if writer is None:
            h, w = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(args.out, fourcc, args.fps, (w, h))

            if not writer.isOpened():
                raise RuntimeError(f"Impossible to create the video: {args.out}")

        writer.write(frame)
        frame_count += 1

    if writer is not None:
        writer.release()
        print(f"\nVideo created: {args.out}")
        print(f"Frame written: {frame_count}")
    else:
        print("\nNo frame found for that topic.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()