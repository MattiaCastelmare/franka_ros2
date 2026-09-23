#!/usr/bin/env python3

import argparse
import os
import subprocess
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage


def image_msg_to_cv2(msg):
    encoding = msg.encoding.lower()

    if encoding in ("rgb8", "bgr8"):
        img = np.frombuffer(msg.data, np.uint8).reshape(
            msg.height, msg.width, 3
        )
        if encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img

    if encoding in ("mono8", "8uc1"):
        img = np.frombuffer(msg.data, np.uint8).reshape(
            msg.height, msg.width
        )
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    if encoding in ("16uc1", "mono16"):
        img = np.frombuffer(msg.data, np.uint16).reshape(
            msg.height, msg.width
        )
        img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
        return cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_GRAY2BGR)

    if encoding == "32fc1":
        img = np.frombuffer(msg.data, np.float32).reshape(
            msg.height, msg.width
        )
        img = np.nan_to_num(img)
        img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
        return cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_GRAY2BGR)

    raise RuntimeError(f"Encoding non supportato: {msg.encoding}")


class LiveRecorder(Node):
    def __init__(self, topic):
        super().__init__("hand_debug_video_recorder")

        out_dir = Path(__file__).resolve().parents[1] / "results" / "videos"
        out_dir.mkdir(parents=True, exist_ok=True)

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.avi = out_dir / f"{stamp}.avi"
        self.mp4 = out_dir / f"{stamp}.mp4"

        self.writer = None
        self.frames = []
        self.times = []

        self.create_subscription(Image, topic, self.callback, 10)

    def callback(self, msg):
        frame = image_msg_to_cv2(msg)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.writer is None:
            self.frames.append(frame.copy())
            self.times.append(t)

            if len(self.frames) >= 10:
                self.open_writer()
            return

        self.writer.write(frame)

    def open_writer(self):
        if self.writer is not None or not self.frames:
            return

        dt = np.diff(self.times)
        dt = dt[dt > 0]
        fps = 1.0 / np.median(dt) if len(dt) else 30.0

        h, w = self.frames[0].shape[:2]
        self.writer = cv2.VideoWriter(
            str(self.avi),
            cv2.VideoWriter_fourcc(*"MJPG"),
            float(fps),
            (w, h),
        )

        if not self.writer.isOpened():
            raise RuntimeError(f"Impossibile creare {self.avi}")

        for frame in self.frames:
            self.writer.write(frame)

        self.frames.clear()
        self.times.clear()

        self.get_logger().info(f"Video recording: {fps:.2f} FPS")

    def close(self):
        if self.writer is None and self.frames:
            self.open_writer()

        if self.writer is None:
            return

        self.writer.release()

        try:
            subprocess.run([
                "ffmpeg", "-y", "-loglevel", "error",
                "-i", str(self.avi),
                "-c:v", "libx264",
                "-crf", "18",
                "-pix_fmt", "yuv420p",
                "-movflags", "+faststart",
                str(self.mp4),
            ], check=True)

            self.avi.unlink(missing_ok=True)
            print(f"\nVideo creato: {self.mp4}")

        except Exception as e:
            print(f"\nConversione MP4 fallita: {e}")
            print(f"AVI mantenuto: {self.avi}")


def live(topic):
    rclpy.init()
    node = LiveRecorder(topic)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def bag_to_video(args):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rosidl_runtime_py.utilities import get_message
    from rclpy.serialization import deserialize_message

    bag_dir = os.path.abspath(args.bag)

    if not os.path.isabs(args.out):
        args.out = os.path.join(bag_dir, args.out)

    rclpy.init()

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_dir, storage_id="sqlite3"),
        ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    type_map = {
        t.name: t.type
        for t in reader.get_all_topics_and_types()
    }

    if args.topic not in type_map:
        raise RuntimeError(f"Topic non trovato: {args.topic}")

    msg_type = get_message(type_map[args.topic])
    writer = None
    count = 0

    while reader.has_next():
        topic, data, _ = reader.read_next()

        if topic != args.topic:
            continue

        msg = deserialize_message(data, msg_type)

        if isinstance(msg, Image):
            frame = image_msg_to_cv2(msg)
        elif isinstance(msg, CompressedImage):
            frame = cv2.imdecode(
                np.frombuffer(msg.data, np.uint8),
                cv2.IMREAD_COLOR,
            )
        else:
            continue

        if writer is None:
            h, w = frame.shape[:2]
            writer = cv2.VideoWriter(
                args.out,
                cv2.VideoWriter_fourcc(*"mp4v"),
                args.fps,
                (w, h),
            )

        writer.write(frame)
        count += 1

    if writer is not None:
        writer.release()
        print(f"Creato video: {args.out}")
        print(f"Frame scritti: {count}")

    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--live", action="store_true")
    parser.add_argument("--bag", default=".")
    parser.add_argument(
        "--topic",
        default="/handover/hand_debug_image",
    )
    parser.add_argument("--out", default="output.mp4")
    parser.add_argument("--fps", type=float, default=30.0)

    args = parser.parse_args()

    if args.live:
        live(args.topic)
    else:
        bag_to_video(args)


if __name__ == "__main__":
    main()
