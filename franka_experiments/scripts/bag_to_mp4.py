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

    raise RuntimeError(f"Encoding non supportato: {msg.encoding}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--bag",
        default=".",
        help="Cartella della rosbag, non il file .db3"
    )
    parser.add_argument(
        "--topic",
        required=True,
        help="Topic immagine da convertire"
    )
    parser.add_argument(
        "--out",
        default="output.mp4",
        help="Nome video output. Se è solo un nome file, viene salvato nella cartella della rosbag"
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=30.0,
        help="FPS del video output"
    )

    args = parser.parse_args()

    # Cartella assoluta della rosbag
    bag_dir = os.path.abspath(args.bag)

    # Se --out è solo un nome file, salva il video dentro la cartella della rosbag
    # Se invece --out è un percorso assoluto, lo usa così com'è
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
        print(f"Topic non trovato: {args.topic}")
        print("\nTopic disponibili:")
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
                raise RuntimeError(f"Impossibile creare il video: {args.out}")

        writer.write(frame)
        frame_count += 1

    if writer is not None:
        writer.release()
        print(f"\nCreato video: {args.out}")
        print(f"Frame scritti: {frame_count}")
    else:
        print("\nNessun frame trovato per quel topic.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()