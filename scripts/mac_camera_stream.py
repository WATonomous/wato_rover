#!/usr/bin/env python3
# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Serves your Mac's built-in camera as JPEG frames over HTTP so a ROS2 node
# in Docker can fetch and publish to /image (e.g. for YOLO + Foxglove).
#
# Usage (on your Mac, no ROS2 required):
#   pip install opencv-python  # if needed
#   python3 scripts/mac_camera_stream.py
#
# Then from Docker, use camera_from_url node with:
#   image_url:=http://host.docker.internal:9999/frame
#
# Default port: 9999. Override with: python3 scripts/mac_camera_stream.py --port 9999

import argparse
import sys
import threading
import time

try:
    import cv2
except ImportError:
    print("Install opencv-python: pip install opencv-python", file=sys.stderr)
    sys.exit(1)

try:
    from http.server import HTTPServer, BaseHTTPRequestHandler
except ImportError:
    from BaseHTTPServer import HTTPServer, BaseHTTPRequestHandler  # Python 2 fallback


class FrameHandler(BaseHTTPRequestHandler):
    """Serves the latest camera frame as JPEG."""

    def do_GET(self):
        if self.path != "/frame" and self.path != "/":
            self.send_error(404)
            return
        if not hasattr(self.server, "latest_jpeg") or self.server.latest_jpeg is None:
            self.send_error(503, "No frame yet")
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(self.server.latest_jpeg)))
        self.end_headers()
        self.wfile.write(self.server.latest_jpeg)

    def log_message(self, format, *args):
        pass


def capture_loop(cap, server, stop_event):
    """Background thread: keep updating server.latest_jpeg."""
    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue
        _, jpeg = cv2.imencode(".jpg", frame)
        server.latest_jpeg = jpeg.tobytes()
        time.sleep(0.033)  # ~30 fps cap


def main():
    parser = argparse.ArgumentParser(description="Stream Mac camera as HTTP JPEG for ROS2.")
    parser.add_argument("--port", type=int, default=9999, help="HTTP port (default 9999)")
    parser.add_argument("--device", type=int, default=0, help="OpenCV camera device index (default 0)")
    parser.add_argument("--width", type=int, default=640, help="Frame width")
    parser.add_argument("--height", type=int, default=480, help="Frame height")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.device)
    if not cap.isOpened():
        print("Could not open camera. Try --device 0 or 1.", file=sys.stderr)
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    class Server(HTTPServer):
        latest_jpeg = None

    server = Server(("", args.port), FrameHandler)
    stop_event = threading.Event()
    thread = threading.Thread(target=capture_loop, args=(cap, server, stop_event), daemon=True)
    thread.start()

    print(f"Mac camera stream: http://localhost:{args.port}/frame")
    print("From Docker use: http://host.docker.internal:{}/frame".format(args.port))
    print("Press Ctrl+C to stop.")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        cap.release()
        server.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()
