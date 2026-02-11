#!/usr/bin/env python3
# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Proxies your phone's camera stream (e.g. from IP Webcam app) to the same
# HTTP /frame interface used by the robot container, so you can run YOLO
# object detection on the phone camera.
#
# Usage (on your Mac, no ROS2 required):
#   1. On your phone: install "IP Webcam" (Android) or similar and start the server.
#   2. Find your phone's IP (e.g. in the app) and the snapshot URL (e.g. http://192.168.1.5:8080/shot.jpg).
#   3. On your Mac:
#      pip install requests   # if needed
#      python3 scripts/phone_camera_stream.py --url http://YOUR_PHONE_IP:8080/shot.jpg
#
# Then use the same Docker + launch steps as for the Mac camera; the robot
# uses http://host.docker.internal:9999/frame.

import argparse
import sys
import threading
import time

try:
    import urllib.request
except ImportError:
    print("Use Python 3.", file=sys.stderr)
    sys.exit(1)

try:
    from http.server import HTTPServer, BaseHTTPRequestHandler
except ImportError:
    from BaseHTTPServer import HTTPServer, BaseHTTPRequestHandler


class FrameHandler(BaseHTTPRequestHandler):
    """Serves the latest fetched frame as JPEG."""

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


def fetch_loop(image_url, server, stop_event, timeout_sec=3.0, interval_sec=0.033):
    """Background thread: fetch from phone URL and update server.latest_jpeg."""
    while not stop_event.is_set():
        try:
            req = urllib.request.Request(image_url)
            with urllib.request.urlopen(req, timeout=timeout_sec) as resp:
                data = resp.read()
            if data:
                server.latest_jpeg = data
        except Exception:
            pass
        time.sleep(interval_sec)


def main():
    parser = argparse.ArgumentParser(
        description="Proxy phone camera (IP Webcam) as HTTP JPEG for ROS2 object detection."
    )
    parser.add_argument(
        "--url",
        type=str,
        required=True,
        help="Phone snapshot URL (e.g. http://192.168.1.5:8080/shot.jpg from IP Webcam app)",
    )
    parser.add_argument("--port", type=int, default=9999, help="HTTP port to serve on (default 9999)")
    parser.add_argument(
        "--interval",
        type=float,
        default=0.033,
        help="Seconds between fetches (~30 fps default)",
    )
    parser.add_argument("--timeout", type=float, default=3.0, help="Request timeout in seconds")
    args = parser.parse_args()

    class Server(HTTPServer):
        latest_jpeg = None

    server = Server(("", args.port), FrameHandler)
    stop_event = threading.Event()
    thread = threading.Thread(
        target=fetch_loop,
        args=(args.url, server, stop_event, args.timeout, args.interval),
        daemon=True,
    )
    thread.start()

    print(f"Phone camera proxy: fetching {args.url}")
    print(f"Serving at: http://localhost:{args.port}/frame")
    print("From Docker use: http://host.docker.internal:{}/frame".format(args.port))
    print("Press Ctrl+C to stop.")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        server.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()
