# Testing object detection with your phone camera

Use your phone as the camera for YOLO object detection and view the feed in Foxglove. Your phone and Mac must be on the same WiFi.

## 1. Stream your phone camera (on the host)

### On your phone

1. Install an IP Webcam app:
   - **Android:** [IP Webcam](https://play.google.com/store/apps/details?id=com.pas.webcam) (or similar).
   - **iOS:** Use an app that exposes a snapshot URL over HTTP (e.g. search “IP webcam” or “camera server”).
2. Start the server in the app (often “Start server”).
3. Note the URL shown in the app, e.g. `http://192.168.1.5:8080/shot.jpg`. The IP is your phone’s IP on the WiFi network.

### On your Mac

Run the proxy script so the robot container can use the stream:

```bash
python3 scripts/phone_camera_stream.py --url http://YOUR_PHONE_IP:8080/shot.jpg
```

Replace `YOUR_PHONE_IP` (and port/path if your app uses something else). Leave this terminal running.

## 2. Start Docker (robot + Foxglove)

Optional: to skip Gazebo for this test, set in `watod-config.sh`:

```bash
export ACTIVE_MODULES="robot vis_tools"
```

Then start the stack:

```bash
./watod up
```

In a **second terminal**, open a shell in the robot container and run the camera + YOLO launch:

```bash
./watod -t robot
```

Inside the container:

```bash
ros2 launch bringup_robot mac_camera_object_detection.launch.py
```

Leave this running. The node will fetch frames from `http://host.docker.internal:9999/frame` (the proxy you started in step 1) and publish `/image` and `/detections_image`.

## 3. Connect Foxglove and view the streams

1. Open [Foxglove Studio](https://foxglove.dev/studio) (desktop or browser).
2. **Open connection** → **Foxglove WebSocket**.
3. URL: `ws://localhost:<FOXGLOVE_BRIDGE_PORT>`. To see the port: run `./watod -v` once and check the output, or look in `modules/.env` for `FOXGLOVE_BRIDGE_PORT`.
4. Add two **Image** panels: one for topic `/image` (raw camera), one for `/detections_image` (YOLO annotations).

You should see your phone camera feed and the object detection overlay.
