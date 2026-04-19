#!/usr/bin/env python3
"""
capture_zed_pointcloud.py

Usage:
    python capture_zed_pointcloud.py --frames 30 --output output_pointcloud.ply

Captures N successful frames from a ZED camera and writes a point cloud (XYZRGBA) to a .ply file.
"""
import argparse
import time
import sys

try:
    import pyzed.sl as sl
except Exception as e:
    print("ERROR: could not import pyzed.sl. Make sure ZED SDK + pyzed are installed and PYTHONPATH is set.")
    print("Exception:", e)
    sys.exit(1)


def capture_pointcloud(frames_to_capture: int, output_path: str, timeout_sec: float = 10.0):
    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # 1280x720
    init_params.coordinate_units = sl.UNIT.MILLIMETER
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA         # best depth quality
    init_params.sdk_verbose = False

    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Failed to open ZED camera:", err)
        zed.close()
        return False

    runtime = sl.RuntimeParameters()
    runtime.sensing_mode = sl.SENSING_MODE.STANDARD  # STANDARD or FILL
    runtime.confidence_threshold = 50                # reduce low-confidence points (0-100)

    # Mat to receive XYZRGBA measure
    point_cloud_mat = sl.Mat()

    success_count = 0
    start_time = time.time()
    last_saved = False

    print(f"Starting capture: target successful frames = {frames_to_capture}")
    try:
        while success_count < frames_to_capture:
            if (time.time() - start_time) > timeout_sec:
                print("Timeout reached while waiting for frames.")
                break

            grab_err = zed.grab(runtime)
            if grab_err == sl.ERROR_CODE.SUCCESS:
                # retrieve XYZRGBA (floating x,y,z in mm + packed RGBA)
                zed.retrieve_measure(point_cloud_mat, sl.MEASURE.XYZRGBA)
                success_count += 1
                print(f"Captured frame {success_count}/{frames_to_capture}")
                # short sleep to allow slight motion capture if needed
                time.sleep(0.05)
            else:
                # not fatal, keep trying
                # print(".", end="", flush=True)
                time.sleep(0.01)

        if success_count == 0:
            print("No successful frames captured. Exiting.")
            zed.close()
            return False

        # Write the last retrieved measure to PLY
        print("Writing point cloud to:", output_path)
        write_err = point_cloud_mat.write(output_path)
        # write returns an sl.ERROR_CODE (SUCCESS == 0)
        if write_err != sl.ERROR_CODE.SUCCESS:
            print("Warning: write returned error code:", write_err)
            # Try fallback via save to filename string (some SDK versions)
            try:
                point_cloud_mat.save(output_path)
                print("Saved using fallback .save()")
            except Exception as ex:
                print("Fallback save failed:", ex)
                zed.close()
                return False

        last_saved = True

    except KeyboardInterrupt:
        print("\nCapture interrupted by user.")
    finally:
        zed.close()

    if last_saved:
        print("Done. Point cloud saved as:", output_path)
        return True
    else:
        print("Point cloud not saved.")
        return False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Capture point cloud from ZED and save to PLY")
    parser.add_argument("--frames", type=int, default=30, help="Number of successful frames to capture (10-30 recommended)")
    parser.add_argument("--output", type=str, default="output_pointcloud.ply", help="Output PLY filename")
    parser.add_argument("--timeout", type=float, default=15.0, help="Timeout (sec) waiting for frames")
    args = parser.parse_args()

    if args.frames < 1:
        print("frames must be >= 1")
        sys.exit(1)

    ok = capture_pointcloud(args.frames, args.output, args.timeout)
    sys.exit(0 if ok else 1)
