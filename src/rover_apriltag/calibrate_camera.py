import cv2
import numpy as np
import argparse
import os
import datetime

# ── Checkerboard config ───────────────────────────────────────────────────────
CHECKERBOARD = (9, 6)   # internal corners (9x6 → 10x7 squares)
SQUARE_SIZE  = 0.0265   # meters per square (2.65 cm)
MIN_IMAGES   = 10       # minimum captures before calibration is allowed
# ─────────────────────────────────────────────────────────────────────────────

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE


def save_ros_yaml(path: str, camera_name: str, camera_matrix, dist_coeffs, image_size, reprojection_error):
    w, h = image_size
    K = camera_matrix.flatten().tolist()
    D = dist_coeffs.flatten().tolist()

    # Rectification matrix (identity for mono camera)
    R = [1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0]

    # Projection matrix (P = [K | 0] for mono)
    P = [K[0], K[1], K[2], 0.0,
         K[3], K[4], K[5], 0.0,
         K[6], K[7], K[8], 0.0]

    content = f"""image_width: {w}
image_height: {h}
camera_name: {camera_name}
camera_matrix:
  rows: 3
  cols: 3
  data: {K}
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: {D[:5]}
rectification_matrix:
  rows: 3
  cols: 3
  data: {R}
projection_matrix:
  rows: 3
  cols: 4
  data: {P}
# reprojection_error: {reprojection_error:.4f} pixels
"""
    with open(path, "w") as f:
        f.write(content)
    print(f"\n✓ ROS YAML saved to: {path}")
    print(f"  Set camera_info_url: \"file://{os.path.abspath(path)}\"")


def run_calibration(objpoints, imgpoints, image_shape, camera_name, output_dir):
    h, w = image_shape[:2]
    print("\n" + "="*60)
    print("Running calibration...")

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, (w, h), None, None
    )

    # Reprojection error
    mean_error = 0.0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        mean_error += cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    mean_error /= len(objpoints)

    print("\nCamera Matrix:")
    print(camera_matrix)
    print("\nDistortion Coefficients:")
    labels = ["k1", "k2", "p1", "p2", "k3"]
    for label, val in zip(labels, dist_coeffs.flatten()):
        print(f"  {label} = {val:.6f}")
    print(f"\nReprojection Error: {mean_error:.4f} pixels")
    if mean_error < 0.5:
        print("✓ Excellent calibration! (< 0.5 px)")
    elif mean_error < 1.0:
        print("✓ Good calibration (< 1.0 px)")
    else:
        print("⚠ High error — consider recapturing from more varied angles")

    os.makedirs(output_dir, exist_ok=True)
    yaml_path = os.path.join(output_dir, f"{camera_name}.yaml")
    save_ros_yaml(yaml_path, camera_name, camera_matrix, dist_coeffs, (w, h), mean_error)


def main():
    parser = argparse.ArgumentParser(description="Interactive camera calibration with live preview")
    parser.add_argument("--device",      type=int,   default=0,        help="Video device index (e.g. 0 for /dev/video0)")
    parser.add_argument("--name",        type=str,   default="camera",  help="Camera name (used for output filename, e.g. left, center, right)")
    parser.add_argument("--output-dir",  type=str,   default=os.path.expanduser("~/.ros/camera_info"), help="Directory to save YAML calibration file")
    parser.add_argument("--width",       type=int,   default=1920,     help="Capture width")
    parser.add_argument("--height",      type=int,   default=1080,     help="Capture height")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    if not cap.isOpened():
        print(f"ERROR: Could not open /dev/video{args.device}")
        return

    objpoints = []
    imgpoints = []
    last_frame = None
    captured = 0

    print("="*60)
    print(f"Calibrating camera: {args.name}  (/dev/video{args.device})")
    print(f"Checkerboard: {CHECKERBOARD[0]}x{CHECKERBOARD[1]} inner corners, {SQUARE_SIZE*100:.2f} cm squares")
    print("="*60)
    print("  SPACE  → capture frame (checkerboard must be detected)")
    print("  C      → run calibration now (needs >= 10 captures)")
    print("  Q      → quit without calibrating")
    print("="*60)

    window = f"Calibration — {args.name} (SPACE=capture, C=calibrate, Q=quit)"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("ERROR: Failed to grab frame")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        display = frame.copy()

        if found:
            cv2.drawChessboardCorners(display, CHECKERBOARD, corners, found)
            status_color = (0, 255, 0)
            status_text  = "Checkerboard DETECTED — press SPACE to capture"
        else:
            status_color = (0, 0, 255)
            status_text  = "No checkerboard detected"

        cv2.putText(display, status_text,       (20, 40),  cv2.FONT_HERSHEY_SIMPLEX, 1.0, status_color, 2)
        cv2.putText(display, f"Captured: {captured}", (20, 80),  cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        if captured < MIN_IMAGES:
            cv2.putText(display, f"Need {MIN_IMAGES - captured} more before calibrating",
                        (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)

        cv2.imshow(window, display)
        last_frame = gray
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            print("Quitting without calibration.")
            break

        elif key == ord(' '):
            if not found:
                print("✗ Checkerboard not detected — move board into view and try again")
            else:
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                objpoints.append(objp)
                imgpoints.append(corners2)
                captured += 1
                print(f"✓ Captured frame {captured}")

        elif key == ord('c'):
            if captured < MIN_IMAGES:
                print(f"⚠ Only {captured} captures — need at least {MIN_IMAGES}. Keep going!")
            else:
                cv2.destroyAllWindows()
                cap.release()
                run_calibration(objpoints, imgpoints, last_frame.shape, args.name, args.output_dir)
                return

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()