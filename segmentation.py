# import cv2
# import numpy as np
# from pathlib import Path

# # =========================
# # HSV THRESHOLDS (your values, 0-255 scale)
# # Note: H wraps around because 250 -> 72 crosses 0
# # =========================
# H_MIN, H_MAX = 250, 72
# S_MIN, S_MAX = 0, 3
# V_MIN, V_MAX = 155, 211

# # =========================
# # SETTINGS
# # =========================
# INPUT_DIR  = "F:\Robotics Automation system AI\PROf Hani\DETECTION\245.png"         # folder of input images
# OUTPUT_DIR = "seg_out"        # saves here
# HSV_SCALE = 255               # set 255 if your H is 0-255, set 179 if already OpenCV H

# # Morphology (tweak if needed)
# OPEN_K  = 3
# CLOSE_K = 3

# def to_opencv_hsv_thresholds(hmin, hmax, smin, smax, vmin, vmax, hsv_scale=255):
#     """
#     OpenCV HSV uses: H in [0,179], S,V in [0,255].
#     If your thresholds are in 0-255 for H, convert to 0-179.
#     """
#     if hsv_scale == 255:
#         # convert hue 0-255 -> 0-179
#         hmin_cv = int(round(hmin * 179.0 / 255.0))
#         hmax_cv = int(round(hmax * 179.0 / 255.0))
#     else:
#         hmin_cv, hmax_cv = int(hmin), int(hmax)

#     # S,V are already 0-255 in OpenCV
#     smin_cv, smax_cv = int(smin), int(smax)
#     vmin_cv, vmax_cv = int(vmin), int(vmax)

#     return hmin_cv, hmax_cv, smin_cv, smax_cv, vmin_cv, vmax_cv

# def hsv_segment(bgr, hmin, hmax, smin, smax, vmin, vmax, hsv_scale=255):
#     hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

#     hmin, hmax, smin, smax, vmin, vmax = to_opencv_hsv_thresholds(
#         hmin, hmax, smin, smax, vmin, vmax, hsv_scale=hsv_scale
#     )

#     lower1 = np.array([hmin, smin, vmin], dtype=np.uint8)
#     upper1 = np.array([hmax, smax, vmax], dtype=np.uint8)

#     if hmin <= hmax:
#         mask = cv2.inRange(hsv, lower1, upper1)
#     else:
#         # wrap-around case: [hmin..179] U [0..hmax]
#         lower_a = np.array([hmin, smin, vmin], dtype=np.uint8)
#         upper_a = np.array([179,  smax, vmax], dtype=np.uint8)
#         lower_b = np.array([0,    smin, vmin], dtype=np.uint8)
#         upper_b = np.array([hmax, smax, vmax], dtype=np.uint8)
#         mask = cv2.inRange(hsv, lower_a, upper_a) | cv2.inRange(hsv, lower_b, upper_b)

#     # clean mask
#     k_open  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (OPEN_K, OPEN_K))
#     k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (CLOSE_K, CLOSE_K))
#     mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k_open,  iterations=1)
#     mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_close, iterations=1)

#     segmented = cv2.bitwise_and(bgr, bgr, mask=mask)
#     return mask, segmented

# def main():
#     in_dir = Path(INPUT_DIR)
#     out_dir = Path(OUTPUT_DIR)
#     out_dir.mkdir(parents=True, exist_ok=True)

#     exts = {".jpg", ".jpeg", ".png", ".bmp"}
#     imgs = [p for p in in_dir.iterdir() if p.suffix.lower() in exts]

#     if not imgs:
#         print(f"No images found in: {in_dir.resolve()}")
#         return

#     for p in imgs:
#         bgr = cv2.imread(str(p))
#         if bgr is None:
#             print("Skip (cannot read):", p.name)
#             continue

#         mask, seg = hsv_segment(bgr, H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX, hsv_scale=HSV_SCALE)

#         cv2.imwrite(str(out_dir / f"{p.stem}_mask.png"), mask)
#         cv2.imwrite(str(out_dir / f"{p.stem}_seg.png"), seg)

#         # Optional: overlay mask on original for quick check
#         overlay = bgr.copy()
#         overlay[mask > 0] = (0, 255, 0)  # green highlight
#         preview = cv2.addWeighted(bgr, 0.8, overlay, 0.2, 0)
#         cv2.imwrite(str(out_dir / f"{p.stem}_overlay.png"), preview)

#         print("Saved:", p.name)

# if __name__ == "__main__":
#     main()






# import cv2
# import numpy as np

# # =========================
# # IMAGE PATH
# # =========================
# IMAGE_PATH = r"Screenshot 2026-02-25 174015.png"

# # =========================
# # HSV Thresholds (your values)
# # =========================
# H_MIN, H_MAX = 250, 65
# S_MIN, S_MAX = 0, 30
# V_MIN, V_MAX = 155, 211

# def segment_and_count(path):
#     img = cv2.imread(path)
#     if img is None:
#         print("Image not found!")
#         return

#     hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

#     # Convert hue from 0-255 scale → OpenCV 0-179 scale
#     hmin = int(H_MIN * 179 / 255)
#     hmax = int(H_MAX * 179 / 255)

#     if hmin <= hmax:
#         lower = np.array([hmin, S_MIN, V_MIN])
#         upper = np.array([hmax, S_MAX, V_MAX])
#         mask = cv2.inRange(hsv, lower, upper)
#     else:
#         mask1 = cv2.inRange(hsv,
#                             np.array([hmin, S_MIN, V_MIN]),
#                             np.array([179, S_MAX, V_MAX]))
#         mask2 = cv2.inRange(hsv,
#                             np.array([0, S_MIN, V_MIN]),
#                             np.array([hmax, S_MAX, V_MAX]))
#         mask = mask1 | mask2

#     # =========================
#     # COUNT DETECTED PIXELS
#     # =========================
#     detected_pixels = cv2.countNonZero(mask)
#     total_pixels = mask.shape[0] * mask.shape[1]
#     percent = (detected_pixels / total_pixels) * 100

#     print("Detected white pixels:", detected_pixels)
#     print("Total image pixels:", total_pixels)
#     print("Detection percentage: {:.4f}%".format(percent))

#     # =========================
#     # OPTIONAL: Count blobs (objects)
#     # =========================
#     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     print("Detected objects (blobs):", len(contours))

#     # Show results
#     result = cv2.bitwise_and(img, img, mask=mask)
#     cv2.imshow("Mask", mask)
#     cv2.imshow("Segmented", result)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()

# segment_and_count(IMAGE_PATH)





#########3 live camera code 


import cv2
import numpy as np
from collections import deque

# =========================
# CONFIG
# =========================
CAMERA_INDEX = 0

# Your HSV thresholds (KEEP AS IS)
H_MIN, H_MAX = 250, 65
S_MIN, S_MAX = 0, 30
V_MIN, V_MAX = 155, 211

# =========================
# ZOOM SETTINGS (from reference)
# =========================
zoom_factor = 1.0
ZOOM_STEP = 0.1
ZOOM_MIN = 1.0
ZOOM_MAX = 3.0

# =========================
# AVERAGE SETTINGS
# =========================
AVG_WINDOW = 10
pixel_hist = deque(maxlen=AVG_WINDOW)     # stores detected pixels
percent_hist = deque(maxlen=AVG_WINDOW)   # stores percentage
blob_hist = deque(maxlen=AVG_WINDOW)      # stores blob count


def apply_zoom(frame, zoom):
    """Digital zoom by cropping center and resizing back."""
    if zoom == 1.0:
        return frame

    h, w = frame.shape[:2]
    new_w = int(w / zoom)
    new_h = int(h / zoom)

    x1 = (w - new_w) // 2
    y1 = (h - new_h) // 2
    x2 = x1 + new_w
    y2 = y1 + new_h

    cropped = frame[y1:y2, x1:x2]
    return cv2.resize(cropped, (w, h), interpolation=cv2.INTER_LINEAR)


def select_boundary(cap):
    print("Reading camera for selection...")
    ret, frame = cap.read()
    if not ret:
        print("Failed to read from camera!")
        return None

    print("\n[ACTION REQUIRED]: Draw a box around the active area.")
    roi = cv2.selectROI("Select Boundary", frame, fromCenter=False, showCrosshair=True)
    cv2.destroyWindow("Select Boundary")

    if roi == (0, 0, 0, 0):
        print("No boundary selected. Using full screen.")
        return None

    return roi


# =========================
# SEGMENTATION PART (DO NOT CHANGE LOGIC)
# =========================
def segment_mask_from_frame(frame_bgr):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    # Convert hue from 0-255 scale → OpenCV 0-179 scale
    hmin = int(H_MIN * 179 / 255)
    hmax = int(H_MAX * 179 / 255)

    if hmin <= hmax:
        lower = np.array([hmin, S_MIN, V_MIN])
        upper = np.array([hmax, S_MAX, V_MAX])
        mask = cv2.inRange(hsv, lower, upper)
    else:
        mask1 = cv2.inRange(
            hsv,
            np.array([hmin, S_MIN, V_MIN]),
            np.array([179, S_MAX, V_MAX]),
        )
        mask2 = cv2.inRange(
            hsv,
            np.array([0, S_MIN, V_MIN]),
            np.array([hmax, S_MAX, V_MAX]),
        )
        mask = mask1 | mask2

    return mask


def main():
    global zoom_factor

    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    if not cap.isOpened():
        print("Camera error")
        return

    roi = select_boundary(cap)

    print("\nControls:")
    print(" q = quit")
    print(" r = reselect ROI")
    print(" + = zoom in")
    print(" - = zoom out")
    print(" 0 = reset zoom")

    last_printed_avg = None

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # ===== APPLY ZOOM =====
        frame = apply_zoom(frame, zoom_factor)

        # ===== ROI (selected boundary) =====
        inference_frame = frame
        if roi is not None:
            x, y, w, h = roi
            x2 = min(frame.shape[1], x + w)
            y2 = min(frame.shape[0], y + h)

            inference_frame = frame[y:y2, x:x2]

            # Draw ROI on display frame
            cv2.rectangle(frame, (x, y), (x2, y2), (255, 0, 0), 2)

        # ===== SEGMENTATION (UNCHANGED LOGIC) =====
        mask = segment_mask_from_frame(inference_frame)

        # ===== METRICS =====
        detected_pixels = cv2.countNonZero(mask)
        total_pixels = mask.shape[0] * mask.shape[1]
        percent = (detected_pixels / total_pixels) * 100 if total_pixels > 0 else 0.0

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blob_count = len(contours)

        # ===== AVG OF LAST 10 FRAMES =====
        pixel_hist.append(detected_pixels)
        percent_hist.append(percent)
        blob_hist.append(blob_count)

        avg_pixels = int(round(sum(pixel_hist) / len(pixel_hist)))
        avg_percent = (sum(percent_hist) / len(percent_hist)) if percent_hist else 0.0
        avg_blobs = int(round(sum(blob_hist) / len(blob_hist)))

        # Print avg only when it changes (clean output)
        avg_tuple = (avg_pixels, round(avg_percent, 4), avg_blobs)
        if avg_tuple != last_printed_avg and len(pixel_hist) == AVG_WINDOW:
            print(f"[AVG {AVG_WINDOW}] pixels={avg_pixels}  percent={avg_percent:.4f}%  blobs={avg_blobs}")
            last_printed_avg = avg_tuple

        # ===== VISUALIZATION =====
        # Put mask back onto full frame for display if ROI was used
        mask_vis = mask
        result_vis = cv2.bitwise_and(inference_frame, inference_frame, mask=mask)

        # Overlay info on main display frame
        cv2.putText(frame, f"Zoom: {zoom_factor:.1f}x", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)

        cv2.putText(frame, f"Pixels: {detected_pixels} ({percent:.4f}%)", (20, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

        cv2.putText(frame, f"Blobs: {blob_count}", (20, 95),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

        cv2.putText(frame, f"AVG{AVG_WINDOW} Pixels: {avg_pixels} ({avg_percent:.4f}%)", (20, 125),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

        cv2.putText(frame, f"AVG{AVG_WINDOW} Blobs: {avg_blobs}", (20, 155),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

        cv2.imshow("Live Feed", frame)
        cv2.imshow("Mask (ROI)", mask_vis)
        cv2.imshow("Segmented (ROI)", result_vis)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            roi = select_boundary(cap)
            pixel_hist.clear()
            percent_hist.clear()
            blob_hist.clear()
            last_printed_avg = None
        elif key == ord('+') or key == ord('='):
            zoom_factor = min(ZOOM_MAX, zoom_factor + ZOOM_STEP)
        elif key == ord('-'):
            zoom_factor = max(ZOOM_MIN, zoom_factor - ZOOM_STEP)
        elif key == ord('0'):
            zoom_factor = 1.0

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

















#----------------------------------------------------------------------------------------------------------------
#below code is without avg 


# import cv2
# import numpy as np

# # =========================
# # CONFIG
# # =========================
# CAMERA_INDEX = 0

# # Your HSV thresholds (UNCHANGED)
# H_MIN, H_MAX = 250, 65
# S_MIN, S_MAX = 0, 30
# V_MIN, V_MAX = 155, 211

# # =========================
# # ZOOM SETTINGS
# # =========================
# zoom_factor = 1.0
# ZOOM_STEP = 0.1
# ZOOM_MIN = 1.0
# ZOOM_MAX = 3.0


# def apply_zoom(frame, zoom):
#     """Digital zoom by cropping center and resizing back."""
#     if zoom == 1.0:
#         return frame

#     h, w = frame.shape[:2]
#     new_w = int(w / zoom)
#     new_h = int(h / zoom)

#     x1 = (w - new_w) // 2
#     y1 = (h - new_h) // 2
#     x2 = x1 + new_w
#     y2 = y1 + new_h

#     cropped = frame[y1:y2, x1:x2]
#     return cv2.resize(cropped, (w, h), interpolation=cv2.INTER_LINEAR)


# def select_boundary(cap):
#     print("Reading camera for selection...")
#     ret, frame = cap.read()
#     if not ret:
#         print("Failed to read from camera!")
#         return None

#     print("\n[ACTION REQUIRED]: Draw a box around the active area.")
#     roi = cv2.selectROI("Select Boundary", frame, fromCenter=False, showCrosshair=True)
#     cv2.destroyWindow("Select Boundary")

#     if roi == (0, 0, 0, 0):
#         print("No boundary selected. Using full screen.")
#         return None

#     return roi


# # =========================
# # SEGMENTATION (UNCHANGED)
# # =========================
# def segment_mask_from_frame(frame_bgr):
#     hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

#     hmin = int(H_MIN * 179 / 255)
#     hmax = int(H_MAX * 179 / 255)

#     if hmin <= hmax:
#         lower = np.array([hmin, S_MIN, V_MIN])
#         upper = np.array([hmax, S_MAX, V_MAX])
#         mask = cv2.inRange(hsv, lower, upper)
#     else:
#         mask1 = cv2.inRange(
#             hsv,
#             np.array([hmin, S_MIN, V_MIN]),
#             np.array([179, S_MAX, V_MAX]),
#         )
#         mask2 = cv2.inRange(
#             hsv,
#             np.array([0, S_MIN, V_MIN]),
#             np.array([hmax, S_MAX, V_MAX]),
#         )
#         mask = mask1 | mask2

#     return mask


# def main():
#     global zoom_factor

#     cap = cv2.VideoCapture(CAMERA_INDEX)
#     cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
#     cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

#     if not cap.isOpened():
#         print("Camera error")
#         return

#     roi = select_boundary(cap)

#     print("\nControls:")
#     print(" q = quit")
#     print(" r = reselect ROI")
#     print(" + = zoom in")
#     print(" - = zoom out")
#     print(" 0 = reset zoom")

#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             break

#         # Apply zoom
#         frame = apply_zoom(frame, zoom_factor)

#         inference_frame = frame

#         # ROI
#         if roi is not None:
#             x, y, w, h = roi
#             x2 = min(frame.shape[1], x + w)
#             y2 = min(frame.shape[0], y + h)

#             inference_frame = frame[y:y2, x:x2]
#             cv2.rectangle(frame, (x, y), (x2, y2), (255, 0, 0), 2)

#         # Segmentation
#         mask = segment_mask_from_frame(inference_frame)

#         # Metrics
#         detected_pixels = cv2.countNonZero(mask)
#         total_pixels = mask.shape[0] * mask.shape[1]
#         percent = (detected_pixels / total_pixels) * 100 if total_pixels > 0 else 0.0

#         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         blob_count = len(contours)

#         # Print every frame
#         print(f"Pixels: {detected_pixels} | Percent: {percent:.4f}% | Blobs: {blob_count}")

#         # Visualization
#         result_vis = cv2.bitwise_and(inference_frame, inference_frame, mask=mask)

#         cv2.putText(frame, f"Zoom: {zoom_factor:.1f}x", (20, 30),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)

#         cv2.putText(frame, f"Pixels: {detected_pixels} ({percent:.4f}%)", (20, 65),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

#         cv2.putText(frame, f"Blobs: {blob_count}", (20, 95),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

#         cv2.imshow("Live Feed", frame)
#         cv2.imshow("Mask (ROI)", mask)
#         cv2.imshow("Segmented (ROI)", result_vis)

#         key = cv2.waitKey(1) & 0xFF
#         if key == ord('q'):
#             break
#         elif key == ord('r'):
#             roi = select_boundary(cap)
#         elif key == ord('+') or key == ord('='):
#             zoom_factor = min(ZOOM_MAX, zoom_factor + ZOOM_STEP)
#         elif key == ord('-'):
#             zoom_factor = max(ZOOM_MIN, zoom_factor - ZOOM_STEP)
#         elif key == ord('0'):
#             zoom_factor = 1.0

#     cap.release()
#     cv2.destroyAllWindows()


# if __name__ == "__main__":
#     main()