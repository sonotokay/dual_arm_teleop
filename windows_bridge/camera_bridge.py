import cv2
import mediapipe as mp
import socket
import json
import math
import time

HOST = "0.0.0.0"
PORT = 5555

mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

WRIST = 0
INDEX_TIP = 8
THUMB_TIP = 4
MIDDLE_MCP = 9

PREVIEW_WINDOW = "dual_arm_bridge (ESC quits)"


def empty_hand():
    return {
        "visible": False,
        "confidence": 0.0,
        "wrist": [0.0, 0.0],
        "rotation": 0.0,
        "pinch": 0.0,
    }


def extract_hand(landmarks, score):
    wrist = landmarks[WRIST]
    middle_mcp = landmarks[MIDDLE_MCP]
    thumb_tip = landmarks[THUMB_TIP]
    index_tip = landmarks[INDEX_TIP]
    rotation = math.atan2(middle_mcp.y - wrist.y, middle_mcp.x - wrist.x)
    pinch = math.hypot(thumb_tip.x - index_tip.x, thumb_tip.y - index_tip.y)
    return {
        "visible": True,
        "confidence": float(score),
        "wrist": [float(wrist.x), float(wrist.y)],
        "rotation": float(rotation),
        "pinch": float(pinch),
    }


def slot_for(wrist_x: float) -> str:
    # Route by image position, not MediaPipe handedness (which flips depending
    # on whether the webcam already mirrors).
    return "right" if wrist_x >= 0.5 else "left"


def build_message(results):
    msg = {"t": time.time(), "left": empty_hand(), "right": empty_hand()}
    if not results.multi_hand_landmarks:
        return msg
    for landmarks, handedness in zip(
        results.multi_hand_landmarks, results.multi_handedness
    ):
        score = handedness.classification[0].score
        slot = slot_for(landmarks.landmark[WRIST].x)
        msg[slot] = extract_hand(landmarks.landmark, score)
    return msg


def annotate(frame, results, fps, connected):
    h, w = frame.shape[:2]
    if results.multi_hand_landmarks:
        for landmarks, handedness in zip(
            results.multi_hand_landmarks, results.multi_handedness
        ):
            mp_draw.draw_landmarks(frame, landmarks, mp_hands.HAND_CONNECTIONS)
            score = handedness.classification[0].score
            wrist = landmarks.landmark[WRIST]
            slot = slot_for(wrist.x).upper()
            px, py = int(wrist.x * w), int(wrist.y * h)
            cv2.putText(frame, f"{slot} {score:.2f}", (px, max(py - 10, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.putText(frame, f"{fps:.1f} fps", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    status = "STREAMING" if connected else "waiting for subscriber"
    color = (0, 255, 0) if connected else (0, 200, 200)
    cv2.putText(frame, status, (10, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)


def serve():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(1)
    srv.setblocking(False)
    print(f"[bridge] listening on {HOST}:{PORT}")

    cap = cv2.VideoCapture(0)
    hands = mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.5)

    conn = None
    prev_t = time.time()
    fps_smooth = 0.0
    running = True
    print("[bridge] waiting for subscriber...")

    try:
        while running:
            if conn is None:
                try:
                    conn, addr = srv.accept()
                    print(f"[bridge] connected: {addr}")
                except BlockingIOError:
                    pass

            ok, frame = cap.read()
            if not ok:
                continue
            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = hands.process(rgb)

            if conn is not None:
                msg = build_message(results)
                line = json.dumps(msg) + "\n"
                try:
                    conn.sendall(line.encode("utf-8"))
                except (ConnectionResetError, BrokenPipeError,
                        ConnectionAbortedError, OSError):
                    print("[bridge] subscriber disconnected")
                    conn.close()
                    conn = None
                    print("[bridge] waiting for subscriber...")

            now = time.time()
            dt = now - prev_t
            prev_t = now
            inst = 1.0 / dt if dt > 0 else 0.0
            fps_smooth = 0.9 * fps_smooth + 0.1 * inst

            annotate(frame, results, fps_smooth, conn is not None)
            cv2.imshow(PREVIEW_WINDOW, frame)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC
                running = False
    finally:
        if conn is not None:
            conn.close()
        cap.release()
        hands.close()
        cv2.destroyAllWindows()
        srv.close()


if __name__ == "__main__":
    serve()