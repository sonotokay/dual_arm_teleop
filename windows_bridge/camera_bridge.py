import cv2
import mediapipe as mp
import socket
import json
import math
import time

HOST = "0.0.0.0"
PORT = 5555

mp_hands = mp.solutions.hands

WRIST = 0
INDEX_TIP = 8
THUMB_TIP = 4
MIDDLE_MCP = 9


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


def build_message(results):
    msg = {"t": time.time(), "left": empty_hand(), "right": empty_hand()}
    if not results.multi_hand_landmarks:
        return msg
    for landmarks, handedness in zip(
        results.multi_hand_landmarks, results.multi_handedness
    ):
        label = handedness.classification[0].label.lower()  # "left" or "right"
        score = handedness.classification[0].score
        if label in ("left", "right"):
            msg[label] = extract_hand(landmarks.landmark, score)
    return msg


def serve():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(1)
    srv.settimeout(1.0)
    print(f"[bridge] listening on {HOST}:{PORT}")

    cap = cv2.VideoCapture(0)
    hands = mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.5)

    try:
        while True:
            print("[bridge] waiting for subscriber...")
            while True:
                try:
                    conn, addr = srv.accept()
                    break
                except socket.timeout:
                    continue
            print(f"[bridge] connected: {addr}")
            try:
                while True:
                    ok, frame = cap.read()
                    if not ok:
                        continue
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    results = hands.process(rgb)
                    msg = build_message(results)
                    line = json.dumps(msg) + "\n"
                    conn.sendall(line.encode("utf-8"))
            except (ConnectionResetError, BrokenPipeError, ConnectionAbortedError, OSError):
                print("[bridge] subscriber disconnected")
            finally:
                conn.close()
    finally:
        cap.release()
        hands.close()
        srv.close()


if __name__ == "__main__":
    serve()