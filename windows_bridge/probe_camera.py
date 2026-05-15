import cv2
import mediapipe as mp
import time

mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

cap = cv2.VideoCapture(0)
prev = time.time()
fps_smooth = 0.0

with mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.5) as hands:
    while True:
        ok, frame = cap.read()
        if not ok:
            break

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = hands.process(rgb)

        if result.multi_hand_landmarks:
            for hl in result.multi_hand_landmarks:
                mp_draw.draw_landmarks(frame, hl, mp_hands.HAND_CONNECTIONS)

        now = time.time()
        dt = now - prev
        prev = now
        fps = 1.0 / dt if dt > 0 else 0
        fps_smooth = 0.9 * fps_smooth + 0.1 * fps

        cv2.putText(frame, f"{fps_smooth:.1f} fps", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow("hands", frame)
        if cv2.waitKey(1) & 0xFF == 27:  # Esc
            break

cap.release()
cv2.destroyAllWindows()