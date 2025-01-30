import cv2

def start_camera():
    cap = cv2.VideoCapture(0)  # 0 - kompyuterning asosiy kamerasi

    if not cap.isOpened():
        print("Kamera ochilmadi!")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Kadrlani o'qib bo'lmadi!")
            break

        cv2.imshow("Kamera", frame)

        # 'q' tugmasini bosganda dastur to'xtaydi
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    start_camera()
