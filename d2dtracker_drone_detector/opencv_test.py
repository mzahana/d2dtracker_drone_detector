import cv2

class FaceDetector:
    def __init__(self, cascade_path='haarcascade_frontalface_default.xml'):
        self.face_cascade = cv2.CascadeClassifier(cascade_path)

    def detect_faces(self, frame):
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray_frame, scaleFactor=1.1, minNeighbors=5)
        return faces

    def draw_faces(self, frame, faces):
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

    def run(self,cap):
        cap = cv2.VideoCapture(0)
        while True:
            ret, frame = cap.read()
            faces = self.detect_faces(frame)
            self.draw_faces(frame, faces)
            cv2.imshow('Face Detection', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()