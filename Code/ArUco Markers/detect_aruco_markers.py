import cv2
import cv2.aruco as aruco

# Instellen van de dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

# Start de webcam
cap = cv2.VideoCapture(0)

while True:
    # Lees het beeld van de webcam
    ret, frame = cap.read()
    if not ret:
        print("Kan geen video ontvangen")
        break

    # Converteer het beeld naar grijswaarden
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Pas contrastverhoging toe
    _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    # Detecteer de ArUco markers
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # Als markers zijn gedetecteerd
    if ids is not None:
        # Teken de markers op het beeld
        aruco.drawDetectedMarkers(frame, corners, ids)

    # Toon het beeld
    cv2.imshow('ArUco Marker Detection', frame)

    # Stop de loop als de 'q' toets wordt ingedrukt
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Ruim op
cap.release()
cv2.destroyAllWindows()
