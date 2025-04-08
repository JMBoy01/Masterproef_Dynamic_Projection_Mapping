import cv2 as cv
import numpy as np
import socket
import struct

# ------------ init code ------------

def load_intrinsic_calibration(filename):
    fs = cv.FileStorage(filename, cv.FILE_STORAGE_READ)
    cameraMatrix = fs.getNode("intrinsic").mat()
    distCoeffs = fs.getNode("dist_coeffs").mat()
    fs.release()
    return cameraMatrix, distCoeffs

def load_extrinsic_calibration(filename):
    fs = cv.FileStorage(filename, cv.FILE_STORAGE_READ)
    cam2proj = fs.getNode("extrinsic").mat()
    fs.release()
    return cam2proj

# ------------ init code ------------



# ------------ matrix manipulation code ------------

def make_transformation_matrix(rvec, tvec):
    R, _ = cv.Rodrigues(rvec)

    w2cam = np.eye(4)
    w2cam[:3, :3] = R
    w2cam[:3, 3] = tvec.flatten()

    return w2cam

def convert_pose_cv_to_gl(pose):
    cv_to_gl = np.array([
        [1,  0,  0,  0],
        [0, -1,  0,  0],
        [0,  0, -1,  0],
        [0,  0,  0,  1]
    ])

    return cv_to_gl @ pose

# ------------ matrix manipulation code ------------



# ----------------- send data code -----------------

class UDPSender:
    def __init__(self, IP, PORT):
        self.IP = IP
        self.PORT = PORT
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_data(self, flag, data):
        message = struct.pack("c16d", flag.encode(), *data.flatten())
        self.sock.sendto(message, (self.IP, self.PORT))
        # print(f"Sent {flag}-matrix:\n{data}\n")

def prepare_intrinsic_data_to_send(intrinsics, width=1920, height=1080, zNear=0.01, zFar=100):
    fx = intrinsics[0, 0]
    fy = intrinsics[1, 1]
    cx = intrinsics[0, 2]
    cy = intrinsics[1, 2]

    data = [fx, fy, cx, cy, width, height, zNear, zFar]
    for i in range(16 - len(data)):
        data.append(0)

    return np.array(data)

# ----------------- send data code -----------------



def main():
    # init sender object
    sender = UDPSender("127.0.0.1", 5005)

    # init cap
    cap = cv.VideoCapture(1, apiPreference=cv.CAP_ANY, params=[cv.CAP_PROP_FRAME_WIDTH, 1920, cv.CAP_PROP_FRAME_HEIGHT, 1080])
    # cap = cv.VideoCapture(0)

    _, init_img = cap.read()
    print(init_img.shape)
    camera_img_width = init_img.shape[1]
    camera_img_height = init_img.shape[0]

    # dictionary for aruco detector and board
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)

    # aruco detecor, TODO: check if extra parameters need to be defined and optimized
    aruco_detector = cv.aruco.ArucoDetector(dictionary)

    # aruco board
    board_size = (2, 2)
    marker_size = 0.06 # meter
    marker_separation = 0.20 # meter
    ids = np.array([1, 2, 4, 3]) # 4, 3 omdat dit de juiste volgorde is die overeen komt met mijn maquette
    board = cv.aruco.GridBoard(board_size, marker_size, marker_separation, dictionary, ids)

    # save board image
    # img_size = 1000
    # board_img = board.generateImage((img_size, img_size))
    # cv.imwrite("board.png", board_img)

    # camera intrinsic parameters
    # intrinsics, dist_coeffs = load_intrinsic_calibration("./Calibration/cameraHome_1280x800")
    intrinsics, dist_coeffs = load_intrinsic_calibration("./Calibration/cameraSchool")

    # projector intrinsic parameters
    # proj_intrinsics, proj_dist_coeffs = load_intrinsic_calibration("./Calibration/projHome")
    proj_intrinsics, proj_dist_coeffs = load_intrinsic_calibration("./Calibration/projSchool")
    proj_img_width = 1920
    proj_img_height = 1080

    # intrinsics_to_send = prepare_intrinsic_data_to_send(intrinsics, width=camera_img_width, height=camera_img_height)
    intrinsics_to_send = prepare_intrinsic_data_to_send(proj_intrinsics, width=proj_img_width, height=proj_img_height)
    sender.send_data('i', intrinsics_to_send)

    # extrinsic parameters = cam2proj transformation matrix
    # extrinsic_matrix = load_extrinsic_calibration("./Calibration/extrinsicHome")
    extrinsic_matrix = load_extrinsic_calibration("./Calibration/extrinsicSchool")

    while True:
        # read img, convert to grayscale
        _, img = cap.read()
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # detect markers
        corners, ids, rejectedImgPoints = aruco_detector.detectMarkers(gray)
        detectedCorners, detectedIds, _, _ = aruco_detector.refineDetectedMarkers(gray, board, corners, ids, rejectedImgPoints, intrinsics, dist_coeffs)

        cv.aruco.drawDetectedMarkers(img, detectedCorners, detectedIds)
        
        # show img with detected markers
        scale = 0.6
        img = cv.resize(img, (int(img.shape[1] * scale), int(img.shape[0] * scale)))
        cv.imshow("camera", img)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

        if detectedIds is None or len(detectedIds) == 0:
            continue

        obj_points, img_points = board.matchImagePoints(detectedCorners, detectedIds)
        if obj_points is None or img_points is None or len(obj_points) < 4 or len(img_points) < 4:
            continue

        retval, rvec, tvec = cv.solvePnP(obj_points, img_points, intrinsics, dist_coeffs)
        if not retval:
            continue
        
        # pose = world2cam transfromation matrix of tracked object
        pose = make_transformation_matrix(rvec, tvec) # combine rvec and tvec into 4x4 transformation_matrix
        pose = extrinsic_matrix @ pose # cam2proj @ world2cam -> world2proj

        pose = convert_pose_cv_to_gl(pose) # world2proj in opengl coordinate system

        sender.send_data('p', pose)

        if cv.waitKey(1) & 0xFF == ord('p'):
            print("pose:")
            print(pose)
            print("------------------------")

    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()