import cv2 as cv
import numpy as np
import screeninfo

# ------------ init code ------------

def loadCameraCalibration(filename):
    fs = cv.FileStorage(filename, cv.FILE_STORAGE_READ)
    cameraMatrix = fs.getNode("intrinsic").mat()
    distCoeffs = fs.getNode("dist_coeffs").mat()
    fs.release()
    return cameraMatrix, distCoeffs

def makeCirclePatternImages(pattern_size, img_size=(1080, 1920), space_between=100, circle_size=30):
    images = []
    all_circle_centers = []
    for itr in range(0, 9, 1):
        start_x = 0
        start_y = 0

        match itr:
            case 0:
                start_x += space_between
                start_y += space_between
            case 1:
                start_x = img_size[1] - pattern_size[0] * 2 * space_between
                start_y += space_between
            case 2:
                start_x = img_size[1] - pattern_size[0] * 2 * space_between
                start_y = img_size[0] - pattern_size[1] * space_between
            case 3:
                start_x += space_between
                start_y = img_size[0] - pattern_size[1] * space_between
            case 4:
                start_x = img_size[1]/4 - 2 * space_between
                start_y = img_size[0]/4 - space_between
            case 5:
                start_x = img_size[1]*3/4 - pattern_size[1] * space_between
                start_y = img_size[0]/4 - space_between
            case 6:
                start_x = img_size[1]*3/4 - pattern_size[1] * space_between
                start_y = img_size[0]*3/4 - pattern_size[0] * space_between
            case 7:
                start_x = img_size[1]/4 - 2 * space_between
                start_y = img_size[0]*3/4 - pattern_size[0] * space_between
            case 8:
                start_x = img_size[1]/2 - pattern_size[0] * space_between
                start_y = img_size[0]/2 - 3 * space_between
        
        img = np.zeros(img_size, dtype=np.uint8)
        circle_centers = []
        for i in range(0, pattern_size[1], 1):
            for j in range(0, pattern_size[0], 1):
                x = int(start_x + 2 * j * space_between + (i % 2) * space_between)
                y = int(start_y + i * space_between)

                img = cv.circle(img, (x, y), circle_size, 255, -1)
                circle_centers.append((x, y))

        images.append(img)
        all_circle_centers.append(circle_centers)

        cv.imwrite('./Images/Circle Patterns/pattern_' + str(itr) + ".png", img)

    return images, all_circle_centers

def initBlobDetector():
    params = cv.SimpleBlobDetector_Params()
    params.blobColor = 255 # 0 = zwart, 255 = wit
    params.filterByColor = True
    params.filterByArea = True
    params.minCircularity = 0.5
    params.minDistBetweenBlobs = 5
    params.filterByCircularity = True
    params.filterByConvexity = False
    params.filterByInertia = False
    params.collectContours = True

    blobDetector = cv.SimpleBlobDetector_create(params)
    return blobDetector

def create_fullscreen_window(screen_id, window_name):
    if screen_id >= len(screeninfo.get_monitors()):
        return
    screen = screeninfo.get_monitors()[screen_id]

    cv.namedWindow(window_name, cv.WND_PROP_FULLSCREEN)
    cv.moveWindow(window_name, screen.x - 1, screen.y - 1)
    cv.setWindowProperty(window_name, cv.WND_PROP_FULLSCREEN, cv.WINDOW_FULLSCREEN)
    return

# ------------ init code ------------



# -------- calibration code ---------

def save_intrinsic_calibration(filename, intrinsic, dist_coeffs):
    fs = cv.FileStorage(filename, cv.FILE_STORAGE_WRITE)
    fs.write("intrinsic", intrinsic)
    fs.write("dist_coeffs", dist_coeffs)

    fs.release()
    print("Matrices opgeslagen in ", filename)

def save_extrinsic_calibration(filename, extrinsic_matrix):
    fs = cv.FileStorage(filename, cv.FILE_STORAGE_WRITE)
    fs.write("extrinsic", extrinsic_matrix)
    
    fs.release()
    print("Matrix opgeslagen in ", filename)

def make_transformation_matrix(R, T):
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = R
    transformation_matrix[:3, 3] = T.flatten()

    return transformation_matrix

# -------- calibration code ---------



def main():
    cap = cv.VideoCapture(1, apiPreference=cv.CAP_ANY, params=[cv.CAP_PROP_FRAME_WIDTH, 1920, cv.CAP_PROP_FRAME_HEIGHT, 1080])
    # cap = cv.VideoCapture(0)
    _, img = cap.read()
    imgSize = (img.shape[1], img.shape[0])
    
    # Debug print
    print("imgSize:")
    print(imgSize)

    # Load camera calibration
    intrinsic_matrix, dist_coeffs = loadCameraCalibration("./Calibration/cameraSchool")

    # Make circle patterns
    circle_pattern_size = (5, 7) # aantal cirkels (hor, ver)
    circle_pattern_img_size = (1080, 1920)
    circle_pattern_images, circle_pattern_centers = makeCirclePatternImages(circle_pattern_size, img_size=circle_pattern_img_size, space_between=100, circle_size=30)

    # Init charuco board
    board_size = (8, 6) # amount of black square + aruco square (hor, ver)
    board_inner_corner_size = (board_size[0] - 1, board_size[1] - 1)
    board_inner_corner_amount = board_inner_corner_size[0] * board_inner_corner_size[1]
    square_length = 0.03 # meter
    marker_length = 0.015 # meter
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_50)
    charuco_board = cv.aruco.CharucoBoard(board_size, square_length, marker_length, dictionary)

    # Init detectors
    aruco_detector = cv.aruco.ArucoDetector(dictionary)
    charuco_detector = cv.aruco.CharucoDetector(charuco_board)
    blob_detector = initBlobDetector()

    # Set full screen window on projector
    print(screeninfo.get_monitors())
    create_fullscreen_window(1, "circle_pattern")

    # Init arrays to store all calibration data
    all_centers_proj_plane = []
    all_centers_proj = []
    all_centers_cam = []

    # Start projecting pattern image
    circle_pattern_img_index = 0
    circle_pattern_img = circle_pattern_images[circle_pattern_img_index]
    cv.imshow("circle_pattern", circle_pattern_img)
    cv.pollKey()

    while True:
        _, img = cap.read()
        img_raw = img.copy()

        detection_succes = True

        # Get centers of circles on the projection plane
        # 1) Detect charuco board in image
        img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        marker_corners, marker_ids, rejected_img_points = aruco_detector.detectMarkers(img_gray)
        marker_corners, marker_ids, _, _ = aruco_detector.refineDetectedMarkers(img_gray, charuco_board, marker_corners, marker_ids, rejected_img_points, intrinsic_matrix, dist_coeffs)
        if not marker_corners:
            detection_succes = False

        charuco_corners, charuco_ids, _, _ = charuco_detector.detectBoard(img_gray, markerCorners=marker_corners, markerIds=marker_ids)
        if charuco_corners is None or len(charuco_corners) < board_inner_corner_amount:
            detection_succes = False

        img = cv.drawChessboardCorners(img, board_inner_corner_size, charuco_corners, patternWasFound=detection_succes)

        # 2) Detect circle pattern in image
        _, img_binary = cv.threshold(img_gray, 220, 255, cv.THRESH_BINARY)

        retval, circle_centers = cv.findCirclesGrid(img_binary, circle_pattern_size, flags=(cv.CALIB_CB_ASYMMETRIC_GRID + cv.CALIB_CB_CLUSTERING), blobDetector=blob_detector)
        if not retval:
            detection_succes = False

        img = cv.drawChessboardCorners(img, circle_pattern_size, circle_centers, patternWasFound=retval)

        # Only if both charuco board and circle pattern are detected, we perform the following calculations
        if detection_succes:
            # 3) Match detected points with object points to find homography
            charuco_obj_points, charuco_corners = charuco_board.matchImagePoints(charuco_corners, charuco_ids)
            
            H, _ = cv.findHomography(charuco_corners, charuco_obj_points)

            # 4) Calculate object points of circles based on homography found with charuco board
            circle_centers_proj_plane = cv.perspectiveTransform(circle_centers, H)

        cv.imshow("camera", img)
        key = cv.pollKey()

        if key == 13 and detection_succes: # ENTER
            # Add collected data to arrays
            all_centers_cam.append( np.array(circle_centers, np.float32) )
            all_centers_proj_plane.append( np.array( [[point[0, 0], point[0, 1], 0] for point in circle_centers_proj_plane] , np.float32) )
            all_centers_proj.append( np.array(circle_pattern_centers[circle_pattern_img_index], np.float32) )

            # Save img to folder to check what images are good
            cv.imwrite("./Images/Projector Calibration/School/Detection/detection_" + str(len(all_centers_proj_plane)) + "_p" + str(circle_pattern_img_index) + ".png", img)
            cv.imwrite("./Images/Projector Calibration/School/Raw/raw_" + str(len(all_centers_proj_plane)) + "_p" + str(circle_pattern_img_index) + ".png", img_raw)

            # Debug print
            print("Images collected: " + str(len(all_centers_proj_plane)))

        elif key == 110: # n
            # Cycle thru pattern images
            if circle_pattern_img_index < len(circle_pattern_images) - 1:
                circle_pattern_img_index += 1
            else:
                circle_pattern_img_index = 0

            circle_pattern_img = circle_pattern_images[circle_pattern_img_index]
            cv.imshow("circle_pattern", circle_pattern_img)
            # cv.pollKey()

        elif key == 99: # c
            break

        elif key == 113:  # q
            cap.release()
            cv.destroyAllWindows()
            exit()

    # Intrinsic calibration of projector # TODO kijken naar mogelijk flags die ik kan gebruiken
    retval_intrinsic, proj_intrinsic_matrix, proj_dist_coeffs, _, _ = cv.calibrateCamera(all_centers_proj_plane, all_centers_proj, circle_pattern_img_size, None, None)
    if not retval_intrinsic:
        print("Could not calibrate projector...")
        return

    save_intrinsic_calibration("./Calibration/projSchool", proj_intrinsic_matrix, proj_dist_coeffs)

    # Extrinsic calibration of camera projector system
    retval_extrinsic, _, _, _, _, R, T, _, _ = cv.stereoCalibrate(all_centers_proj_plane, all_centers_cam, all_centers_proj, intrinsic_matrix, dist_coeffs, proj_intrinsic_matrix, proj_dist_coeffs, circle_pattern_img_size)
    if not retval_extrinsic:
        print("Could not stereo calibrate camera projector system...")
        return

    extrinsic_matrix = make_transformation_matrix(R, T)
    
    save_extrinsic_calibration("./Calibration/extrinsicSchool", extrinsic_matrix)

    # Debug prints
    print("\n")
    print("---- intrinsic calibration ----")
    print("reprojectionError:")
    print(retval_intrinsic)
    print("projMatrix:")
    print(proj_intrinsic_matrix)
    print("projDistCoeffs:")
    print(proj_dist_coeffs)
    print("-------------------------------")
    print("---- extrinsic calibration ----")
    print("reprojectionError:")
    print(retval_extrinsic)
    print("extrinsic_matrix:")
    print(extrinsic_matrix)
    print("-------------------------------")

    return

if __name__ == "__main__":
    main()