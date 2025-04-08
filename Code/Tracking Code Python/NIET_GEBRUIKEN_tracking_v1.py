import cv2 as cv
import numpy as np
import open3d as o3d

# ------------ o3d code ------------

def init_visualizer(mesh_path, camera_matrix, extrinsic_matrix, width=1920, height=1080):
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=width, height=height, visible=True)  
    # visible=False -> headless mode so main thread not used for open3d gui

    # load mesh and add to 3d env
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    mesh.compute_vertex_normals()
    mesh.translate(-mesh.get_center()) # set position of model to world origin
    vis.add_geometry(mesh)

    # add origin and axis visualization to 3d env
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)  # De grootte kun je aanpassen
    coordinate_frame.translate(-coordinate_frame.get_center())

    print("coordinate_frame.get_center(): ")
    print(coordinate_frame.get_center())

    vis.add_geometry(coordinate_frame)

    # change background color of 3d env to black
    render_option = vis.get_render_option()
    render_option.background_color = np.asarray([0.0, 0.0, 0.0])  # black -> RGB = [0, 0, 0]

    # default camera of 3d env
    # view_control = vis.get_view_control()
    # view_control.set_front([0, 0, -1])
    # view_control.set_lookat([0, 0, 0])
    # view_control.set_up([0, 1, 0])
    # view_control.set_zoom(0.5)

    # replace default camera with pinhole camera model so intrinsic and extrinsic parameters can be applied
    apply_camera_parameters(vis, camera_matrix, extrinsic_matrix, width, height)

    # update 3d env by calling these functions   
    vis.poll_events()
    vis.update_renderer()

    return vis, mesh

def update_pose(vis, mesh, pose_matrix, previous_pose):
    # mesh.transform moves mesh from current pose as reference while pose_matrix is defined with world origin as reference
    # -> save the previous transformation matrix, 
    #    combine new pose with inverted of previous pose to get total transformation
    # -> apply new transforamtion matrix to mesh to update pose correctly 
    pose = np.dot(pose_matrix, np.linalg.inv(previous_pose))

    mesh.transform(pose)

    # update mesh and 3d env
    vis.update_geometry(mesh)
    vis.poll_events()
    vis.update_renderer()

    # return pose_matrix as previous_pose
    return pose_matrix

def get_o3d_viewer_image(vis):
    # retrieve img from virtual camera
    img = vis.capture_screen_float_buffer(do_render=True)
    img = np.asarray(img)
    img = (img * 255).astype(np.uint8)
    img = cv.cvtColor(img, cv.COLOR_RGB2BGR)
    return img

def apply_camera_parameters(vis, intrinsic_matrix, extrinsic_matrix, width=1920, height=1080):
    # make camera object
    cam_params = o3d.camera.PinholeCameraParameters()

    # apply intrinsic parameters
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(width=width, height=height,
                             fx=intrinsic_matrix[0, 0], fy=intrinsic_matrix[1, 1],
                             cx=intrinsic_matrix[0, 2], cy=intrinsic_matrix[1, 2])
    cam_params.intrinsic = intrinsic

    # apply extrinsic parameters
    cam_params.extrinsic = extrinsic_matrix

    # print("cam_params.extrinsic:")
    # print(cam_params.extrinsic)
    print("cam_params.intrinsic.get_principal_point():")
    print(cam_params.intrinsic.get_principal_point())

    # set new camera as view control of 3d env
    # https://github.com/isl-org/Open3D/issues/1164#:~:text=in%20current%20version%2C%20you%20should%20set%20the%20parameter%20allow_arbitrary%3DTrue%2C%20i.e.%2C%0Aview_control.convert_from_pinhole_camera_parameters(cam_params%2C%20allow_arbitrary%3DTrue)
    vis.get_view_control().convert_from_pinhole_camera_parameters(cam_params, allow_arbitrary=True)

    # TODO check if needed
    # vis.get_view_control().set_zoom(0.8)

    print(vis.get_view_control().convert_to_pinhole_camera_parameters().extrinsic)

# ------------ o3d code ------------



# ------------ init code ------------

def load_camera_calibration(filename):
    fs = cv.FileStorage(filename, cv.FILE_STORAGE_READ)
    cameraMatrix = fs.getNode("cameraMatrix").mat()
    distCoeffs = fs.getNode("distCoeffs").mat()
    fs.release()
    return cameraMatrix, distCoeffs

def get_board_center(obj_points):
    obj_points = np.vstack(obj_points)
    board_center = np.mean(obj_points, axis=0)
    return board_center.reshape(3, 1)

# ------------ init code ------------



# ------------ matrix manipulation code ------------

def convert_tvec_to_board_center(rvec, tvec, detected_obj_points, board_center):
    detected_obj_points_center = np.mean(detected_obj_points, axis=0).reshape(3, 1)

    R, _ = cv.Rodrigues(rvec)

    tvec_corrected = tvec + R @ (board_center - detected_obj_points_center)

    return tvec_corrected

def make_transformation_matrix(rvec, tvec):
    R, _ = cv.Rodrigues(rvec)

    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = R
    transformation_matrix[:3, 3] = tvec.flatten()

    return transformation_matrix

# TODO check if needed
def convert_pose_cv_to_o3d(pose):
    # coordinate system o3d: https://github.com/isl-org/Open3D/issues/6508
    cv_to_o3d = np.array([
        [1,  0,  0,  0],
        [0, -1,  0,  0],
        [0,  0, -1,  0],
        [0,  0,  0,  1]
    ])

    return cv_to_o3d @ pose

def correct_for_o3d(intrinsic, extrinsic):
    cx = intrinsic[0, 2]  # principle point x in pixels
    cy = intrinsic[1, 2]  # principle point y in pixels
    fx = intrinsic[0, 0]  # focal length x
    fy = intrinsic[1, 1]  # focal length y

    # Transformeer het principle point naar camera-ruimte (genormaliseerd)
    principle_offset = np.array([(cx - intrinsic.shape[1] / 2) / fx, 
                                 (cy - intrinsic.shape[0] / 2) / fy, 
                                 0, 1])  # Z = 0 want het is een offset

    # Projecteer naar wereldruimte
    translation_correction = extrinsic[:3, :3] @ principle_offset[:3]

    # Pas de translatie in de extrinsieke matrix aan
    extrinsic[:3, 3] -= translation_correction

    return extrinsic

# ------------ matrix manipulation code ------------

def main():
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
    board_center = get_board_center(board.getObjPoints())

    # camera parameters, TODO: replace generic camera matrix with projector intrinsic parameters
    # camera_matrix, dist_coeffs = load_camera_calibration("./cameraPhone")
    camera_matrix, dist_coeffs = load_camera_calibration("./cameraHome")
    generic_camera_matrix = np.array([
        [600, 0, 640 / 2], 
        [0, 600, 480 / 2], 
        [0, 0, 1]
    ], np.float32)

    # save board image
    # img_size = 1000
    # board_img = board.generateImage((img_size, img_size))
    # cv.imwrite("board.png", board_img)

    extrinsic = np.eye(4)
    # extrinsic = correct_for_o3d(generic_camera_matrix, extrinsic)
    extrinsic = convert_pose_cv_to_o3d(extrinsic)
    # extrinsic = np.linalg.inv(extrinsic)
    # visualizer, mesh = init_visualizer("./House_Assembly_v3.obj", generic_camera_matrix, extrinsic)
    visualizer, mesh = init_visualizer("./Test.obj", generic_camera_matrix, extrinsic, 640, 480)

    previous_pose = np.eye(4)

    cap = cv.VideoCapture(0)

    while True:
        # read img, convert to grayscale
        _, img = cap.read()
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # detect markers
        corners, ids, rejectedImgPoints = aruco_detector.detectMarkers(gray)
        detectedCorners, detectedIds, _, _ = aruco_detector.refineDetectedMarkers(gray, board, corners, ids, rejectedImgPoints, camera_matrix, dist_coeffs)

        cv.aruco.drawDetectedMarkers(img, detectedCorners, detectedIds)
        
        # show img with detected markers
        cv.imshow("img", img)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

        if detectedIds is None or len(detectedIds) == 0:
            continue

        # TODO test: estimate pose of aruco board
        obj_points, img_points = board.matchImagePoints(detectedCorners, detectedIds)
        retval, rvec, tvec = cv.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)
        if not retval:
            continue

        # TODO test
        tvec_corrected = convert_tvec_to_board_center(rvec, tvec, obj_points, board_center) # convert tvec to center of board 
        transformation_matrix = make_transformation_matrix(rvec, tvec_corrected) # combine rvec and tvec into 4x4 transformation_matrix

        # TODO check if needed: convert opencv axis convention to visualizer xyz convention
        pose = convert_pose_cv_to_o3d(transformation_matrix)

        # print("transformation_matrix:")
        # print(transformation_matrix)
        # print("pose: ")
        # print(pose)
        
        previous_pose = update_pose(visualizer, mesh, pose, previous_pose)

        # get image from viewer and display
        o3d_img = get_o3d_viewer_image(visualizer)
        scale = 1
        o3d_img = cv.resize(o3d_img, (int(o3d_img.shape[1]*scale), int(o3d_img.shape[0]*scale)))
        cv.imshow("o3d_img", o3d_img)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

        
    # close cap and o3d visualizer
    visualizer.destroy_window()

    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()