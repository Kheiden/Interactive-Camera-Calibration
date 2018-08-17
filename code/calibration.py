import threading
import time
import glob
import cv2
import os
import numpy as np

from PIL import Image

class Calibration():

    def __init__(self):

        self.home_dir = "/home/pi"
        self.end_thread = False
        self.rightFrame = None
        self.leftFrame = None

        self.checkerboard = (6,9)

        self.wait_for_picture = False

    def start(self):
        """
        Start the application
        """
        print("Loading config...")
        # TODO Load config
        thread = threading.Thread(group=None, target=self.threaded_camera_stream, name="threaded_camera_stream")
        print("Starting threaded_camera_stream")
        thread.start()

        while True:
            print("Press 't' to take picture")
            print("Press 'q' to quit")
            var = input("> ")
            if var == 'q':
                print("Closing threads and quitting application...")
                self.end_thread = True
                break
            if var == 't':
                self.wait_for_picture = True
                while True:
                    if self.wait_for_picture == False:
                        break
                    time.sleep(0.1)

                timestamp = time.time()


                gray = cv2.cvtColor(self.rightFrame, cv2.COLOR_BGR2GRAY)
                # Find the chess board corners
                ret_right, _ = cv2.findChessboardCorners(gray, self.checkerboard, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)

                gray = cv2.cvtColor(self.leftFrame, cv2.COLOR_BGR2GRAY)
                # Find the chess board corners
                ret_left, _ = cv2.findChessboardCorners(gray, self.checkerboard, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)


                if ret_left == False:
                    print("No Chessboard Detected: Left")
                    print("Try Taking another photo")
                    continue
                if ret_right == False:
                    print("No Chessboard Detected: Right")
                    print("Try Taking another photo")
                    continue

                jpg_image_r = Image.fromarray(self.rightFrame)
                jpg_image_l = Image.fromarray(self.leftFrame)

                jpg_image_r = jpg_image_r.convert('RGB')
                jpg_image_l = jpg_image_l.convert('RGB')

                print("Saving photo to disk: '/home/pi/RPi-tankbot/local/frames/camera_{}_right.jpg'".format(timestamp))
                jpg_image_r.save("/home/pi/RPi-tankbot/local/frames/camera_{}_right.jpg".format(timestamp), format='JPEG')

                print("Saving photo to disk: '/home/pi/RPi-tankbot/local/frames/camera_{}_left.jpg'".format(timestamp))
                jpg_image_l.save("/home/pi/RPi-tankbot/local/frames/camera_{}_left.jpg".format(timestamp), format='JPEG')

                while True:
                    # I want to calibrate the left and right cameras every time
                    # I take a new stereo photo
                    print("Calibrating left camera...")
                    self.calibrate_camera(cam_num=0, res_x=640, res_y=480)
                    print("Calibrating right camera...")
                    self.calibrate_camera(cam_num=1, res_x=640, res_y=480)
                    print("Calibrating stereo camera pair...")

                    output = self.calibrate_stereo_cameras(res_x=640, res_y=480)
                    if output != False:
                        # This means that the left and right photos were successful in
                        # calibrating the stereo camera pair with the other photos
                        # TODO: Implement below code:
                        # self.coverage_map(chessboard_coordinates)
                        break
                    #Remove last two images from glob
                    images = sorted(glob.glob('{}/RPi-tankbot/local/frames/camera_*.jpg'.format(self.home_dir)))
                    #print("This should be in order:")
                    #for i in images:
                    #    print(i)
                    for filename in images[-2:]:
                        #print("Deleting file:", filename)
                        os.remove(filename)

                    if len(images) == 0:
                        print("Take more chessboard images.")
                    break
                continue

        return False

    def threaded_camera_stream(self):
        res_x = 640
        res_y = 480

        right = cv2.VideoCapture(1)
        right.set(cv2.CAP_PROP_FRAME_WIDTH, res_x)
        right.set(cv2.CAP_PROP_FRAME_HEIGHT, res_y)
        right.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        left = cv2.VideoCapture(0)
        left.set(cv2.CAP_PROP_FRAME_WIDTH, res_x)
        left.set(cv2.CAP_PROP_FRAME_HEIGHT, res_y)
        left.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        # First start the camera stream thread
        counter = 0
        while True:
            if self.end_thread == True:
                print("Stopping threaded_camera_stream")
                break
            counter += 1
            #print("Taking photo number", counter)
            ret_right = right.grab()
            ret_left = left.grab()
            self.wait_for_picture = True
            _, self.rightFrame = right.retrieve()
            _, self.leftFrame = left.retrieve()
            self.wait_for_picture = False

            if ret_left == False:
                print("Problem taking left photo")
            if ret_right == False:
                print("Problem taking right photo")

            #imgRGB_right=cv2.cvtColor(self.rightFrame, cv2.COLOR_BGR2RGB)
            #imgRGB_left=cv2.cvtColor(self.leftFrame, cv2.COLOR_BGR2RGB)

            imgRGB_combined = np.concatenate((self.leftFrame, self.rightFrame), axis=1)
            #jpg_image = Image.fromarray(imgRGB_combined)

            # TODO: Current problem is that the image does not show properly
            # when tested.
            cv2.imshow('combined_image', imgRGB_combined)
            # Only need below to ensure the video stream happens.
            # Loop will be controlled by self.end_thread
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        right.release()
        left.release()
        cv2.destroyAllWindows()
        return False


    def calibrate_stereo_cameras(self, res_x=640, res_y=480):
        # We need a lot of variables to calibrate the stereo camera
        """
        Based on code from:
        https://gist.github.com/aarmea/629e59ac7b640a60340145809b1c9013
        """
        processing_time01 = cv2.getTickCount()
        objectPoints = None

        rightImagePoints = None
        rightCameraMatrix = None
        rightDistortionCoefficients = None

        leftImagePoints = None
        leftCameraMatrix = None
        leftDistortionCoefficients = None

        rotationMatrix = None
        translationVector = None

        imageSize= (res_x, res_y)
        #imageSize = (1920, 1080)

        TERMINATION_CRITERIA = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
        OPTIMIZE_ALPHA = 0.25

        # TODO- Implement a cache for the stereopair calibration (if possible)
        #try:
        #    npz_file = np.load('{}/calibration_data/{}p/stereo_camera_calibration.npz'.format(self.home_dir, res_y))
        #    processing_time02 = cv2.getTickCount()
        #    processing_time = (processing_time02 - processing_time01)/ cv2.getTickFrequency()
        #    return processing_time
        #except:
        #    pass

        for cam_num in [0, 1]:
            right_or_left = ["_right" if cam_num==1 else "_left"][0]

            try:
                npz_file = np.load('{}/calibration_data/{}p/camera_calibration{}.npz'.format(self.home_dir, res_y, right_or_left))

                list_of_vars = ['map1', 'map2', 'objpoints', 'imgpoints', 'camera_matrix', 'distortion_coeff']

                if sorted(list_of_vars) == sorted(npz_file.files):
                    print("{} camera calibration data has been found in cache.".format(right_or_left[1:]))
                    map1 = npz_file['map1']
                    map2 = npz_file['map2']
                    objectPoints = npz_file['objpoints']
                    if right_or_left == "_right":
                        rightImagePoints = npz_file['imgpoints']
                        rightCameraMatrix = npz_file['camera_matrix']
                        rightDistortionCoefficients = npz_file['distortion_coeff']
                    if right_or_left == "_left":
                        leftImagePoints = npz_file['imgpoints']
                        leftCameraMatrix = npz_file['camera_matrix']
                        leftDistortionCoefficients = npz_file['distortion_coeff']
                else:
                    print("Camera data file found but data corrupted.")
            except:
                #If the file doesn't exist
                print("Camera calibration data not found in cache.")
                return False


        print("Calibrating cameras together...")

        leftImagePoints = np.asarray(leftImagePoints, dtype=np.float64)
        rightImagePoints = np.asarray(rightImagePoints, dtype=np.float64)

        print(len(objectPoints), len(leftImagePoints), len(rightImagePoints))
        print(objectPoints.shape, leftImagePoints.shape, rightImagePoints.shape)
        print(objectPoints.dtype, leftImagePoints.dtype, rightImagePoints.dtype)

        try:
            (RMS, _, _, _, _, rotationMatrix, translationVector) = cv2.fisheye.stereoCalibrate(
                    objectPoints, leftImagePoints, rightImagePoints,
                    leftCameraMatrix, leftDistortionCoefficients,
                    rightCameraMatrix, rightDistortionCoefficients,
                    imageSize, None, None,
                    cv2.CALIB_FIX_INTRINSIC, TERMINATION_CRITERIA)
        except Exception as e:
            # Unable to calibrate stereo pair.
            print("Error calibrating stereo pair!", e)
            return False

        print("Root Means Squared:", RMS)

        print("Rectifying cameras...")
        R1 = np.zeros([3,3])
        R2 = np.zeros([3,3])
        P1 = np.zeros([3,4])
        P2 = np.zeros([3,4])
        Q = np.zeros([4,4])

        (leftRectification, rightRectification, leftProjection, rightProjection,
                dispartityToDepthMap) = cv2.fisheye.stereoRectify(
                        leftCameraMatrix, leftDistortionCoefficients,
                        rightCameraMatrix, rightDistortionCoefficients,
                        imageSize, rotationMatrix, translationVector,
                        0, R2, P1, P2, Q,
                        cv2.CALIB_ZERO_DISPARITY, (0,0) , 0, 0)

        print("Saving calibration...")
        leftMapX, leftMapY = cv2.fisheye.initUndistortRectifyMap(
                leftCameraMatrix, leftDistortionCoefficients, leftRectification,
                leftProjection, imageSize, cv2.CV_16SC2)
        rightMapX, rightMapY = cv2.fisheye.initUndistortRectifyMap(
                rightCameraMatrix, rightDistortionCoefficients, rightRectification,
                rightProjection, imageSize, cv2.CV_16SC2)

        np.savez_compressed('{}/calibration_data/{}p/stereo_camera_calibration.npz'.format(self.home_dir, res_y), imageSize=imageSize,
                leftMapX=leftMapX, leftMapY=leftMapY,
                rightMapX=rightMapX, rightMapY=rightMapY)

        processing_time02 = cv2.getTickCount()
        processing_time = (processing_time02 - processing_time01)/ cv2.getTickFrequency()
        return processing_time


    def calibrate_camera(self, cam_num=0, res_x=640, res_y=480):
        """
        cam_num 0 is left and 1 is right.

        Code sample based on:
        http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html
        and
        https://medium.com/@kennethjiang/calibrate-fisheye-lens-using-opencv-333b05afa0b0
        """
        # Arrays to store object points and image points from all the images.
        processing_time01 = cv2.getTickCount()
        right_or_left = ["_right" if cam_num==1 else "_left"][0]


        subpix_criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
        calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW

        objp = np.zeros( (self.checkerboard[0]*self.checkerboard[1], 1, 3) , np.float64)
        objp[:,0, :2] = np.mgrid[0:self.checkerboard[0], 0:self.checkerboard[1]].T.reshape(-1, 2)

        _img_shape = None
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.
        """
        TODO:
        if the file exists, then load it
        If the file doesn't exist, then calibrate the cameras and save result to file
        """

        images = glob.glob('{}/RPi-tankbot/local/frames/camera_*{}.jpg'.format(self.home_dir, right_or_left))

        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        for index, file_name in enumerate(images):
            print(index, file_name)
            img = cv2.imread(file_name)
            if _img_shape == None:
                _img_shape = img.shape[:2]
            else:
                assert _img_shape == img.shape[:2], "All images must share the same size."

            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, self.checkerboard, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)

            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)
                cv2.cornerSubPix(gray,corners,(3,3),(-1,-1),subpix_criteria)
                imgpoints.append(corners)
            else:
                print("Error! couldn't find chessboard corner in below file!!")
                print(file_name)
                print("Removing file '{}'".format(file_name))
                os.remove(file_name)

        # Opencv sample code uses the var 'grey' from the last opened picture
        N_OK = len(objpoints)
        DIM= (res_x, res_y)
        K = np.zeros((3, 3))
        D = np.zeros((4, 1))
        rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
        tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]

        try:
            rms, camera_matrix, distortion_coeff, _, _ = \
                cv2.fisheye.calibrate(
                    objpoints,
                    imgpoints,
                    gray.shape[::-1],
                    K,
                    D,
                    rvecs,
                    tvecs,
                    calibration_flags,
                    (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
                )
        except:
            print("Camera calibration Failed")
            processing_time02 = cv2.getTickCount()
            processing_time = (processing_time02 - processing_time01)/ cv2.getTickFrequency()
            return processing_time
        print("Root Means Squared:", rms)

        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
        np.savez('{}/calibration_data/{}p/camera_calibration{}.npz'.format(self.home_dir,  res_y, right_or_left),
            map1=map1, map2=map2, objpoints=objpoints, imgpoints=imgpoints,
            camera_matrix=camera_matrix, distortion_coeff=distortion_coeff)

        # Starting from here if cache is found...
        processing_time02 = cv2.getTickCount()
        processing_time = (processing_time02 - processing_time01)/ cv2.getTickFrequency()
        return processing_time


if __name__ == '__main__':
    c = Calibration()
    c.start()
