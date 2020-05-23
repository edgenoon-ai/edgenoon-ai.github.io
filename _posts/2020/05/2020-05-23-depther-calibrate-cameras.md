---
layout: single
title: "Depther project - part 2: calibrate dual camera, parameters rectification"
author: bartoszptak
excerpt: "This article shows how to perform dual camera calibration using cv2.calibrateCamera when we have data collected. Next, we discuss how to calibrate cameras with each other using cv2.stereoRectify."
modified: 2020-05-23
tags: [opencv, dual camera, calibration]
category: [signal-processing]
image: "assets/images/posts/2020/05/depther-calibrate-cameras/Checkerboard.png"
---

### Previous parts
*  **[Depther project - part 1: collect calibration data](../depther-collect-calibration-data)**

This article shows how to perform dual camera calibration using cv2.calibrateCamera when we have data collected. Next, we discuss how to calibrate cameras with each other using cv2.stereoRectify. This article is rich in links to OpenCV documentation. It is very well written and describes well how algorithms work.

# Checkboard corners
Initially, we need to find the inside corners of the chessboard ([`cv2.findChessboardCorners`](https://docs.opencv.org/master/d9/d0c/group__calib3d.html#ga93efa9b0aa890de240ca32b11253dd4a)). The image should be grayscale.
```python
cb_shape = (7, 6)

corners = cv2.findChessboardCorners(
            img, cb_shape, cv2.CALIB_CB_FAST_CHECK)[1]
```
Next, we will use the [`cv2.cornerSubPix`](https://docs.opencv.org/master/dd/d1a/group__imgproc__feature.html#ga354e0d7c86d0d9da75de9b9701a9a87e) function, which will allow us to determine the position of corners more precisely. The typical size of `winSize`, or 'Half of the side length of the search window' is `(11,11)`.

```python
criteria = (cv2.TERM_CRITERIA_EPS +
                     cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

cv2.cornerSubPix(img, corners, (11, 11), (-1, -1), criteria)
```
If you would like to visualize points (as in the photo in the header), just use the [`cv2.drawChessboardCorners`](https://docs.opencv.org/master/d9/d0c/group__calib3d.html#ga6a10b0bb120c4907e5eabbcd22319022) function and display the image with `cv2.imshow` or save using `cv2.imwrite`.

```python
cv2.drawChessboardCorners(img, cb_shape, corners, ret)
```

# Calibrate each camera
Use [`cv2.calibrateCamera`](https://docs.opencv.org/master/d9/d0c/group__calib3d.html#ga3207604e4b1a1758aa66acb6ed5aa65d) to find the internal parameters of the left and right cameras. In this case, both cameras use the corners of the chessboard visible from the left camera.

The returned values of interest to us are: 3x3 floating-point camera matrix and vector of distortion coefficients.

```python
imageSize = (1280, 720)

matrix_left, distortion_left = cv2.calibrateCamera(
            left_obj, left_points, imageSize, None, None)[1:3]

matrix_right, distortion_right = cv2.calibrateCamera(
            left_obj, right_points, imageSize, None, None)[1:3]
```

# Calibrates a stereo camera set up

The [`cv2.stereoCalibrate`](https://docs.opencv.org/master/d9/d0c/group__calib3d.html#ga91018d80e2a93ade37539f01e6f07de5) function estimates the transformation between two cameras making a stereo pair.

As a result, we get the external parameters of the right camera in the left system. We are only interested in translation and rotation matrices, so I use the list slicing `[5:7]`.

```python
rotation_matrix, translation_matrix = cv2.stereoCalibrate(
            left_obj, left_points, right_points,
            matrix_left, distortion_left,
            matrix_right, distortion_right,
            self.imageSize, flags=cv2.CALIB_FIX_INTRINSIC, criteria=self.term)[5:7]
```

# Computes rectification transforms
Computes rectification transforms ([`cv2.stereoRectify`](https://docs.opencv.org/master/d9/d0c/group__calib3d.html#ga617b1685d4059c6040827800e72ad2b6)) for each head of a calibrated stereo camera.
```python
rect_left, rect_right, \
proj_left, proj_right, \
dispartity, \
ROI_left, ROI_right = cv2.stereoRectify(
            matrix_left, distortion_left,
            matrix_right, distortion_right,
            imageSize, rot_matrix, trans_vector,
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=-1)
```

# Results

* [rotation matrix](https://en.wikipedia.org/wiki/Rotation_matrix)
```text
[
    [ 0.999970,  0.000521,  0.007599],
    [-0.000438,  0.999940, -0.010859],
    [-0.007604,  0.010855,  0.999912]
]
```

* [translation matrix](https://en.wikipedia.org/wiki/Transformation_matrix)
```text
[
 [-0.069396, -6.556753, -0.000429]
]
```
We can calculate the [Euclidean distance](https://en.wikipedia.org/wiki/Euclidean_distance) between cameras. The left camera is at the point (0,0,0), so the calculations are simplified and we get the result: `6.557120`.

* dispartity matrix
```text
[
    [1.0,  0.0,  0.0, -528.562538],
    [0.0,  1.0,  0.0, -435.739891],
    [0.0,  0.0,  0.0, 1310.538592],
    [0.0,  0.0,  0.0,   14.409730],
]
```

# Full code
All the code, fragments of which have been discussed, can be found at [this link](https://github.com/bartoszptak/Depther/blob/master/2_calibrate.py#L123). This is the 2nd article from the 3-part project.