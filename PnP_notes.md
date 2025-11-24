## PnP
### Goal
`solvePnP` doesn't find the camera's coordinate in the world. Instead, it finds the world's coordinates from the camera's perspective

The camera is always at the origin (0,0,0) in its own coordinate system `camera_color_optical_frame`. solvePnP figures out the 3D rotation and translation of the world frame relative to the camera.

### Pipeline

#### Inputs:

1. `objectPoints`: Your list of 3D world coordinates (e.g., {0.55, 1.45, 0.0}). 已知的3D點座標

2. `imagePoints`: The matching list of 2D pixel coordinates (e.g., {152, 301}). 對應在相機上的2D座標

3. camera_matrix_: Your camera's lens info. This is crucial because it tells the algorithm how to project 3D points onto a 2D plane (the pinhole camera model).


### Flow
1. 猜現在的Rotation(R_mat), Translation(T_vec)是多少，iterate 過每一GT點看誤差
* 隨機抽樣 (Sample): 從你所有的 GT 點對 (例如 16 個角點) 中，隨機抽出 4 個。

* 建立模型 (Model): 只用這 4 個點，它會精確解出一個猜想的姿態 (R_guess, T_guess)。

* 驗證 (Check): 它會用這個 (R_guess, T_guess) 去「重新投影」所有的 16 個點，然後計算「有多少個點的投影誤差小於 3.0 像素？」。這些點稱為「inliers」。

* 重複： 重複 1a-1c 200 次，並保留那個得到最多 inliers 的 (R_guess, T_guess)。

2. solvePnP 得到的是 world frame 到 camera frame 的轉換。
* tvec (T_vec_best) 代表的是 world 的原點 (0,0,0)，在 camera 座標系中的 3D 位置。
* rvec (R_mat_best) 代表的是 world 的座標軸，在 camera 座標系中的旋轉。

3. 那麼我們就可以反推從相機認為的世界原點 tvec  計算相機真正的位置，t_mc = -R_mc^T * cv::Mat(tvec);

