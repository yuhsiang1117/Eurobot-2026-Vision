## Frames

* **World Origin (world frame):**
    * This is a fixed, arbitrary (0,0,0) point for your environment. You decide where it is.
    * In your code, you defined it as the reference point for all your `WORLD_MARKERS`. For example, you decided marker `20` is at `{0.6f, 1.4f, 0.0f}` relative to this world origin.
    * Think of it as the "origin" of your map.

* **Camera Origin (camera_color_optical_frame):**
    * This is the center of the camera's sensor. It's the camera's *own* (0,0,0) point.
    * The camera is **always** at the origin of its own frame.


## Camera matrix: 要校正的相機參數
### What the Numbers Inside Mean

The camera matrix (often called $K$) is a 3x3 matrix that holds four essential numbers:

$$
K = \begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{bmatrix}
$$



* **$f_x$ and $f_y$ (Focal Length):**
    * This is the "zoom" of your lens, measured in **pixels**.
    * It describes how strongly the camera magnifies the scene. A telephoto lens would have a very large focal length, while a fisheye lens would have a small one.
    * (They are separate numbers because pixels aren't always perfectly square).

* **$c_x$ and $c_y$ (Principal Point):**
    * This is the *exact center* of your 2D image sensor, measured in **pixels**.
    * You might think this is just (width/2, height/2), but it's almost never perfect. It's usually off by a few pixels. This is the precise optical center where light rays converge.

---

### Why It's "Crucial"

This matrix is the key that unlocks the 3D-to-2D projection.

Let's say you have a 3D point in the world, at coordinates $(X, Y, Z)$ relative to your camera. You want to know which pixel $(u, v)$ it will land on in your image.

The (simplified) formula is:

* $u = f_x \cdot (X / Z) + c_x$
* $v = f_y \cdot (Y / Z) + c_y$

**Breaking this down:**

* **$(X / Z)$ and $(Y / Z)$:** This is basic perspective. An object twice as far away ($Z$ is doubled) will appear half as big.
* **$f_x$ and $f_y$:** This scales the point. A "zoomed-in" lens (big $f_x$) makes the $u$ coordinate larger (moves it further from the center).
* **$c_x$ and $c_y$:** This shifts the point. It makes sure the center of the 3D world $(0, 0, Z)$ lands on the *actual* optical center ($c_x, c_y$) of your sensor, not at the top-left corner (pixel 0,0).