## autonomy_human

"autonomy_human" is a [ROS](http://ros.org) package that includes the Human Robot Interaction code developed for the following paper:

> Valiallah(Mani) Monajjemi, Jens Wawerla, Richard T. Vaughan, and Greg Mori. "HRI in The Sky: Creating and 
> Commanding Teams of UAVs With a Vision-Mediated Gestural Unterface", In Proceedings of the IEEE
> International Conference on Intelligent Robots and Systems (IROS'13), Tokyo, Japan, November 2013.

In brief this package provides the following functionalities:

- Human face tracking: OpenCV based face detection combined with a Kalman filter to robustly track a single human's face. The face tracker also reports the quality of detected face, AKA as the "face score". (more info in [1])
- Optical flow calculation with stabilization to be used for hand wave gesture recognition
- Probabilistic skin segmentation for tracked user [Experimental]

Please refer to the paper for more information about the implementation details.

### Notes on Compiling

This package has external dependency on [OpenCV](http://opencv.org/) which can be handled by `rosdep`. This package also depends on ROS packages [cv_bridge](http://wiki.ros.org/cv_bridge) and [image_transport](http://wiki.ros.org/image_transport). You can alternatively use the OpenCV binary packages shipped with ROS: `ros-*-opencv2` and `ros-*-vision-opencv`. For example for ROS Groovy on Ubuntu: `sudo apt-get install ros-groovy-opencv2 ros-groovy-vision-opencv`. To compile package simply run `rosmake autonomy_human`. 

#### Execution

You need to execute the `autonomy_human` executable with appropriate parameters (described in the next section). You can look into `launch/demo_usbcam.launch` for sample configuration. 

### Subscribed Topics

- `input_rgb_image`: The input image stream of type [image transport](http://www.ros.org/wiki/image_transport).

### Published Topics

- `output_rgb_debug`, `output_rgb_optical` and `output_rgb_skin`: Debug image streams of type [image transport](http://www.ros.org/wiki/image_transport) for debug, skin and optical flow respectively.

- `human`: Information about the detected human. The type of this message is custom and is defined in `msg/human.msg` file. The details are as follows:

	- `header`: Standard ROS header.
	- `numFaces`: The number of detected faces in the current frame (regardless of tracker's stated). The type is `uint32`.
	- `faceROI`: The bounding box for the currently tracked face in pixels represented in image coordinate frame. The type is [sensor_msgs/RegionOfInterest](http://http://docs.ros.org/api/sensor_msgs/html/msg/RegionOfInterest.html). 
	- `faceScore`: The quality of the currently tracked face. Please refer to the paper for more details. The type is `uint32`.
	- `flowScore`: This `float32[2]` array represents the average optical flow for all pixels in the  left and right regions around the tracked face. These values can be used to detect human hand waves by thresholding. You may also need to use low-pass or median filters for more robust detection.

### Parameters

- `~debug_mode`: Integer value the determines debug images to be published: (Default is set to `2`)
	- bit 0: Unused
	- bit 1: Enables publishing the general debug image to `output_rgb_debug`
	- bit 2: Enables publishing the skin segmentation result to `output_rgb_skin`
	- bit 3: Unused
	- bit 4: Enables publishing the optical flow calculation result to `output_rgb_optical`

- `~cascade_file`: The absolute path to the Cascade Classifier `XML` or `YAML` file. "Frontal Face" databases from `OpenCV` is shipped with this package. Please consult the `launch/demo_usbcam.launch` file for how to use those. More information about cascade classifiers in OpenCV can be found [here](http://http://docs.opencv.org/modules/objdetect/doc/cascade_classification.html).

- `~profile_hack_enabled`: Determines if the detector should look for the `profile` faces as well as `frontal` faces. The default value is `False`.

- `~cascade_profile_file`: Similar to `~cascade_file`. Should only be set if `~profile_hack_enabled` is set to `True`

- `~skin_enabled`: Determines if probabilistic skin segmentation should be performed. The default value is `False`. If set to `True`, the pixels in the detected facial area would be first thresholded, then used to determine the human skin's histogram. This histogram is then used to determine probabilistically (using a Bayesian filter) the likelihood of each pixel in the image to be human's skin. The `output_rgb_skin` is a grayscale image created from that probability distribution.

- `~gesture_enabled`: Enables the optical flow calculation in two regions around human's face. Default is `False`.

- `~flowstablize_mode`: The flow stabilization method to use. Default is `0`. Possible values: 
	- `0`: Disabled
	- `1`: Median filter based, as described in the paper.
	- `2`: (Very Experimental), do the stabilization by calculating camera homography transformation from Optical Flow.

- `~initial_min_score`: Minimum quality of detected face to be considered as legitimate. Default is `5`.
- `~initial_detect_frames`: The minimum number of consecutive frames that the face should be detected before starting the face tracker. Default is `6`. 
- `~initial_reject_frames`: If the face is being tracked by the Kalman filter but is not be detected for this number of frames, the face is considered lost. Default is `6`.
- `~min_flow`: Value of min cut-off value for optical flow. Default is `10`. Start with smaller values.
- `~min_face_width`: Minimum acceptable width for detected face. Default is `12` pixels.
- `~min_face_height`: Minimum acceptable height for detected face. Default is `18` pixels.
- `~max_face_width`: Maximum acceptable width for detected face. Default is `60` pixels.
- `~max_face_height`: Maximum acceptable height for detected face. Default is `80` pixels.

**Note**: The bigger the range for face detector's `width` and `height`, the more computation intensive the face detection would be.

- `meas_cov` and `proc_cov`: Kalman filter parameters for measurement and process covariances. Defaults are 1.0 and 0.05 respectively.

### Notes on `demo_usbcam.launch` file

To launch this you need to install ROS packages [usb_cam](http://wiki.ros.org/usb_cam) and [image_pipeline](http://wiki.ros.org/image_pipeline). For groovy you can `sudo aptitude install ros-groovy-usb-cam ros-groovy-image-pipeline`. You may also need to modify the camera parameters for `usb_cam` for your particular camera.

### References 

[1] Alex Couture-Beil, Richard T. Vaughan, and Greg Mori. "Selecting and commanding individual robots in a vision-based multi-robot system", In Proceedings of the Canadian Conference on Computer and Robot Vision, May 2010.
