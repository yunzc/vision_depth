# vision_depth

Project for MIT 6.179 C/C++ class. The goal is to take two images of the same scene and the transforms of the camera from the pose when taking image 1 and the pose when taking image 2 to infer the depth of the scenes. To solutions are provided: one uses SIFT detect and match features, then find the depth of these points; the other is quite slow and performs pixel matching to create a depth map of the whole scene. Sample testing on a few opencv features and the code to calibrate the camera for obtaining the camera matrix is included in the package. 

## Getting Started 

Have OpenCV installed and basic required c++ installed. The project uses cmake to build the package. 

## Building the Package
```
cmake . 
make
```

## Running sift_img_depth (find depth of a few feature points)

```
./sift_img_depth <img1> <img2> <translation_x> <translation_z> <angle difference>
```
With included images, to see example, run: 
```
./sift_img_depth images/im1.jpg images/im2.jpg 10 0 0
```
Note that z is the depth direction (pointing towards scene). System is right handed and the transformation is from image 1 to image 2. 

After running a code, two images should show up: one is the original image (in grayscale), the other is the original with the depth points marked, darker means object is closer to camera, whiter means further away. 

## Running image_depth (return depth map a full image)

```
./image_depth <img1> <img2> <translation_x> <translation_z> <angle difference> 
```
With included images, try 
```
./image_depth images/dim1.jpg images/dim2.jpg 10 0 0
```
Note that here we are using the downsampled images, so have to downsample accordingly (see below) (using downsampled version other wise takes a long time to compute). You can see an example result at images/example_depth.jpg

## Note 

Note that in the images directory, imx.jpg are none downscaled images, so make sure the camera_params is calibrated accordingly for these, vice versa for dimx.jpg, the downscaled images. To make sure of calibration, go to calibration_config.xml and make sure line 19 is set correctly 
```
<Input>"images/calib_images.xml"</Input>
``` 
It should be either images/calib_images.xml or images/calib_downscale_images.xml
