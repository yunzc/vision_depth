# vision_depth

Project for MIT 6.179 C/C++ class. The goal is to take two images of the same scene and the transforms of the camera from the pose when taking image 1 and the pose when taking image 2 to infer the depth of the scenes. To solutions are provided: one uses SURF to find the depth of a few detected features, the other is quite slow and performs pixel matching to create a depth map of the whole scene. Sample testing on a few opencv features and the code to calibrate the camera for obtaining the camera matrix is included in the package. 

## Getting Started 

Have OpenCV installed and basic required c++ installed. The project uses cmake to build the package. 

## Building the Package
'''
cmake . 
make
'''

## Running 

For SURF (depth of a few features): 
'''
./process_img <img1> <img2> <translation_x> <translation_z> <angle difference>
'''
With included images, to see example, run: 
 '''
./process_img images/im1.jpg images/im2.jpg -30 0 0
'''
Note that z is the depth direction (pointing towards scene). System is right handed and the transformation is from image 1 to image 2. 
