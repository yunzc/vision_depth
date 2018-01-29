# vision_depth

to make: 
cmake .
make

Usage: ./process_img "img1" "img2" "translation_x" "translation_z" "angle difference" 

Z is the depth direction (pointing towards scene). System is right handed. 

Note that transformation is from image 1 to image 2. 
