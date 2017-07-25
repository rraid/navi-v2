Chilipy
=======

This is a python interface to the [Chilitags](https://github.com/chili-epfl/chilitags) library, used to detect fiducial markers in images. 

I built this so I could use Chilitags from python, and also as a learning experiment in writing Cython code, particularly code that interfaces with OpenCV.  

## Build

This requires Chilitags, OpenCV, numpy, and cython. Currently, I suspect it will only build on a linux or macosx machine. 

The build process is: 

- Build/install OpenCV
- Build chilitags
- Edit setup.py to point to the chilitags include lib 
  directories. 
- Finally, `python setup.py build`

Building is a little rough right now, I'll improve that later. 

## Usage

The API is essentially the same as the C++ library. The biggest difference is that method names have been pythonized, and methods return the appropriate python types. 

Interop with OpenCV's python module should be transparent. Images (`cv::Mat`) created by Chilitags are transparently converted between numpy ndarray's, in the same format used by the OpenCV python module. 

Note: only CV_8UC3 images are supported at the moment. 

### Examples

#### Detect tags

```
import chilipy
import cv2

# Read one of the chilitags test images
# https://github.com/chili-epfl/chilitags-testdata/blob/master/stills/640x480/nao05.jpg
image = cv2.imread('nao05.jpg') 

c = chilipy.Chilipy()
corner_map = c.find(image)

for id, corners in corner_map.items(): 
    print id
    # do something with the corners
```

#### Encode and Decode tags
```
import chilipy

c = chilipy.Chilipy()

a = c.encode(1)
c.decode(a) # returns 1
```

#### Draw tags
```
import chilipy
import cv2

c = chilipy.Chilipy()

img = c.draw(1, cell_size=8)
cv2.imshow("tag 1", img)
```
