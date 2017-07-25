import numpy
cimport numpy

from cython.operator import dereference, postincrement
from libcpp cimport bool as cpp_bool
from libcpp.map cimport map as std_map
from libc.string cimport memcpy

numpy.import_array()

# These are in cvdef.h via opencv2/core.hpp in ocv3
cdef extern from "opencv2/core/types_c.h":
    cdef int CV_8UC1
    cdef int CV_8UC3
    cdef int CV_8SC1
    cdef int CV_8SC3
    cdef int CV_16UC1
    cdef int CV_16UC3
    cdef int CV_16SC1
    cdef int CV_16SC3
    cdef int CV_32SC1
    cdef int CV_32SC3
    cdef int CV_32FC1
    cdef int CV_32FC3
    cdef int CV_64FC1
    cdef int CV_64FC3

cdef extern from "opencv2/core/core.hpp" namespace "cv":

    cdef cppclass Scalar:
        Scalar()
        Scalar(double, double, double, double)
    
    cdef cppclass Mat:
        int rows
        int cols
        unsigned char *data
        
        Mat()
        Mat(int, int, int, void *) # , const size_t *steps=0)
        int channels()
        cpp_bool isContinuous()
        int type()

    # Instantiate template, cython doesn't do typed template params:
    # --> class Foo<int> {} <--  doesn't work
    # so, create a class that is the instance we need to use
    cdef cppclass Matx66uc "cv::Matx<unsigned char, 6, 6>":
        Matx()
        unsigned char val[]
    
cdef extern from "chilitags.hpp" namespace "chilitags":
    cdef cppclass Quad:
        Quad()
        float val[]
    ctypedef std_map[int, Quad] TagCornerMap
    
    cdef cppclass Chilitags:
        Chilitags()
        void setFilter(int, float)
        TagCornerMap find(const Mat &, DetectionTrigger)
        void setDetectionPeriod(int)
        void setPerformance(PerformancePreset preset)
        void setCornerRefinement(cpp_bool)
        void setMaxInputWidth(int)
        void setMinInputWidth(int)
        Matx66uc encode(int) const
        int decode(const Matx66uc &bits) const
        Mat draw(int, int, cpp_bool, Scalar)
    
    ctypedef enum DetectionTrigger "chilitags::Chilitags::DetectionTrigger":
        TRACK_AND_DETECT "chilitags::Chilitags::TRACK_AND_DETECT"
        DETECT_ONLY "chilitags::Chilitags::DETECT_ONLY"
        TRACK_ONLY "chilitags::Chilitags::TRACK_ONLY"
        DETECT_PERIODICALLY "chilitags::Chilitags::DETECT_PERIODICALLY"
        ASYNC_DETECT_PERIODICALLY "chilitags::Chilitags::ASYNC_DETECT_PERIODICALLY"
        ASYNC_DETECT_ALWAYS "chilitags::Chilitags::ASYNC_DETECT_ALWAYS"
    
    ctypedef enum PerformancePreset "chilitags::Chilitags::PerformancePreset":
        FASTER "chilitags::Chilitags::FASTER"
        FAST "chilitags::Chilitags::FAST"
        ROBUST "chilitags::Chilitags::ROBUST"

TRIGGER_TRACK_AND_DETECT = TRACK_AND_DETECT
TRIGGER_DETECT_ONLY = DETECT_ONLY
TRIGGER_TRACK_ONLY = TRACK_ONLY
TRIGGER_DETECT_PERIODICALLY = DETECT_PERIODICALLY
TRIGGER_ASYNC_DETECT_PERIODICALLY = ASYNC_DETECT_PERIODICALLY
TRIGGER_ASYNC_DETECT_ALWAYS = ASYNC_DETECT_ALWAYS

PRESET_FASTER = FASTER
PRESET_FAST = FAST
PRESET_ROBUST = ROBUST

cdef _mat_to_ndarray(const Mat &m):
    """Convert a cv::Mat to a numpy ndarray
    Currently supports only CV_8UC3 Mats
    """
    cdef int rows = m.rows
    cdef int cols = m.cols
    cdef int channels = m.channels()

    if not m.isContinuous():
        return None

    cdef int cv_type = m.type()
    cdef numpy.ndarray arr
    
    if cv_type == CV_8UC3:
        arr = numpy.empty([rows, cols, channels], dtype=numpy.uint8, order="C")
    # elif cv_type == CV_8SC3, etc. 
    else:
        raise Exception("BAD PROGRAMMER! No Cookie")
    
    memcpy(<unsigned int*> arr.data, m.data, rows * cols * channels * arr.dtype.itemsize)

    return arr

cdef Mat *_ndarray_to_mat(numpy.ndarray arr):
    """Convert a numpy array to a cv::Mat
    This returns a pointer for the moment, don't lose it. 
    """
    cdef int rows
    cdef int cols
    cdef int channels
    cdef int cv_type
    
    rows = arr.shape[0]
    cols = arr.shape[1]
    channels = arr.shape[2]
    
    # better way to check this?
    if arr.dtype == numpy.dtype('uint8') and channels == 3:
        cv_type = CV_8UC3
    else:
        raise Exception("BAD COOKIE! No programmer!")
    
    cdef Mat *mat
    mat = new Mat(rows, cols, cv_type, arr.data)
    return mat


cdef _Quad_to_ndarray(const Quad &mx):
    """Convert a Quad (aka Matx<float, 4, 2>) to an ndarray
    """
    cdef numpy.ndarray arr
    arr = numpy.empty([4, 2, 1], dtype=numpy.float32, order="C")
    memcpy(<unsigned int*> arr.data, <unsigned int *> mx.val, 4 * 2 * 1 * arr.dtype.itemsize)
    return arr

cdef _matx66uc_to_ndarray(const Matx66uc &mx):
    """Convert a Matx<unsigned char, 6, 6> to an ndarray
    """
    cdef numpy.ndarray arr
    arr = numpy.empty([6, 6, 1], dtype=numpy.uint8, order="C")
    memcpy(<unsigned int*> arr.data, <unsigned int *> mx.val, 6 * 6 * 1 * arr.dtype.itemsize)
    return arr

cdef Matx66uc _ndarray_to_matx66uc(numpy.ndarray bits):
    """Convert an ndarray to a Matx<unsigned char, 6, 6>
    """
    cdef Matx66uc mx
    memcpy(<unsigned int *> mx.val, <unsigned int*> bits.data, 6 * 6 * 1 * bits.dtype.itemsize)
    return mx

cdef class Chilipy:
    """Chilitags API wrapped in Python
    Method names have been pythonized, functionality and parameters are
    identical

    This class is the core of detection of chilitags.

    Its main function is to find tags in an image, i.e. return the id and the
    position in the image of the corners of each detected chilitag.

    It also provides some utilities, like encoding and decoding id's to/from
    bit matrices, or drawing a given tag.

    It sets a default persistence of 5 frames for the tags, and a gain of 0.
    Please refer to the documentation of set_filter() for more detail. The
    default detection performance is balanced between accuracy and processing
    time (see chilipy.PRESET_FAST); it can be changed with set_performance().
    """
    cdef Chilitags *thisptr
     
    def __cinit__(self):
        self.thisptr = new Chilitags()
     
    def __dealloc__(self):
        del self.thisptr
     
    def set_filter(self, persistence, gain):
        """Parameters to palliate with the imperfections of the detection.

        persistence -- the number of frames in which a tag should be absent
        before being removed from the output of find(). 0 means that tags
        disappear directly if they are not detected.

        gain -- a value between 0 and 1 corresponding to the weight of the
        previous (filtered) position in the new filtered position. 0 means that
        the latest position of the tag is returned.
        """
        self.thisptr.setFilter(persistence, gain)
    
    def find(self, input_image, detection_trigger=TRIGGER_DETECT_ONLY):
        """This is the main method of Chilitags. 
        
        \returns the detected tags, in
        the form of a mapping between their id's and the position of their four
        corners.

        \param inputImage an OpenCV image (gray or BGR)

        \param detectionTrigger specifies how to combine tracking and full
        detection. Tracking is drastically faster, but it can at best return
        tags previously found; it won't find new ones, but can lose some. See
        Chilitags::DetectionTrigger for a description of the possible values.
        """
        cdef Mat *image
        cdef TagCornerMap tcm
        
        image = _ndarray_to_mat(input_image)
        # note: in cython, pointers are dereferenced via subscript, ptr[0]
        tcm = self.thisptr.find(dereference(image), detection_trigger)
        
        result = {}
        cdef int key
        cdef Quad value
        cdef std_map[int, Quad].iterator i = tcm.begin()
        while i != tcm.end():
            key = dereference(i).first
            value = dereference(i).second
            result[key] = _Quad_to_ndarray(value)
            postincrement(i)
        
        del image # release cv::Mat
        
        return result
    
    def set_detection_period(self, period):
        """When the detection trigger is Chilitags::DETECT_PERIODICALLY,
        `period` specifies the number of frames between each full detection.
        The default is 15, which means that out of 15 consecutive calls to
        find(), one will use a full detection, and the 14 others will only
        track previous results.
        """
        self.thisptr.setDetectionPeriod(period)
    
    def set_performance(self, preset):
        """Applies one of the performance tuning preset (See
        PerformancePreset). To tune more finely the performance trade-offs, see
        set_corner_refinement(), set_max_input_wdith(), and
        set_min_input_width().
        """
        self.thisptr.setPerformance(preset)

    def set_corner_refinement(self, refine_corners):
        """Enable or disable the corner refinement. It is enabled (true) by
        default. When disabled, the processing time is reduced by ~33%, but the
        coordinates of the tags lose their sub-pixel precision, and there is a
        marginally higher level of false negatives.
        """
        self.thisptr.setCornerRefinement(refine_corners)
     
    def set_max_input_wdith(self, max_width):
        """Ensures that the image used as input for the detection is at most
        `max_width` wide. The smaller, the faster, but tags smaller than 20
        pixels won't be detected.

        max_width -- the width to which input images should be reduced to, or 0
        if no resizing should occur (default).
        """

        self.thisptr.setMaxInputWidth(max_width)
    
    def set_min_input_width(self, min_width):
        """Chilitags searches for tags on the input image and on subsamples
        reduced to 50%, 25%, 12.5%, etc. of the original size. The subsamples
        are reduced as long as they are at least `minWidth` wide. This value
        can be changed to adjust the lower limit of subsampling. For example,
        the chilipy.PRESET_ROBUST performance preset calls
        `setMinInputWidth(160)`.

        If `minWidth` is set to 0, subsampling is completely disabled, i.e.
        tags are searched only on the original input image. This is the
        behaviour set by chilipy.PRESET_FAST, i.e. the default behaviour.

        Disabling the subsampling reduces the processing time by ~40%, but
        large tags (having sides larger than hundreds of pixels) are likely to
        be missed.
        """
        self.thisptr.setMinInputWidth(min_width)

    def draw(self, id, cell_size=1, with_margin=False, color=(0, 0, 0)):
        """draw(id, cell_size=1, with_margin=False, color=(0, 0, 0)) ->
        Return an OpenCV image of a given tag. 
        
        Args: 
        id -- id of the tag to draw, between [0, 1024)
        
        Keyword args: 
        cell_size -- the (integer) scale factor with which to draw the tag. In
        other words, every bit of the data matrix of the tag will be `cellSize`
        large.

        with_margin -- a boolean coding whether the returned image of the tag
        should be surrounded by a white frame, ensuring that the edges of the tag
        will contrast with the background.

        color -- the RGB color with which to draw the tag. Values are integers
        within [0,255]. The darker, the better. Black is default and optimal.
        """
        # cdef double r, g, b
        # r = color[0] / 256.0
        # g = color[1] / 256.0
        # b = color[2] / 256.0
        # cdef Scalar _color = Scalar(r, g, b, 0)
        cdef Scalar _color = Scalar(color[0], color[1], color[2], 0)
        
        m = self.thisptr.draw(id, cell_size, with_margin, _color)
        arr = _mat_to_ndarray(m)
        return arr

    def encode(self, id):
        """Finds the black and white, 6x6 matrix corresponding to the given id.
        \param id the id of the tag to encode, between 0 (included) and 1024
        (excluded).
        \returns the 36-element bit matrix coding the given id
        (black is 0, white is 1) 
        """
        cdef Matx66uc result
        result = self.thisptr.encode(id)
        arr = _matx66uc_to_ndarray(result)
        return arr

    def decode(self, bits):
        """Finds the tag id corresponding given the black and white, 6x6
        matrix.

        \returns the id decoded from the bit matrix, between 0 (included) and
        1024 (excluded). If the bit matrix did not code a valid id, -1 is
        returned.
        
        \param bits the 36-element bit matrix coding the given id (black is 0,
        white is 1) 
        """
        mx = _ndarray_to_matx66uc(bits) 
        result = self.thisptr.decode(mx)
        return result
