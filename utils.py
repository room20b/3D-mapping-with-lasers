try:
    from PIL import Image
    import cv2
    import numpy as np
except ImportError:
    pass

def compress_list(lst, bins=10, as_int=0, ignore_negative=True):
    """
    Averages a list of numbers into smaller bins.
    """
    new_lst = []
    chunk_size = int(round(len(lst)/float(bins)))
    for bin in xrange(bins):
        samples = lst[bin*chunk_size:bin*chunk_size+chunk_size]
        if ignore_negative:
            samples = [_ for _ in samples if _ >= 0]
            if not samples:
                samples = [-1]
        new_lst.append(sum(samples)/float(len(samples)))
    if as_int:
        new_lst = [int(round(_)) for _ in new_lst]
    return new_lst

def only_red(im):
    """
    Strips out everything except red.
    """
    im = im.convert('RGBA')
    data = np.array(im)
    red, green, blue, alpha = data.T
    im2 = Image.fromarray(red.T)
    return im2
def only_red2(img):
    output_img = img.copy()
    img_hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_red = np.array([0,0,150])
    upper_red = np.array([255,255,255])
    mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

    #lower_red = np.array([0,0,150])
    #upper_red = np.array([255,255,255])
    #mask1 = cv2.inRange(img_hsv, lower_red, upper_red)

    #mask = mask0+mask1

    #output_img[np.where(mask==0)] = 0
    return mask0
def only_red3(img):
    return cv2.inRange(img,np.array([17,15,100]),np.array([50,56,200]))
def normalize(arr):
    """
    Linear normalization
    http://en.wikipedia.org/wiki/Normalization_%28image_processing%29
    """
    arr = arr.astype('float')
    # Do not touch the alpha channel
    for i in range(3):
        minval = arr[...,i].min()
        maxval = arr[...,i].max()
        if minval != maxval:
            arr[...,i] -= minval
            arr[...,i] *= (255.0/(maxval-minval))
    return arr
