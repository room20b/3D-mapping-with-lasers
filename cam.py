import numpy as np
import cv2
from PIL import Image
from PIL import ImageFilter
from PIL.ImageChops import difference
from math import pi, tan, atan
import utils
import time
import serial
import sys

#import matplotlib.pyplot as plt

class mapper(object):
    def __init__(self, **kwargs):
        self.vert_fov_deg = float(kwargs.pop('vert_fov_deg', 41.41))
        self.horz_fov_deg = float(kwargs.pop('horz_fov_deg', 53.50))
        self.rpc = float(kwargs.pop('rpc',0.00036))
        self.ro = float(kwargs.pop('ro', 0.00978))
        self.h = float(kwargs.pop('h', 50))
        self.filter_outliers = bool(kwargs.pop('filter_outliers', True))
        self.outlier_filter_threshold = float(kwargs.pop('outlier_filter_threshold', 2.3))
        self.blur_radius = int(kwargs.pop('blur_radius', 2))
        self.laser_position = kwargs.pop('laser_position', 'bottom')
        self.normalize_brightness = kwargs.pop('normalize_brightness', False)
        self.rotations = int(kwargs.pop('rotations',5))
        self.time = float(kwargs.pop('time',1000.0))
        self.cap = cv2.VideoCapture(1)
        _,frame = self.cap.read()
        self.height,self.width,channels = frame.shape
        print(self.width,self.height)
        self.map = np.zeros((self.rotations,self.width),dtype=float)
        self.ser = serial.Serial()
        self.ser.port = '/dev/tty.HC-06-DevB'
        #ser.port = "/dev/ttyS2"
        self.ser.baudrate = 9600
        self.ser.bytesize = serial.EIGHTBITS #number of bits per bytes
        self.ser.parity = serial.PARITY_NONE #set parity check: no parity
        self.ser.stopbits = serial.STOPBITS_ONE #number of stop bits
        #ser.timeout = None          #block read
        self.ser.timeout = 1            #non-block read
        #ser.timeout = 2              #timeout block read
        self.ser.xonxoff = False     #disable software flow control
        self.ser.rtscts = False     #disable hardware (RTS/CTS) flow control
        self.ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
        self.ser.writeTimeout = 2     #timeout for write
    def dist_calc(self,diff_img):
    #def dist_calc(self,on_img,off_img):
        #if self.normalize_brightness:
        #    on_img = Image.fromarray(utils.normalize(np.array(on_img)).astype('uint8'), 'RGB')
        #    off_img = Image.fromarray(utils.normalize(np.array(off_img)).astype('uint8'), 'RGB')
        #on_img = utils.only_red(on_img)
        #off_img = utils.only_red(off_img)
        #diff_img = difference(on_img, off_img)

        if self.blur_radius:
            diff_img = diff_img.filter(ImageFilter.GaussianBlur(radius=self.blur_radius))
        x = diff_img.convert('L')
        #x.show()
        y = np.asarray(x.getdata(), dtype=np.float64).reshape((x.size[1], x.size[0]))
        laser_measurements = [0]*self.width # [row]
        laser_brightness = [0]*self.width # [brightness]
        final_measurements = [-1]*self.width # [brightest row]
        for col_i in range(y.shape[1]):
            col_max = max([(y[row_i][col_i], row_i) for row_i in range(y.shape[0])])
            col_max_brightness, col_max_row = col_max
            laser_measurements[col_i] = col_max_row
            laser_brightness[col_i] = col_max_brightness
        #plt.subplot(211)
        #plt.bar(range(len(laser_measurements)-80),laser_measurements[:-80])
        #print(np.asarray(laser_measurements))
        if self.filter_outliers:
            brightness_std = np.std(laser_brightness)
            brightness_mean = np.mean(laser_brightness)
            outlier_level = brightness_mean - brightness_std * self.outlier_filter_threshold
        for col_i, col_max_row in enumerate(laser_measurements):
            if self.filter_outliers==False or (self.filter_outliers and laser_brightness[col_i] > outlier_level):
                final_measurements[col_i] = col_max_row

        D_lst = []
        for laser_row_i in final_measurements:
            if laser_row_i < 0:
                D_lst.append(laser_row_i)
            else:
                pfc = abs(laser_row_i - self.height/2)
                theta = self.rpc * pfc + self.ro
                if theta!=0:
                    D = self.h/tan(theta)
                else:
                    D = -1
                D_lst.append(D)
        return np.asarray(laser_measurements)
    def mapz(self):
        self.ser.open()
        if self.ser.isOpen():
            self.ser.flushInput()
            self.ser.flushOutput()
            for i in range(0,self.rotations):
                self.ser.write(b'1\r')
                time.sleep(0.5)
                print("starting ",i)
                a = time.time()
                while(self.ser.inWaiting()==0):
                    if(time.time()-a>2):
                        print('hahahahahhahahhahaha stuck at starting lmao')
                        self.ser.write(b'1\r')
                        break
                a = time.time()
                argh = self.ser.read()
                print(argh)
                if(argh != b'2'):
                    break
                time.sleep(0.5)
                _,cv_on = self.cap.read()
                cv2.imwrite("images/on"+str(i)+".jpg",cv_on)
                print("on taken")

                while(self.ser.inWaiting()==0):
                    pass
                print('time elapslsed ',str(time.time()-a))
                a = time.time()
                argh = self.ser.read()
                print(argh)
                if(argh != b'3'):
                    break


                time.sleep(0.5)
                _,cv_off = self.cap.read()
                cv2.imwrite("images/off"+str(i)+".jpg",cv_off)
                print("off taken")
                diff = cv2.absdiff(cv_on,cv_off)

                red_diff = utils.only_red2(diff)
                red_diff_pil = Image.fromarray(diff)
                cv2.imwrite("images/diff"+str(i)+".jpg",diff)
                cv2.imwrite("images/red_diff"+str(i)+".jpg",red_diff)
                print("red_diff_pil")
                print(cv_off.shape)
                dist = np.asarray(self.dist_calc(red_diff_pil))
                print(dist.shape)
                self.map[i] = dist
                print(sum(dist)/len(dist))
                print("map")

                while(self.ser.inWaiting()==0):
                    pass
                print('time elapslsed ',str(time.time()-a))
                argh = self.ser.read()
                print(argh)
                if(argh != b'4'):
                    break
                time.sleep(0.5)
                #time.sleep(self.time/1000)
                #print self.dist_calc(pil_on,pil_off)

        self.cap.release()
        cv2.destroyAllWindows()
        #print self.map
        np.savetxt("images/zzz.csv",self.map,delimiter=",")
        return self.map

def calibrate():
    ##calibration
    on = cv2.imread('on_c.jpg',0)
    pil_on = Image.fromarray(on)
    off = cv2.imread('off_c.jpg',0)
    pil_off = Image.fromarray(off)
    dists = []
#0024,-056
#00114,0154

    for rpc in np.linspace(0.0005,0.001,num=10):
        for ro in np.linspace(0.02,0.035,num=10):
            print(rpc,ro)
            mz = mapper(rpc=rpc,ro=ro,h=50)
            dist = mz.dist_calc(pil_on,pil_off)
            dists.append([rpc,ro,sum(dist)/len(dist)])

    np.savetxt("dists3.csv",np.asarray(dists),delimiter=",")
#calibrate()
def main(argv):
    time.sleep(10)
    print(argv[0])
    q = mapper(rotations=int(argv[0]))
    dist_mat = q.mapz()
fff='''
on = cv2.imread('images/on0.jpg')
off = cv2.imread('images/off0.jpg')
diff = cv2.absdiff(on,off)
red_on = utils.only_red2(diff)
#cv2.imshow('red1',cv2.cvtColor(on, cv2.COLOR_BGR2HSV))
#cv2.imshow('red2',red_on)
#cv2.imshow('red3',diff)
#cv2.imshow('on',on)
#cv2.waitKey()
#cv2.destroyAllWindows()



diff_img = Image.fromarray(red_on)
#pil_on = Image.fromarray(on)


#red_off = utils.only_red2(off)
#pil_off = Image.fromarray(off)

mapper = mapper()
print(mapper.rpc, mapper.ro)
#dist = mapper.dist_calc(pil_on,pil_off)
dist = mapper.dist_calc(diff_img)
print(sum(dist)/len(dist))
#plt.subplot(212)
#plt.bar(range(len(dist)-80),dist[:-80])
#plt.show()'''

if __name__=='__main__':
    main(sys.argv[1:])