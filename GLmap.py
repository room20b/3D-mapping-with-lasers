import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *
import math
import numpy as np

import csv

#verticies=[(2.0, 1.9999999999999998, 0.0), (1.0, 0.5773502691896256, 0.0)
#, (2.0, 0.5358983848622454, 0.0), (1.0, 0.0, 0.0), (2.0, -0.5358983848622454, 0.0), (1.0, -0.5773502691896256, 0.0)]
#val=[1]*320
#print(val)
def P2C(input_vec, theta):
	delta_phi=math.radians(100)/(len(input_vec)) #convert to radians
	output_z=[0]*len(input_vec)
	output_y=[0]*len(input_vec)
	output_x=[0]*len(input_vec)
	for i in range(len(input_vec)):
		'''output_z[i]=input_vec[i]*math.sin(theta)
		output_y[i]=input_vec[i]*math.cos(theta)*math.sin((int(len(input_vec)/2)-i)*delta_phi)
		output_x[i]=input_vec[i]*math.cos(theta)*math.cos((int(len(input_vec)/2)-i)*delta_phi)'''
		output_x[i]=input_vec[i]*math.cos(theta)
		output_y[i]=input_vec[i]*math.cos(theta)*math.tan((len(input_vec)/2-i)*delta_phi)
		#print(math.tan((len(input_vec)/2-i)*delta_phi))
		output_z[i]=input_vec[i]*math.sin(theta)
		#print(input_vec[i]*math.sin(theta))
	zipped=tuple(zip(output_y,output_z,output_x))
	#print(zipped)
	return zipped
#cartesian=P2C(val,0)
#print(cartesian)
def generatelflat(r,c):
	val=[1]*c
	output_val=[]
	k=0.0
	for i in range(r):
		temp=np.asarray(val)/math.cos(math.radians(k))
		#print(temp)
		output_val.append(temp)
		k+=1.8
	return output_val

#temp_mat=generatelflat(30,30)
#print(temp_mat)

#mat=np.ones((20,50))
#print(mat)
def mat_P2C(input_mat):
	input_mat /= 150
	mat=[]
	i=0.0
	for line in input_mat:
		#print(line)
		theta=math.radians(i)
		vec_cartesian=P2C(line,theta)
		mat.append(vec_cartesian)
		i+=1.8
	return mat
#cat_mat=mat_P2C(mat)
#cat_mat1=mat_P2C(temp_mat)

#print(cat_mat)
def Cube(cat_mat):
	glBegin(GL_POINTS)
	for line in cat_mat:
		for vertex in range(len(line)):
			#print(verticies[vertex])
			glVertex3fv(line[vertex])
	glEnd()

def axes():
	glPushMatrix()
	#clear matrix
	glLoadIdentity()
	#apply rotations
	glRotatef(1, 1.0, 0.0, 0.0)
	glRotatef(1, 0.0, 1.0, 0.0)
	glRotatef(1, 0.0, 0.0, 1.0)
	#move the axes to the screen corner
	glTranslatef(-3.0, -2.0, 0.0)
	#draw our axes
	glBegin(GL_LINES)
	#draw line for x axis
	glColor3f(1.0, 0.0, 0.0)
	glVertex3f(0.0, 0.0, 0.0)
	glVertex3f(1.0, 0.0, 0.0)
	#draw line for y axis
	glColor3f(0.0, 1.0, 0.0)
	glVertex3f(0.0, 0.0, 0.0)
	glVertex3f(0.0, 1.0, 0.0)
	#draw line for Z axis
	glColor3f(0.0, 0.0, 1.0)
	glVertex3f(0.0, 0.0, 0.0)
	glVertex3f(0.0, 0.0, 1.0)
	glEnd()
	#load the previous matrix
	glPopMatrix()

def main():
	dist = []
	f = open('images/zzz11.csv',newline='')
	l = csv.reader(f)
	n = 0
	for r in l:
		#print(len(r))
		n = n+1
		dist[len(dist):] = list(map(float,r))
	f.close()
	f_dist = np.asarray(dist).reshape(n,-1)
	print(f_dist.shape)

	cat_mat = mat_P2C(f_dist)

	pygame.init()
	display = (800,600)
	pygame.display.set_mode(display, DOUBLEBUF|OPENGL)

	gluPerspective(90, (display[0]/display[1]), 0.1, 50.0)

	glTranslatef(0.0,0.0, -5)

	while True:
		#pygame.event.pump()
		#print(pygame.key.get_pressed())
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()
				quit()
			if event.type == pygame.KEYDOWN or event.type == KEYDOWN:
				print(event)
				if event.key == pygame.K_w:
					#print('HI PRESSED W')
					glTranslatef(0,0,0.1)
				elif event.key == pygame.K_s:
					glTranslatef(0,0,-0.1)
				elif event.key == pygame.K_d:
					glTranslatef(-0.05,0,0)
				elif event.key == pygame.K_a:
					glTranslatef(0.05,0,0)
				elif event.key == pygame.K_RIGHT:
					glRotatef(-2, 0, 1, 0)
				elif event.key == pygame.K_LEFT:
					glRotatef(2, 0, 1, 0)
				elif event.key == pygame.K_UP:
					glRotatef(3, 1, 0, 0)
				elif event.key == pygame.K_DOWN:
					glRotatef(-3, 1, 0, 0)
				elif event.key == pygame.K_r:
					glRotatef(180, 0, 1, 0)
		#glTranslatef(0.01,0,0)
		#glRotatef(3, 0, 1, 0)
		glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
		Cube(cat_mat)
        #axes()
		pygame.display.flip()
		pygame.time.wait(10)


main()