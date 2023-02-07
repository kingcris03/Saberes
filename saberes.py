from time import *
from vpython import *
from sympy import *
from math import *
from numpy import *
import cv2 as cv2
import pyautogui
from pyautogui import*
g=vector (0,-9.80655,0)
earth_radius= 6371000
Guardadito = open('Datos.txt', 'w')
g=vector (0,-9.80655,0)
earth_radius= 6371000
Guardadito = open('Datos.txt', 'w')

def caso_1():
  SCREEN_SIZE = (1920, 1080)
  fourcc = cv2.VideoWriter_fourcc(*"XVID")
  out = cv2.VideoWriter("Salida.avi", fourcc, 20.0, (SCREEN_SIZE))
  fps = 60
  prev = 0
  phi = float(input('Ingrese el angulo  PHI: '))
  theta = float(input('Ingrese el angulo THETA: '))
  v0=float(input('Ingrese la magnitud de la velocidad inicial: '))
  hdealtura=float(input('ingrese la altura inicial del sistema: '))
  masa=float(input('ingrese la masa del objeto: '))
  Record=input('Quiere grabar (Y/N): ')
  ground=box(pos=vector(0,0,0),size=vector(100,.01,100),color=color.green)
  ball=sphere(pos=vector(0,hdealtura,0),radius=.1, color=color.green, make_trail=True, fast=False)
  r0=ball.pos
  ball.m=masa
  ball.p=ball.m*v0*vector(cos(radians(theta))*sin(radians(phi)),sin(radians(theta)),cos(radians(theta))*cos(radians(phi)))
  t=0
  dt=0.001
  ######
  lista_datos = []
  graphi = graph(title='Parabolic Position', scroll=True, xmin=0, xmax=10, fast=False)
  x_plot = gcurve(graph=graphi, color=color.red, label="x")
  y_plot = gcurve(graph=graphi, color=color.blue, label="y")
  ######
  if Record == 'Y':
   while ball.pos.y>=0:
     rate(1000)
     time_transcurrido = time() - prev
     img = pyautogui.screenshot()
     prev = time()
     frame = array(img)
     frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
     out.write(frame)
     Fnet=ball.m*g
     ball.p=ball.p+Fnet*dt
     ball.pos=ball.pos+ball.p*dt/ball.m
     x_plot.plot(t,ball.pos.x)
     y_plot.plot(t,ball.pos.y)
     t=t+dt
     cv2.waitKey(1)
   cv2.destroyAllWindows()
   out.release()
   
  elif Record == 'N':
    while ball.pos.y>=0:
     rate(1000)
     Fnet=ball.m*g
     ball.p=ball.p+Fnet*dt
     ball.pos=ball.pos+ball.p*dt/ball.m
     x_plot.plot(t,ball.pos.x)
     y_plot.plot(t,ball.pos.y)
     t=t+dt



def caso_2():
  SCREEN_SIZE = (1920, 1080)
  fourcc = cv2.VideoWriter_fourcc(*"XVID")
  out = cv2.VideoWriter("Salida.avi", fourcc, 20.0, (SCREEN_SIZE))
  fps = 60
  prev = 0
  phi = float(input('Ingrese el angulo  PHI: '))
  theta = float(input('Ingrese el angulo THETA: '))
  v0=float(input('Ingrese la magnitud de la velocidad inicial: '))
  hdealtura=float(input('ingrese la altura inicial del sistema: '))
  masa=float(input('ingrese la masa del objeto: '))
  coef_air=float(input('ingrese la cantidad adimensional del coeficiente de friccion (aire): '))
  RHO=float(input('ingrese la densidad del aire: '))
  A=float(input('ingrese el area transversal del proyectil: '))
  Record=input('Quiere grabar (Y/N): ')
  ground=box(pos=vector(0,0,0),size=vector(100,.01,100),color=color.green)
  ball_2=sphere(pos=vector(0,hdealtura,0),radius=.1, color=color.green, make_trail=True)
  r1=ball_2.pos
  ball_2.m=masa
  ball_2.p=ball_2.m*v0*vector(cos(radians(theta))*sin(radians(phi)),sin(radians(theta)),cos(radians(theta))*cos(radians(phi)))
  t=0
  dt=0.001
  a_friccion=(coef_air*RHO*A)/(2*masa*cos(radians(theta)))
  vector_friccion=vector(v0*cos(radians(theta))*sin(radians(phi))*a_friccion,v0*sin(radians(theta))*a_friccion,0) 
  ######
  lista_datos = []
  graphi = graph(scroll=True, xmin=0, xmax=10, fast=False)
  x_plot = gcurve(graph=graphi, color=color.red, label="x")
  y_plot = gcurve(graph=graphi, color=color.blue, label="y")
  ######
  if Record == 'Y':
   while ball_2.pos.y>=0 :
      rate(1000)
      time_transcurrido = time() - prev
      img = pyautogui.screenshot()
      prev = time()
      frame = array(img)
      frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
      out.write(frame)
      Ftotal=(ball_2.m*g)-(ball_2.m*vector_friccion)
      ball_2.p=ball_2.p+Ftotal*dt
      ball_2.pos=ball_2.pos+ball_2.p*dt/ball_2.m
      x_plot.plot(t,ball_2.pos.x)
      y_plot.plot(t,ball_2.pos.y)
      t=t+dt
      cv2.waitKey(1)
      Guardadito.write(f'|{dt}|'+f'|{ball_2.pos.x}|' +f'|{ball_2.pos.y}|'+f'|{ball_2.pos.z}|'+f'|{t}|'+ '\n')
   cv2.destroyAllWindows()
   out.release()
  elif Record == 'N':
    while ball_2.pos.y>=0 :
      rate(1000)
      Ftotal=(ball_2.m*g)-(ball_2.m*vector_friccion)
      ball_2.p=ball_2.p+Ftotal*dt
      ball_2.pos=ball_2.pos+ball_2.p*dt/ball_2.m
      x_plot.plot(t,ball_2.pos.x)
      y_plot.plot(t,ball_2.pos.y)
      t=t+dt
      Guardadito.write(f'|{dt}|'+f'|{ball_2.pos.x}|' +f'|{ball_2.pos.y}|'+f'|{ball_2.pos.z}|'+f'|{t}|'+ '\n')

def caso_3():
  SCREEN_SIZE = (1920, 1080)
  fourcc = cv2.VideoWriter_fourcc(*"XVID")
  out = cv2.VideoWriter("Salida.avi", fourcc, 20.0, (SCREEN_SIZE))
  fps = 60
  prev = 0
  phi = float(input('Ingrese el angulo  PHI: '))
  theta = float(input('Ingrese el angulo THETA: '))
  v0=float(input('Ingrese la magnitud de la velocidad inicial: '))
  hdealtura=float(input('ingrese la altura inicial del sistema: '))
  masa=float(input('ingrese la masa del objeto: '))
  coef_air=float(input('ingrese la cantidad adimensional del coeficiente de friccion (aire): '))
  RHO=float(input('ingrese la densidad del aire: '))
  A=float(input('ingrese el area transversal del proyectil: '))
  Record=input('Quiere grabar (Y/N): ')
  ground=box(pos=vector(0,0,0),size=vector(100,.01,100),color=color.green)
  ball_3=sphere(pos=vector(0,hdealtura,0),radius=.1, color=color.green, make_trail=True)
  r1=ball_3.pos
  ball_3.m=masa
  ball_3.p=ball_3.m*v0*vector(cos(radians(theta))*sin(radians(phi)),sin(radians(theta)),cos(radians(theta))*cos(radians(phi)))
  t=0
  dt=0.001
  ######
  lista_datos = []
  graphi = graph(scroll=True, xmin=0, xmax=10, fast=False)
  x_plot = gcurve(graph=graphi, color=color.red, label="x")
  y_plot = gcurve(graph=graphi, color=color.blue, label="y")
  ######
  Gravedaaaa_text = str(input('Desea considerar efectos gravitatorios Y/N: '))
  if Record == 'N':
   if Gravedaaaa_text == 'N':
    while ball_3.pos.y>=0 :
      RHO_2=(0.0001031*ball_3.pos.y)+1.216
      a_friccion_2=(coef_air*RHO_2*A)/(2*masa*cos(radians(theta)))
      vector_friccion_2=vector(v0*cos(radians(theta))*sin(radians(phi))*a_friccion_2,v0*sin(radians(theta))*a_friccion_2,0)
      rate(1000)
      Fabs=(ball_3.m*g)-(ball_3.m*vector_friccion_2)
      ball_3.p=ball_3.p+Fabs*dt
      ball_3.pos=ball_3.pos+ball_3.p*dt/ball_3.m
      x_plot.plot(t,ball_3.pos.x)
      y_plot.plot(t,ball_3.pos.y)
      t=t+dt
      Guardadito.write(f'|{dt}|'+f'|{ball_3.pos.x}|' +f'|{ball_3.pos.y}|'+f'|{ball_3.pos.z}|'+f'|{t}|'+ '\n')

   elif Gravedaaaa_text == 'Y':
     while ball_3.pos.y>=0 :
      g1=vector(0,(g.y)*pow((earth_radius/(earth_radius+ball_3.pos.y)),2),0)
      RHO_2=(0.0001031*ball_3.pos.y)+1.216
      a_friccion_2=(coef_air*RHO_2*A)/(2*masa*cos(radians(theta)))
      vector_friccion_2=vector(v0*cos(radians(theta))*sin(radians(phi))*a_friccion_2,v0*sin(radians(theta))*a_friccion_2,0)
      rate(1000)
      Fabs=(ball_3.m*g)-(ball_3.m*vector_friccion_2)
      ball_3.p=ball_3.p+Fabs*dt
      ball_3.pos=ball_3.pos+ball_3.p*dt/ball_3.m
      x_plot.plot(t,ball_3.pos.x)
      y_plot.plot(t,ball_3.pos.y)
      t=t+dt
      Guardadito.write(f'|{dt}|'+f'|{ball_3.pos.x}|' +f'|{ball_3.pos.y}|'+f'|{ball_3.pos.z}|'+f'|{t}|'+ '\n')
  elif Record == 'Y':
    if Gravedaaaa_text == 'N':
      while ball_3.pos.y>=0 :
       time_transcurrido = time() - prev
       img = pyautogui.screenshot()
       prev = time()
       frame = array(img)
       frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
       out.write(frame)
       RHO_2=(0.0001031*ball_3.pos.y)+1.216
       a_friccion_2=(coef_air*RHO_2*A)/(2*masa*cos(radians(theta)))
       vector_friccion_2=vector(v0*cos(radians(theta))*sin(radians(phi))*a_friccion_2,v0*sin(radians(theta))*a_friccion_2,0)
       rate(1000)
       Fabs=(ball_3.m*g)-(ball_3.m*vector_friccion_2)
       ball_3.p=ball_3.p+Fabs*dt
       ball_3.pos=ball_3.pos+ball_3.p*dt/ball_3.m
       x_plot.plot(t,ball_3.pos.x)
       y_plot.plot(t,ball_3.pos.y)
       t=t+dt
       cv2.waitKey(1)
       Guardadito.write(f'|{dt}|'+f'|{ball_3.pos.x}|' +f'|{ball_3.pos.y}|'+f'|{ball_3.pos.z}|'+f'|{t}|'+ '\n')
     

    elif Gravedaaaa_text == 'Y':
      while ball_3.pos.y>=0 :
       time_transcurrido = time() - prev
       img = pyautogui.screenshot()
       prev = time()
       frame = array(img)
       frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
       out.write(frame)
       g1=vector(0,(g.y)*pow((earth_radius/(earth_radius+ball_3.pos.y)),2),0)
       RHO_2=(0.0001031*ball_3.pos.y)+1.216
       a_friccion_2=(coef_air*RHO_2*A)/(2*masa*cos(radians(theta)))
       vector_friccion_2=vector(v0*cos(radians(theta))*sin(radians(phi))*a_friccion_2,v0*sin(radians(theta))*a_friccion_2,0)
       rate(1000)
       Fabs=(ball_3.m*g)-(ball_3.m*vector_friccion_2)
       ball_3.p=ball_3.p+Fabs*dt
       ball_3.pos=ball_3.pos+ball_3.p*dt/ball_3.m
       x_plot.plot(t,ball_3.pos.x)
       y_plot.plot(t,ball_3.pos.y)
       t=t+dt
       cv2.waitKey(1)
       Guardadito.write(f'|{dt}|'+f'|{ball_3.pos.x}|' +f'|{ball_3.pos.y}|'+f'|{ball_3.pos.z}|'+f'|{t}|'+ '\n')
    cv2.destroyAllWindows()
    out.release()


caso_1()