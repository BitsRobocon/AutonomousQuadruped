"""This file uses De Casteljau's algorithm to find the trajectory from 12 control points.
 Ref: https://javascript.info/bezier-curve
 """

import time
import math
import numpy as np
import plotView3d as view3d
import fourLegSimulator as fls

class BezierPoints():
    def __init__(self):
        self.L_span = 8.    #half of stride length
        self.phase = np.array([0.,0.,0.,0.]) #FL, FR, BL, BR   change later
        self.dt = 0.05  # timestamp
        self.NumControlPoints = 12
        self.v_d = 0.7 #10/11 #m/s
        self.T_st = 2 * self.L_span/(100 * self.v_d) # Stance time(0.18)
        self.T_sw = 0.15 # Swing time(sec)
        self.T_stride = self.T_st + self.T_sw #0.33
        self.precision=0.01  
        self.delta = -0.85

        self.t1=0
        self.t2=0
        self.t3=0
        self.t4=0
        self.t_elapse_ref=0.0 #change this value between 0 and 0.33(stride time)

        self.t_i = np.array([self.t1,self.t2,self.t3,self.t4])

        dS_gallop =[0,0.2,0.55,0.75]#gallop
        dS_trot =[0,0.5,0.5,0]#trot
        dS_walk =[0,0.5,0.75,0.25]#walk- 0.5 and 0.75 flipped, looks correct
        dS_bound =[0,0,0.5,0.5]# bound
        dS_pace =[0,0.5,0,0.5] # pace

        self.dS = []
        self.dS = dS_walk
        self.S_st_i = 0       #Stance 
        self.S_sw_i = 0       #Swing
        self.x = 0            #coordinates initialized
        self.y = 0
        self.z = 0
        self.femur = 12.
        self.tibia = 11.5
        self.hip = 0.
        self.bodyHeight = 18.
        self.clr_height = 4.
        self.lateral_fraction = 0.
        
        self.bezierControlPoints=np.array([[-self.L_span,0.0 - self.bodyHeight],
                     [-self.L_span*1.4,0.0 - self.bodyHeight],
                     [-self.L_span*1.5,self.clr_height*0.9-self.bodyHeight],
                     [-self.L_span*1.5,self.clr_height*0.9-self.bodyHeight],
                     [-self.L_span*1.5,self.clr_height*0.9-self.bodyHeight],
                     [0.0,self.clr_height*0.9-self.bodyHeight],
                     [0.0,self.clr_height*0.9-self.bodyHeight],
                     [0.0,self.clr_height*1.157-self.bodyHeight],
                     [self.L_span*1.5,self.clr_height*1.157-self.bodyHeight],
                     [self.L_span*1.5,self.clr_height*1.157-self.bodyHeight],
                     [self.L_span*1.4,0.0-self.bodyHeight],
                     [self.L_span,0.0-self.bodyHeight]])

        self.angles=[]
        self.alpha_list=[]
        self.beta_list=[]
        self.gamma_list=[]
        self.alpha_list2=[]
        self.beta_list2=[]
        self.gamma_list2=[]
        self.alpha_list3=[]
        self.beta_list3=[]
        self.gamma_list3=[]
        self.alpha_list4=[]
        self.beta_list4=[]
        self.gamma_list4=[]
        self.x_list1 = []
        self.z_list1 = []
        self.x_list2 = []
        self.z_list2 = []
        self.x_list3 = []
        self.z_list3 = []
        self.x_list4 = []
        self.z_list4 = []
        
    def Binomial(self,n, k):
        return math.factorial(n)/(math.factorial(k) * math.factorial(n-k))

    def BernSteinPoly(self,t, n, i, p):
        return self.Binomial(n, i) * ((1-t)**(n - i)) * (t**i) * p

    def legIK(self, x, y, z):
        R = math.sqrt(x**2 + y**2 + z**2)
        R1 = math.sqrt(z**2 + y**2)
        theta1 = math.atan2(-z,y)
        theta2 = math.acos(self.hip/R1)
        alpha = (theta2-theta1) * 180/math.pi
        R2 = math.sqrt(R**2-self.hip**2)
        phi1 = math.asin(x/R2)
        temp = (self.femur**2 + R2**2 - self.tibia**2)/(2*self.femur * R2)
        
        if temp > 1:  # breach condition
            temp = 1
        if temp < -1:
            temp = -1
        phi2 = math.acos(temp)
        beta = (phi1-phi2)*180/math.pi

        temp2 = (self.femur**2 + self.tibia ** 2 - R2**2)/(2*self.femur * self.tibia)
        if temp2 > 1:
            temp2 = 1

        if temp2 < -1:
            temp2 = -1

        psi = math.acos(temp2)
        gamma = (math.pi-psi)*180/math.pi
        return np.array([alpha,beta,gamma])

    def lateralMotion(self, lateral_fraction, x):
        X_POLAR = np.cos(lateral_fraction*(math.pi/2))
        Y_POLAR = np.sin(lateral_fraction*(math.pi/2))
    
        stepX = x * X_POLAR
        stepY = x * Y_POLAR
    
        return stepX, stepY

    def processing(self):
        view3d.drawPoints3d([0,0,0])
        view3d.drawCurve3d([[22,15,0],[-22,15,0],[-22,-15,0],[22,-15,0],[22,15,0]])
        t = 0.0 #clock started
        t_TD_ref = t #initialized from TouchDown
        start = time.perf_counter()   
        legNum = 1
        for i in range(0,1):#np.linspace(0, 1, 20, endpoint = True): #no. of cycles
            self.S_st_i = 0    # Stance
            self.S_sw_i = 0    # Swing
            print('Cycle no: ' + str(i+1))
            TD = False    #touchdown
            while(not TD):
                loopStart = time.perf_counter()
                self.t_elapse_ref = t - t_TD_ref
                print(self.t_elapse_ref)
                TD = False

                if self.t_elapse_ref >= self.T_stride:
                    t_elapse_ref = 0
                    TD = True

                if TD:
                    t_TD_ref = t
                    print('\nTouchDown\n')

                self.t_i = self.t_elapse_ref - self.T_stride * np.array(self.dS)
        
                for legTime in self.t_i:
                    print('Leg no: ' + str(legNum))
            
                    """NOTE: PAPER WAS MISSING THIS LOGIC!!
                    This is to avoid a phase discontinuity if the user selects a Step Length and Velocity combination that causes Tstance > Tswing.
                    """
                    if legTime < -self.T_sw: #old stance phase active
                        legTime += self.T_stride
                        print('slow stance case')
                        #time updated to current stance phase, triggers next 'if'
            
                    if legTime>0 and legTime < self.T_st: #current stance phase active
                        self.x=0
                        self.y=0
                        self.z=0
                        self.S_st_i = legTime/self.T_st
                        self.phase[legNum-1] = self.S_st_i
                        self.x = self.L_span*(1 - 2*self.S_st_i) + 0
                        self.z = self.delta * (math.cos(math.pi * self.x/(2 * self.L_span)) + 0)- self.bodyHeight #this 0 is Pox and 18 Poy
                        self.x, self.y = self.lateralMotion(self.lateral_fraction, self.x)
                        # y+=helix
                        print('in current stance: '+ str(self.S_st_i))
                        print('Coords :'+'('+str(self.x)+', '+str(self.y)+', '+str(self.z)+')')
                        print('Angles :', end="")
                        self.angles = self.legIK(self.x, self.y, self.z)
                        print(self.angles)
                
                
                    if legTime >= -self.T_sw and legTime <= 0: #old swing phase active
                        self.x=0
                        self.y=0
                        self.z=0
                        self.S_sw_i = (legTime+self.T_sw) / self.T_sw
                        self.phase[legNum-1] = self.S_sw_i
                        for index in range(0, 12):
                            self.x += self.BernSteinPoly(self.S_sw_i, 11, index, self.bezierControlPoints[index][0])
                            self.z += self.BernSteinPoly(self.S_sw_i, 11, index, self.bezierControlPoints[index][1])
                        self.x, self.y = self.lateralMotion(self.lateral_fraction, self.x)
                        # y+=helix
                        print('in old swing: '+ str(self.S_sw_i))
                        print('Coords :'+'('+str(self.x)+', '+str(self.y)+', '+str(self.z)+')')
                        print('Angles :', end="")
                        self.angles = self.legIK(self.x, self.y, self.z)
                        print(self.angles)
                
                    elif legTime >= self.T_st and legTime <= self.T_stride: #current swing phase active
                        self.x=0
                        self.y=0
                        self.z=0
                        self.S_sw_i = (legTime - self.T_st)/ self.T_sw
                        self.phase[legNum-1]=self.S_sw_i
                        for index in range(0,12):
                            self.x += self.BernSteinPoly(self.S_sw_i, 11, index, self.bezierControlPoints[index][0])
                            self.z += self.BernSteinPoly(self.S_sw_i, 11, index, self.bezierControlPoints[index][1])
                        self.x, self.y = self.lateralMotion(self.lateral_fraction,self.x)
                        # y+=helix
                        print('in current swing: '+ str(self.S_sw_i))
                        print('Coords :'+'('+str(self.x)+', '+str(self.y)+', '+str(self.z)+')')
                        print('Angles :', end="")
                        self.angles = self.legIK(self.x, self.y, self.z) 
                        print(self.angles)

                    if legNum == 1: #change this no. to change leg
                        view3d.drawPoints3d([self.x+20,self.y+15,self.z])
                        val0 = self.angles[0]*math.pi/180
                        val1 = self.angles[1]*math.pi/180
                        val2 = self.angles[2]*math.pi/180
                        self.alpha_list.append(val0)
                        self.beta_list.append(val1)
                        self.gamma_list.append(val2)
                        self.x_list1.append(self.x)
                        self.z_list1.append(self.z)
                        
                    if legNum == 2: #change this no. to change leg
                        view3d.drawPoints3d([self.x+20,self.y-15,self.z])
                        val0 = self.angles[0]*math.pi/180
                        val1 = self.angles[1]*math.pi/180
                        val2 = self.angles[2]*math.pi/180
                        self.alpha_list2.append(val0)
                        self.beta_list2.append(val1)
                        self.gamma_list2.append(val2)
                        self.x_list2.append(self.x)
                        self.z_list2.append(self.z)
                
                
                    if legNum == 3: #change this no. to change leg
                        view3d.drawPoints3d([self.x-20,self.y+15,self.z])
                        val0 = self.angles[0]*math.pi/180
                        val1 = self.angles[1]*math.pi/180
                        val2 = self.angles[2]*math.pi/180
                        self.alpha_list3.append(val0)
                        self.beta_list3.append(val1)
                        self.gamma_list3.append(val2)
                        self.x_list3.append((float(self.x) + 40))
                        self.z_list3.append(self.z)
            
            
                    if legNum == 4 : #change this no. to change leg
                        view3d.drawPoints3d([self.x-20,self.y-15,self.z])
                        val0 = self.angles[0]*math.pi/180
                        val1 = self.angles[1]*math.pi/180
                        val2 = self.angles[2]*math.pi/180
                        self.alpha_list4.append(val0)
                        self.beta_list4.append(val1)
                        self.gamma_list4.append(val2)
                        self.x_list4.append((float(self.x)+40))
                        self.z_list4.append(self.z)
                
                    legNum += 1
                    print('----this leg done----\n')
                legNum = 1
                print('********this time instant done for all legs**********')
                self.t_elapse_ref += self.precision
                t += self.precision
                loopEnd = time.perf_counter()
                loopTime = loopEnd-loopStart
                # helix+=0.1
            print('\n========================this cycle done============================\n')
        print('\n\nTotal time taken: '+str(time.perf_counter()- start)+' sec')
        fls.fourLegSimulator(self.beta_list, self.gamma_list, self.beta_list2, self.gamma_list2, self.beta_list3, self.gamma_list3, self.beta_list4, self.gamma_list4, self.bodyHeight, self.femur, self.tibia)

if __name__ == "__main__":
    bleh = BezierPoints()
    bleh.processing()