"""This file uses De Casteljau's algorithm to find the trajectory from 12 control points.
 Ref: https://javascript.info/bezier-curve
 """

import time
import math
import numpy as np

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
        # self.S_st_i=0       #Stance 
        # self.S_sw_i=0       #Swing
        self.x=0            #coordinates initialized
        self.y=0
        self.z=0
        self.femur = 12.
        self.tibia= 11.5
        self.hip=0.
        self.bodyHeight = 18.
        self.clr_height = 4.
        self.lateral_fraction = 0
        
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
        
    def Binomial(n, k):
        return math.factorial(n)/(math.factorial(k) * math.factorial(n-k))

    def BernSteinPoly(self,n, t, i, p):
        return self.Binomial(n, i) * ((1-t)**(n - i)) * (t**i) * p

    def legIK(self, x, y, z):
        R = math.sqrt(x**2 + y**2 + z**2)
        R1 = math.sqrt(z**2 + y**2)
        theta1 = math.atan2(-z,y)
        theta2 = math.acos(self.hip/R1)
        alpha = (theta2-theta1) * 180/math.pi
        R2=math.sqrt(R**2-self.hip**2)
        phi1=math.asin(x/R2)
        temp=(self.femur**2 + R2**2 - self.tibia**2)/(2*self.femur * R2)
        
        if temp>1:  # breach condition
            temp=1
        if temp<-1:
            temp=-1
        phi2=math.acos(temp)
        beta = (phi1-phi2)*180/math.pi

        temp2 = (self.femur**2 + self.tibia ** 2 - R2**2)/(2*self.femur * self.tibia)
        if temp2>1:
            temp2=1

        if temp2<-1:
            temp2=-1

        psi=math.acos(temp2)
        gamma = (math.pi-psi)*180/math.pi
        return np.array([alpha,beta,gamma])

    def lateralMotion(lateral_fraction, x):
        X_POLAR = np.cos(lateral_fraction*(math.pi/2))
        Y_POLAR = np.sin(lateral_fraction*(math.pi/2))
    
        stepX = x * X_POLAR
        stepY = x * Y_POLAR
    
        return stepX, stepY

    def processing(self):
        t = 0.  # clock
        t_TD_ref = t    # for touchdown reference 
        legNum = 1
        for i in np.linspace(0, 1, 20, endpoint = True): #no. of cycles
            S_st_i = 0    # Stance
            S_sw_i = 0    # Swing
            print('Cycle no: ' + str(i+1))
            TD = False    #touchdown
            while(not TD):
                # if precision<loopTime:
                #     precision=loopTime
                # time.sleep(precision-(0.75*loopTime))
                loopStart=time.perf_counter()
                t_elapse_ref = t - t_TD_ref
                print(t_elapse_ref)
                TD = False
                # -------------------------------------------------
                # dS = gaitTransition(t, t_gt, dT_gt, dS_trot, dS_pace)

                # ---------------------------------------------------
                if t_elapse_ref >= self.T_stride:
                    t_elapse_ref = 0
                    TD = True

                if TD:
                    t_TD_ref = t
                    print('\nTouchDown\n')

                # print(t_elapse_ref)
                t_i = t_elapse_ref - self.T_stride * np.array(dS)
                # print(t_i)
        
                for legTime in t_i:
                    print('Leg no: ' + str(legNum))
            
                    """NOTE: PAPER WAS MISSING THIS LOGIC!!
                    This is to avoid a phase discontinuity if the user selects a Step Length and Velocity combination that causes Tstance > Tswing.
                    """
                    if legTime < -self.T_sw: #old stance phase active
                        legTime += self.T_stride
                        print('slow stance case')
                        #time updated to current stance phase, triggers next 'if'
            
                    if legTime>0 and legTime < self.T_st: #current stance phase active
                        x=0
                        y=0
                        z=0
                        S_st_i = legTime/self.T_st
                        self.phase[legNum-1] = S_st_i
                        x = self.L_span*(1-2*S_st_i) + 0
                        z = delta * (math.cos(math.pi*x/(2 * self.L_span)) + 0)- self.bodyHeight #this 0 is Pox and 18 Poy
                        x,y = self.lateralMotion(self.lateral_fraction,x)
                        # y+=helix
                        print('in current stance: '+ str(S_st_i))
                        print('Coords :'+'('+str(x)+', '+str(y)+', '+str(z)+')')
                        print('Angles :', end="")
                        angles = self.legIK(x, y, z)
                        print(angles)
                
                
                    if legTime >= -self.T_sw and legTime <= 0: #old swing phase active
                        x=0
                        y=0
                        z=0
                        S_sw_i = (legTime+self.T_sw)/self.T_sw
                        self.phase[legNum-1]=S_sw_i
                        for index in range(0,12):
                            x += self.BernSteinPoly(S_sw_i, 11, index, self.bezierControlPoints[index][0])
                            z += self.BernSteinPoly(S_sw_i, 11, index, self.bezierControlPoints[index][1])
                        x,y = self.lateralMotion(self.lateral_fraction,x)
                        # y+=helix
                        print('in old swing: '+ str(S_sw_i))
                        print('Coords :'+'('+str(x)+', '+str(y)+', '+str(z)+')')
                        print('Angles :', end="")
                        angles = self.legIK(x, y, z)
                        print(angles)
                
                    elif legTime >= self.T_st and legTime <= self.T_stride: #current swing phase active
                        x=0
                        y=0
                        z=0
                        S_sw_i = (legTime - self.T_st)/ self.T_sw
                        self.phase[legNum-1]=S_sw_i
                        for index in range(0,12):
                            x += self.BernSteinPoly(S_sw_i, 11, index, self.bezierControlPoints[index][0])
                            z += self.BernSteinPoly(S_sw_i, 11, index, self.bezierControlPoints[index][1])
                        x,y = self.lateralMotion(self.lateral_fraction,x)
                        # y+=helix
                        print('in current swing: '+ str(S_sw_i))
                        print('Coords :'+'('+str(x)+', '+str(y)+', '+str(z)+')')
                        print('Angles :', end="")
                        angles = self.legIK(x, y, z)
                        print(angles)

                    if legNum==1: #change this no. to change leg
                        # view3d.drawPoints3d([x+20,y+15,z])
                        val0 = self.angles[0]*math.pi/180
                        val1 = self.angles[1]*math.pi/180
                        val2 = self.angles[2]*math.pi/180
                        self.alpha_list.append(val0)
                        self.beta_list.append(val1)
                        self.gamma_list.append(val2)
                        self.x_list1.append(x)
                        self.z_list1.append(z)
                        # M1.set_position(val1)
                        # M2.set_position(val2)
                        
                    if legNum==2: #change this no. to change leg
                        # view3d.drawPoints3d([x+20,y-15,z])
                        val0 = self.angles[0]*math.pi/180
                        val1 = self.angles[1]*math.pi/180
                        val2 = self.angles[2]*math.pi/180
                        self.alpha_list2.append(val0)
                        self.beta_list2.append(val1)
                        self.gamma_list2.append(val2)
                        self.x_list2.append(x)
                        self.z_list2.append(z)
                
                
                    if legNum==3: #change this no. to change leg
                        # view3d.drawPoints3d([x-20,y+15,z])
                        val0 = self.angles[0]*math.pi/180
                        val1 = self.angles[1]*math.pi/180
                        val2 = self.angles[2]*math.pi/180
                        self.alpha_list3.append(val0)
                        self.beta_list3.append(val1)
                        self.gamma_list3.append(val2)
                        self.x_list3.append((float(x) + 40))
                        self.z_list3.append(z)
            
            
                    if legNum==4: #change this no. to change leg
                        # view3d.drawPoints3d([x-20,y-15,z])
                        val0 = self.angles[0]*math.pi/180
                        val1 = self.angles[1]*math.pi/180
                        val2 = self.angles[2]*math.pi/180
                        self.alpha_list4.append(val0)
                        self.beta_list4.append(val1)
                        self.gamma_list4.append(val2)
                        self.x_list4.append((float(x)+40))
                        self.z_list4.append(z)
                
                    legNum += 1
                    print('----this leg done----\n')
                legNum = 1
                print('********this time instant done for all legs**********')
                t_elapse_ref += self.precision
                t += self.precision
                loopEnd=time.perf_counter()
                loopTime=loopEnd-loopStart
                # helix+=0.1
            print('\n========================this cycle done============================\n')
        print('\n\nTotal time taken: '+str(time.perf_counter()- start)+' sec')

    if __name__ == "__main__":
        pass