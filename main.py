#!/usr/bin/env python
import numpy as np
#import tensorflow as tf
import datetime
import pandas as pd
import csv
import math
import matplotlib.pyplot as plt
import rospy

from Environment_Strombom import Environment_Strombom


if __name__ == '__main__':
    


    	N_EPISODE = 300	
    	max_steps = 1000

        NumberOfSheep = 100
        NeighbourhoodSize = int(NumberOfSheep/2)
        NumberOfShepherds = 1	
        env = Environment_Strombom(NumberOfSheep,NeighbourhoodSize,NumberOfShepherds)
        
        
        #Creating file to store analysis
        #Create header for the saving DQN learning file
        now = datetime.datetime.now()
        header = ["Ep","Step","Action_X","Action_Y", "Dog_X","Dog_Y","Center_X_p","Center_Y_p","Furthest_x_p","Furthest_y_p","Termination_Code"]

        filename = "Testing/stromData" +".csv"
        with open(filename, 'w') as f:
            pd.DataFrame(columns = header).to_csv(f,encoding='utf-8', index=False, header = True)
    
        for episode_i in range(N_EPISODE):
            done,s_t,case = env.reset()
            total_reward = 0.

            # number of timesteps
            for step in range(max_steps):
                a_t = env.Strombom_action(case)
                ## Compute next states, reward, and completion status
                s_t1, done,infor,case = env.step(a_t)

                #Saving data to file
                save_data = np.hstack([episode_i+1,step+1,a_t[0],a_t[1],infor[0],infor[1],infor[2],infor[3],infor[4],infor[5],done]).reshape(1,10)
                with open(filename, 'a') as f:
                    pd.DataFrame(save_data).to_csv(f,encoding='utf-8', index=False, header = False)		    	

                s_t = s_t1
                #env.view(fHandler)
                if done:
                    break

            print("TOTAL REWARD @ " + str(episode_i+1))
            print("Total Step: " + str(step+1))		
            print("Termination code: " + str(done))
            now = datetime.datetime.now()
            print("Time:" + now.strftime("%Y%m%d-%H%M"))
            print("")

		

	
		
   
    
