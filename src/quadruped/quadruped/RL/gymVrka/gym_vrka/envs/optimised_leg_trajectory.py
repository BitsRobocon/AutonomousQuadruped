'''
This scriptwill contain a ros node which will subscribe to the position topic
for each leg and calculate as well as execute the most optimised path to be
followed - using an RL system.

Subscribers
    /leg/position < quadruped_interfaces.msg.LegPosition > # could be changed as one for each leg

Publishers
    # add motor topics here

Author
    ekanshgupta92 <ekanshgupta92@gmail.com>

'''
