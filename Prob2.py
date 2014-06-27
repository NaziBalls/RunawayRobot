__author__ = 'nagpalk'
# ----------
# Part Two
#
# Now we'll make the scenario a bit more realistic. Now Traxbot's
# sensor measurements are a bit noisy (though its motions are still
# completetly noise-free and it still moves in an almost-circle).
# You'll have to write a function that takes as input the next
# noisy (x, y) sensor measurement and outputs the best guess
# for the robot's next position.
#
# ----------
# YOUR JOB
#
# Complete the function estimate_next_pos. You will be considered
# correct if your estimate is within 0.01 stepsizes of Traxbot's next
# true position.
#
# ----------
# GRADING
#
# We will make repeated calls to your estimate_next_pos function. After
# each call, we will compare your estimated position to the robot's true
# position. As soon as you are within 0.01 stepsizes of the true position,
# you will be marked correct and we will tell you how many steps it took
# before your function successfully located the target bot.

# These import steps give you access to libraries which you may (or may
# not) want to use.
from robot import *  # Check the robot.py tab to see how this works.
from math import *
from matrix import * # Check the matrix.py tab to see how this works.
import random

# This is the function you have to write. Note that measurement is a
# single (x, y) point. This function will have to be called multiple
# times before you have enough information to accurately predict the
# next position. The OTHER variable that your function returns will be
# passed back to your function the next time it is called. You can use
# this to keep track of important information over time.
def calculate_heading_angle(measurement,OTHER):

    x3 = measurement[0]
    y3 = measurement[1]
    x1 = OTHER[0][0]
    y1 = OTHER[0][1]
    x2 = OTHER[1][0]
    y2= OTHER[1][1]

    if(x2-x1)==0:
        HA1 = 0
    else:
        HA1 = atan( (y2-y1)/(x2-x1) )

    if (x2<x1):
        HA1 += pi

    if (x3-x2)==0:
        HA2 = 0
    else:
        HA2 = atan( (y3-y2)/(x3-x2) )
    if (x3<x2):
        HA2 += pi

    delHA = (HA2 - HA1)
    if delHA < -5 :
        delHA = delHA%(2*pi)
    HA3 = (HA2+delHA)%(2*pi)

    return [HA3, delHA]


def estimate_next_pos(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""



    x3 = measurement[0]
    y3 = measurement[1]

    if(OTHER == None):
        #                                                            HA-2       r-3     x-4    y-5    count-6, theta -7
        OTHER = [measurement, (measurement[0]+1, measurement[1]+1), (0,10000), (1,10000), (x3,2000), (y3,2000), 0, (0,10000) ]

    count = OTHER[6]
    if(count>1):
        #Localize x and y  given current measurement
        [xupdated, xvarupdated] = update(x3, 1, OTHER[4][0], OTHER[4][1])
        [yupdated, yvarupdated] = update(y3, 1, OTHER[5][0], OTHER[5][1])


        #Calculate Heading Angle
        [HA3mean1, theta] = calculate_heading_angle(measurement, OTHER)
            #Update our guess at what theta is
        thetavar1 = 1.0
        [thetaupdated, thetavarupdated] = update(theta, thetavar1, OTHER[7][0], OTHER[7][1])
            #Use this guess to update our guess of HA
        HA3var1 = 1.0
        HA3mean = (OTHER[2][0] + thetaupdated) % (2*pi)
        HA3var = OTHER[2][1]
        if (abs(HA3mean-HA3mean1)<1):
            [HA3updated, HA3varupdated] = update(HA3mean1, HA3var1, HA3mean, HA3var)
        else:
            [HA3updated, HA3varupdated] = [HA3mean, HA3var]

        #Calculate Distance Travelled
        rmean1 = distance_between(measurement, OTHER[1])
        rvar1 = 1.0
        rmean = OTHER[3][0]
        rvar = OTHER[3][1]
        [rupdated, rvarupdated] = update(rmean1, rvar1, rmean, rvar)

        #Update x and y with movement
        xmove = rupdated*cos(HA3updated)
        ymove = rupdated*sin(HA3updated)
        xvar = rvarupdated+thetavarupdated
        yvar = rvarupdated+thetavarupdated
        [xmoved,xvarmoved]=predict(xmove, xvar, xupdated, xvarupdated)
        [ymoved,yvarmoved]=predict(ymove, yvar, yupdated, yvarupdated)


        #Make updates to parameters
        OTHER[2] = (HA3updated,HA3varupdated)
        OTHER[3] = (rupdated, rvarupdated)
        OTHER[4] = (xmoved,xvarmoved)
        OTHER[5] = (ymoved, yvarmoved)
        OTHER[7] = (thetaupdated, thetavarupdated)
        xy_estimate = (xmoved,ymoved)

    else:
        xy_estimate = (x3, y3)

    OTHER[0] = (OTHER[1][0], OTHER[1][1])
    OTHER[1] = (measurement[0],measurement[1])
    OTHER[6] = OTHER[6] + 1
    # You must return xy_estimate (x, y), and OTHER (even if it is None)
    # in this order for grading purposes.
    return xy_estimate, OTHER


# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def update(mean1, var1, mean2, var2):
    new_mean = (var2 * mean1 + var1 * mean2) / (var1 + var2)
    new_var = 1.0/(1/var1 + 1/var2)
    return [new_mean, new_var]

def predict(mean1, var1, mean2, var2):
    new_mean = mean1 + mean2
    new_var = var1 + var2
    return [new_mean, new_var]
# This is here to give you a sense for how we will be running and grading
# your code. Note that the OTHER variable allows you to store any
# information that you want.
def demo_grading(estimate_next_pos_fcn, target_bot, OTHER = None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    #For Visualization
    import turtle    #You need to run this locally to use the turtle module
    window = turtle.Screen()
    window.bgcolor('white')
    size_multiplier= 25.0  #change Size of animation
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.1, 0.1, 0.1)
    measured_broken_robot = turtle.Turtle()
    measured_broken_robot.shape('circle')
    measured_broken_robot.color('red')
    measured_broken_robot.resizemode('user')
    measured_broken_robot.shapesize(0.1, 0.1, 0.1)
    prediction = turtle.Turtle()
    prediction.shape('arrow')
    prediction.color('blue')
    prediction.resizemode('user')
    prediction.shapesize(0.1, 0.1, 0.1)
    prediction.penup()
    broken_robot.penup()
    measured_broken_robot.penup()
    #End of Visualization
    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            print "You got it right! It took you ", ctr, " steps to localize."
            localized = True
        if ctr == 1000:
            print "Sorry, it took you too many steps to localize the target."
        #More Visualization
        measured_broken_robot.setheading(target_bot.heading*180/pi)
        measured_broken_robot.goto(measurement[0]*size_multiplier, measurement[1]*size_multiplier-200)
        measured_broken_robot.stamp()
        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-200)
        broken_robot.stamp()
        prediction.setheading(target_bot.heading*180/pi)
        prediction.goto(position_guess[0]*size_multiplier, position_guess[1]*size_multiplier-200)
        prediction.stamp()
        #End of Visualization
    return localized

# This is a demo for what a strategy could look like. This one isn't very good.
def naive_next_pos(measurement, OTHER = None):
    """This strategy records the first reported position of the target and
    assumes that eventually the target bot will eventually return to that
    position, so it always guesses that the first position will be the next."""
    if not OTHER: # this is the first measurement
        OTHER = measurement
    xy_estimate = OTHER
    return xy_estimate, OTHER

# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.
test_target = robot(2.1, 4.3, 0.5, 2*pi / 34.0, 1.5)
measurement_noise = 0.05 * test_target.distance
test_target.set_noise(0.0, 0.0, measurement_noise)

demo_grading(estimate_next_pos, test_target)




