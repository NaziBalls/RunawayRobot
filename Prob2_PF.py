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
    N= 1000
    debug = False

    x3 = measurement[0]
    y3 = measurement[1]

    #                                      COUNT P
    if OTHER == None:
        OTHER = [measurement, measurement, 0,   []]
        xy_estimate = measurement
        p =[]
    #Run second iteration without guesses

    elif OTHER[2] == 1:
        xy_estimate = measurement
        p=[]
    #Run third iteration with first guess

    elif OTHER[2]== 2:
        x_est = measurement[0]
        y_est = measurement[1]
        [bearing_est, turning_est] = calculate_heading_angle(measurement,OTHER)
        velocity_est = distance_between(OTHER[0], OTHER[1])




        #Make the particles
        p = []
        for i in range(N):
            x_pos = random.gauss(x_est, 0.001)
            y_pos = random.gauss(y_est, 0.001)
            bearing = random.gauss(bearing_est, 0.001)
            turning = random.gauss(turning_est, 0.001)
            velocity = random.gauss(velocity_est, 0.001)
            p.append([x_pos, y_pos, bearing, turning, velocity])
            x_est = p[0][0]
            y_est = p[0][1]
            xy_estimate = (x_est, y_est)

            if (debug):
                print("The particles initially created are ", p[i])

    else:

        [HA, theta] = calculate_heading_angle(measurement,OTHER)



        p = OTHER[3]

        #Discard and inject 10% samples
        for i in range(N/10):
            x_est = measurement[0]
            y_est = measurement[1]
            [bearing_est, turning_est] = calculate_heading_angle(measurement,OTHER)
            velocity_est = distance_between(OTHER[0], OTHER[1])


            x_pos = random.gauss(x_est, 0.001)
            y_pos = random.gauss(y_est, 0.001)
            bearing = random.gauss(bearing_est, 0.001)
            turning = random.gauss(turning_est, 0.001)
            velocity = random.gauss(velocity_est, 0.001)
            p[i]=[x_pos, y_pos, bearing, turning, velocity]
            if (debug):
                print ("Your reinjected particles at step ", OTHER[2], "are ", p[i])



        #Measurement Update
        p = OTHER[3]
        w = []
        for i in range(N):
            w.append(weight(p[i], measurement, OTHER))


        # resampling
        p3 = []
        index = int(random.random() * N)
        beta = 0.0
        mw = max(w)
        for i in range(N):
            beta += random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % N
            newp = [p[index][0], p[index][1], p[index][2], p[index][3], p[index][4]]
            p3.append(newp)
            if (debug):
                print("Your resampled particles at step ", OTHER[2], "are ", p[index])
        p = p3




        # Motion update
        for i in range(N):

            if (debug):
                print("Your particles start at ", p[i], "at step ", OTHER[2])

            velocity = p[i][4]
            bearing = p[i][2]
            p[i][0] = p[i][0] + velocity*cos(bearing)
            p[i][1] = p[i][1] + velocity*sin(bearing)
            p[i][2] = p[i][2] + p[i][3]

            if(debug):
                print("And move to ", p[i])

        #Return a particle with a high importance weight
        index = w.index(max(w))
        x_est = p[index][0]
        y_est = p[index][1]

        xy_estimate = (x_est, y_est)

    OTHER[0] = (OTHER[1][0], OTHER[1][1])
    OTHER[1] = (measurement[0],measurement[1])
    OTHER[2] = OTHER[2]+1
    OTHER[3] = p

    return xy_estimate, OTHER



def weight(particle, meas1, OTHER):
    [bearing, turning] = calculate_heading_angle(meas1, OTHER)
    x1 = meas1[0]
    y1 = meas1[1]

    distancex = abs(x1-particle[0])
    distancey = abs(y1-particle[1])
    distancebearing = abs(bearing-particle[2])
    distanceturning = abs(turning-particle[3])
    w = 5/(distancex+distancey+distancebearing+distanceturning)
    return w








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
    particles = []
    for i in range(100):
        particle = turtle.Turtle()
        particle.shape('circle')
        particle.color('black')
        particle.resizemode('user')
        particle.shapesize(.01, .01, .01)
        particle.penup()
        particles.append(particle)

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

        #for i in range(100):
        #    if ctr>2:
        #        particles[i].goto(OTHER[3][i][0]*size_multiplier, OTHER[3][i][1]*size_multiplier-200)
        #        particles[i].stamp()
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




