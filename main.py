import cv2
import cv2 as cv
import numpy as np
import os
import math
import time
import statistics 
from matplotlib import pyplot as plt
from scipy.optimize import minimize
from scipy import stats
from ltsfit.ltsfit import ltsfit


class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def compute(self, current_value):
        error = self.setpoint - current_value

        # Proportional term
        P = self.kp * error

        # Integral term
        self.integral += error
        I = self.ki * self.integral

        # Derivative term
        D = self.kd * (error - self.prev_error)

        # Compute the control signal
        control_signal = P + I + D

        # Update the previous error for the next iteration
        self.prev_error = error

        return control_signal

def auto_canny(image, sigma=0.33):
    # compute the median of the single channel pixel intensities
    v = np.median(image)
    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    print("upper", upper, "lower:", lower)
    edged = cv2.Canny(image, lower, upper)
    # return the edged image
    print(lower, upper)
    return edged




videofolder = "bilder/video_short"


# Angle control - _ID not working
angle_setpoint = -90  # The desired angle you want to maintain
angle_pid = PIDController(kp=0.5, ki=0.1, kd=0.2, setpoint=angle_setpoint)

# Position control - _ID not working
position_setpoint = 320  # The desired position you want to reach
position_pid = PIDController(kp=0.1, ki=0.01, kd=0.02, setpoint=position_setpoint)

angle_history = []
position_history = []
tid_robust = []
tid_ls = []
tid_polyfit = []
tid_linregres = []
tid_manual = []
tid_median = []
tid_nn_alternativ = []
for filename in sorted(os.listdir(videofolder)):
    print(filename)
    image = cv2.imread(videofolder+"/"+filename)
    # image = cv2.imread("bilder/oya3.jpg")
    
    assert image is not None, "file could not be read, check with os.path.exists()"
    
    height, width, depth = image.shape
    height, width, depth = image.shape
    # print("Height: ", height, "Width: ", width)

    # Crop ROI and resize to 640px widt:
    image = image[height*1//4:, :]
    image = cv2.resize(image, (640, int(height/width*640)), interpolation = cv2.INTER_AREA)

    copy_image = image.copy()
    starttid = time.time()

    height, width, depth = image.shape
    # print("Height: ", height, "Width: ", width)

    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # OPTIONAL: Apply Gaussian blur to reduce noise
    # gray = cv2.GaussianBlur(gray,(11,11),0)
 
    # Use Canny edge detection to find edges in the image
    edges = auto_canny(gray)
  
    
    # Normal Hough transform. Results in rho, theta
    lines = cv2.HoughLines(edges, 1, np.pi / 180, threshold=120)

    # Draw and classify all lines:
    left_rho_theta=[]
    right_rho_theta=[]
    if lines is not None:
        # print(lines)
        # break
        line_left = []
        line_right = []
        for line in lines:
            rho,theta = line[0]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            if theta < np.pi/3:
                line_left.append([x1, y1])
                line_left.append([x2, y2])
                cv.line(image,(x1,y1),(x2,y2),(0,255,0),2)
                left_rho_theta.append([rho,theta])
            elif theta > 2*np.pi/3:
                line_right.append([x1, y1])
                line_right.append([x2, y2])
                cv.line(image,(x1,y1),(x2,y2),(0,0,255),2)
                right_rho_theta.append([rho,theta])
            else:   # Not left or right lane:
                # cv.line(image,(x1,y1),(x2,y2),(255,0,0),2)
                pass
            
    
    image=copy_image

    slutttid = time.time()
    tid_nn_alternativ.append(slutttid-starttid)
    
    # Define the function to optimize (sum of squared errors)
    def error_function(params, data):
        a, b = params
        x, y = data.T
        predicted = a * x + b
        error = np.sum((y - predicted)**2)
        return error

    def draw_line(a,b,rgb,width=2,image=image):
        cv.line(image,(0,int(b)),(2000,int(a*2000+b)),rgb,width)

    def rho_theta_to_ab(rho, theta):
        # Convert polar coordinates to Cartesian form ax + b
        a = -np.cos(theta) / np.sin(theta)
        b = rho / np.sin(theta)
        return a, b


    # Generate example data (line segments defined by starting and ending points)
    print(tid_median)
    # Initial guesses for the line parameters

    line_left = np.array(line_left)
    line_right = np.array(line_right)    

    x1=line_left[:,0]
    y1=line_left[:,1]
    x2=line_right[:,0]
    y2=line_right[:,1]

    # Choose line fitting method:
    # -------------------------------------------

    # Median(PREFERED for hough transform):
    start=time.time()
    lines = np.array(left_rho_theta)
    rho = np.median(lines[:,0])
    theta = np.median(lines[:,1])
    a,b = rho_theta_to_ab(rho,theta)
    end = time.time()
    tid_median.append(end-start)
    draw_line(a,b,(255,0,0),5)


    start=time.time()
    lines = np.array(right_rho_theta)
    rho = np.median(lines[:,0])
    theta = np.median(lines[:,1])
    a,b = rho_theta_to_ab(rho,theta)
    end = time.time()
    tid_median.append(end-start)
    draw_line(a,b,(255,0,0),5)

    # -------------------------------------------

    # Least Squares Polyfit:
    start = time.time()
    a, b = np.polyfit(x1, y1, 1)
    end = time.time()
    tid_polyfit.append(end-start)
    # draw_line(a,b,(0,100,0),8)

    start = time.time()
    a, b = np.polyfit(x2, y2, 1)
    end = time.time()
    tid_polyfit.append(end-start)
    print("NEW method: ",a,b, " in ", end-start)
    # draw_line(a,b,(0,100,0),8)
    
    # -------------------------------------------

    # Least Squares Linregress:
    start = time.time()
    a, b, r_value, p_value, std_err = stats.linregress(x1,y1)
    end = time.time()
    tid_linregres.append(end-start)
    # draw_line(a,b,(0,140,0),6)

    start = time.time()
    a, b, r_value, p_value, std_err = stats.linregress(x2,y2)
    end = time.time()
    tid_linregres.append(end-start)
    print("NEWNEW method: ",a,b, " in ", end-start)
    # draw_line(a,b,(0,140,0),6)
    
    # -------------------------------------------

    # Least Squares Manual:
    start = time.time()
    xm=np.mean(x1)
    ym=np.mean(y1)
    a=np.sum((x1-xm)*(y1-ym))/np.sum((x1-xm)**2)
    b=ym-(a*xm)
    end = time.time()
    tid_manual.append(end-start)
    draw_line(a,b,(0,180,0),4)

    start = time.time()
    xm=np.mean(x2)
    ym=np.mean(y2)
    a=np.sum((x2-xm)*(y2-ym))/np.sum((x2-xm)**2)
    b=ym-(a*xm)
    end = time.time()
    tid_manual.append(end-start)
    print("NEWNEWNEW method: ",a,b, " in ", end-start)
    draw_line(a,b,(0,180,0),4)

    # -------------------------------------------

    # Least Squares (minimize):
    initial_params = [1, 1]

    start = time.time()
    result = minimize(error_function, initial_params, args=(line_left,))
    end = time.time()
    tid_ls.append(end-start)

    optimal_params = result.x
    a1_optimal, b1_optimal = optimal_params
    print(f"Line: y = {a1_optimal:.2f}x + {b1_optimal:.2f} in {end-start:-2f}")
    cv.line(image,(0,int(b1_optimal)),(2000,int(a1_optimal*2000+b1_optimal)),(0,220,0),2)
    
    start = time.time()
    result = minimize(error_function, initial_params, args=(line_right,))
    end = time.time()
    tid_ls.append(end-start)

    optimal_params = result.x
    a2_optimal, b2_optimal = optimal_params
    print(f"Line: y = {a2_optimal:.2f}x + {b2_optimal:.2f}")
    cv.line(image,(0,int(b2_optimal)),(2000,int(a2_optimal*2000+b2_optimal)),(0,220,0),2)

    # -------------------------------------------

    # Robust line fit

    sigx1=np.ones_like(x1)
    sigy1=np.ones_like(y1)
    sigx2=np.ones_like(x2)
    sigy2=np.ones_like(y2)
    try:
        s=time.time()
        p = ltsfit(x1, y1, sigx1, sigy1, clip=2.6, corr=True, epsy=True,
                frac=None, label='Fitted', label_clip='Clipped',
                legend=True, pivot=None, plot=True, text=True)
        b1_optimal, a1_optimal = p.coef
        tid_robust.append(time.time()-s)
        print(f"Best fitting parameters: {p.coef} in {time.time()-s} seconds")
        cv.line(image,(0,int(b1_optimal)),(2000,int(a1_optimal*2000+b1_optimal)),(0,0,0),4)

        s=time.time()
        p = ltsfit(x2, y2, sigx2, sigy2, clip=2.6, corr=True, epsy=True,
                frac=None, label='Fitted', label_clip='Clipped',
                legend=True, pivot=None, plot=True, text=True)
        b2_optimal, a2_optimal = p.coef
        tid_robust.append(time.time()-s)
        print(f"Best fitting parameters: {p.coef} in {time.time()-s} seconds")
        cv.line(image,(0,int(b2_optimal)),(2000,int(a2_optimal*2000+b2_optimal)),(0,0,0),4)
    except:     # Wierd bug that crashes sometimes
        pass
    # ------------------------------------------
    
    
    def find_intersection(a1, b1, a2, b2):
        if a1 == a2:
            # The lines are parallel and will never intersect
            return None

        x = (b2 - b1) / (a1 - a2)
        y = a1 * x + b1
        return round(x), round(y)
    def find_intersection_with_y_equals_constant(a1, b1, a2, b2, y_constant):
        # Solve for x when y = y_constant in both line equations
        x1 = (y_constant - b1) / a1 if a1 != 0 else None
        x2 = (y_constant - b2) / a2 if a2 != 0 else None

        return (x1+x2) / 2
    

    x_intersect, y_intersect = find_intersection(a1_optimal, b1_optimal, a2_optimal, b2_optimal)
    print(x_intersect, y_intersect)
    image = cv2.circle(image, (int(x_intersect),int(y_intersect)), radius=5, color=(255, 0, 0), thickness=-1)   

    y_robot = height
    x_robot = width/2

    x_center_of_lane_at_bottom = int(find_intersection_with_y_equals_constant(a1_optimal, b1_optimal, a2_optimal, b2_optimal, y_robot))
    image = cv2.circle(image, (x_center_of_lane_at_bottom,y_robot), radius=100, color=(255, 0, 0), thickness=-1)   

    # Center line for lane
    cv.line(image,(x_center_of_lane_at_bottom,y_robot),(int(x_intersect),int(y_intersect)),(0,0,255),2)
   
    # Angle between robot and aiming point(Vanishing point)
    angle_radians = math.atan2(y_intersect - y_robot, x_intersect - x_robot)
    angle = math.degrees(angle_radians)

    # Simulate controllers
    angle_control_signal = -angle_pid.compute(angle)
    position_control_signal = position_pid.compute(x_center_of_lane_at_bottom)
    steering_angle = angle_control_signal + position_control_signal

    # not used:
    angle_history.append(angle)
    position_history.append(x_center_of_lane_at_bottom)

    print("angle and pos:")
    print(angle, x_center_of_lane_at_bottom)
    print("Angle control, position control and steering angle:")
    print(angle_control_signal," + ", position_control_signal," = ", steering_angle)

    # Combine and display the result
    hori1 = np.concatenate((copy_image, image), axis=1) 
    hori2 = np.concatenate((gray, edges), axis=1) 
  
    cv2.imshow("Edges", hori2)
    cv2.imshow("Image lines", image)
    cv2.imwrite("lines.jpg", image)
    cv2.waitKey(0)

    cv2.destroyAllWindows()

def print_tid(l,name="List"):
    print(name+": ")
    print("len: ", len(l))
    print("Mean: ", statistics.mean(l))
    print("stdev: ", statistics.stdev(l))
    print()
 
print_tid(tid_ls, "Least Squares")
print_tid(tid_polyfit, "Polyfit")
print_tid(tid_linregres, "Linregres")
print_tid(tid_manual, "Manual")
print_tid(tid_robust, "LTS")
print_tid(tid_median, "Median")
print_tid(tid_nn_alternativ,"nn alternativ")
