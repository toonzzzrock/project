import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import pickle
global kernel
margin = 0
no_of_windows = 15
kernel = np.ones((5,5),np.uint8)
dirt = r'D:\4.14M\project\lane detect\opencv_frame_0.png'
xpos = ypos = 0
h = 0 
w = 0
a = [[204, 198], [466, 198], [635, 351], [23, 333]]
if len(a) == 0:
    a_ch = True 
else:
    a_ch = False
clicked = False
def get_lane_line_indices_sliding_windows(image, plot=False):
    """
    Get the indices of the lane line pixels using the 
    sliding windows technique.
         
    :param: plot Show plot or not
    :return: Best fit lines for the left and right lines of the current lane 
    """
    # Sliding window width is +/- margin
    
 
    frame_sliding_window = image.copy()
    
    # Set the height of the sliding windows
    window_height = np.int(h/no_of_windows)   
 
    # Find the x and y coordinates of all the nonzero 
    # (i.e. white) pixels in the frame. 
    nonzero = image.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1]) 
         
    # Store the pixel indices for the left and right lane lines
    left_lane_inds = []
    right_lane_inds = []
         
    # Current positions for pixel indices for each window,
    # which we will continue to update
    leftx_base, rightx_base = histogram_peak(image)
    leftx_current = leftx_base
    rightx_current = rightx_base
 
    # Go through one window at a time
    
         
    for window in range(no_of_windows):
       
      # Identify window boundaries in x and y (and right and left)
      win_y_low = image.shape[0] - (window + 1) * window_height
      win_y_high = image.shape[0] - window * window_height
      win_xleft_low = leftx_current - margin
      win_xleft_high = leftx_current + margin
      win_xright_low = rightx_current - margin
      win_xright_high = rightx_current + margin
      cv2.rectangle(frame_sliding_window,(win_xleft_low,win_y_low),(
        win_xleft_high,win_y_high), (255,255,255), 2)
      cv2.rectangle(frame_sliding_window,(win_xright_low,win_y_low),(
        win_xright_high,win_y_high), (255,255,255), 2)
 
      # Identify the nonzero pixels in x and y within the window
      good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                          (nonzerox >= win_xleft_low) & (
                           nonzerox < win_xleft_high)).nonzero()[0]
      good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                           (nonzerox >= win_xright_low) & (
                            nonzerox < win_xright_high)).nonzero()[0]
                                                         
      # Append these indices to the lists
      left_lane_inds.append(good_left_inds)
      right_lane_inds.append(good_right_inds)
         
      # If you found > minpix pixels, recenter next window on mean position
      minpix = int(h/24) 
      if len(good_left_inds) > minpix:
        leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
      if len(good_right_inds) > minpix:        
        rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
                     
    # Concatenate the arrays of indices
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)
 
    # Extract the pixel coordinates for the left and right lane lines
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds] 
    rightx = nonzerox[right_lane_inds] 
    righty = nonzeroy[right_lane_inds]
 
    # Fit a second order polynomial curve to the pixel coordinates for
    # the left and right lane lines
    #left_fit = np.polyfit(leftx, h-lefty, 2)
    #right_fit = np.polyfit(rightx, h-righty, 2)
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)
    
    plt.scatter(leftx, h-lefty)
    plt.scatter(rightx, h-righty)
    x = np.linspace(0, 300, 1000)
    y = left_fit[0]*x**2 + left_fit[1]*x + left_fit[2]
    plt.plot(x, y)
    x = np.linspace(1100, 1200, 1000)
    y = right_fit[0]*x**2 + right_fit[1]*x + right_fit[2]
    plt.plot(x, y)
    plt.show()
    
    if plot==True:
         
      # Create the x and y values to plot on the image  
      ploty = np.linspace(
        0, frame_sliding_window.shape[0]-1, frame_sliding_window.shape[0])
      left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
      right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
 
      # Generate an image to visualize the result
      out_img = np.dstack((
        frame_sliding_window, frame_sliding_window, (
        frame_sliding_window))) * 255
             
      # Add color to the left line pixels and right line pixels
      out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
      out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [
        0, 0, 255]
      if plot:
          # Plot the figure with the sliding windows
          figure, ( ax2, ax3) = plt.subplots(2,1) # 3 rows, 1 column
          figure.set_size_inches(10, 10)
          figure.tight_layout(pad=3.0)
          ax2.imshow(frame_sliding_window, cmap='gray')
          ax3.imshow(out_img)
          ax3.plot(left_fitx, ploty, color='yellow')
          ax3.plot(right_fitx, ploty, color='yellow')  
          ax2.set_title("Warped Frame with Sliding Windows")
          ax3.set_title("Detected Lane Lines with Sliding Windows")
          plt.show()        
             
    return left_fit, right_fit
def abs_sobel_thresh(img, orient='x', thresh_min=20, thresh_max=100):
	"""
	Takes an image, gradient orientation, and threshold min/max values
	"""
	# Convert to grayscale
	gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
	# Apply x or y gradient with the OpenCV Sobel() function
	# and take the absolute value
	if orient == 'x':
		abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 1, 0))
	if orient == 'y':
		abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 0, 1))
	# Rescale back to 8 bit integer
	scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))
	# Create a copy and apply the threshold
	binary_output = np.zeros_like(scaled_sobel)
	# Here I'm using inclusive (>=, <=) thresholds, but exclusive is ok too
	binary_output[(scaled_sobel >= thresh_min) & (scaled_sobel <= thresh_max)] = 1

	# Return the result
	return binary_output

def mag_thresh(img, sobel_kernel=3, mag_thresh=(30, 75)):
	"""
	Return the magnitude of the gradient
	for a given sobel kernel size and threshold values
	"""
	# Convert to grayscale
	gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
	# Take both Sobel x and y gradients
	sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
	sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
	# Calculate the gradient magnitude
	gradmag = np.sqrt(sobelx**2 + sobely**2)
	# Rescale to 8 bit
	scale_factor = np.max(gradmag)/255
	gradmag = (gradmag/scale_factor).astype(np.uint8)
	# Create a binary image of ones where threshold is met, zeros otherwise
	binary_output = np.zeros_like(gradmag)
	binary_output[(gradmag >= mag_thresh[0]) & (gradmag <= mag_thresh[1])] = 1

	# Return the binary image
	return binary_output


def dir_threshold(img, sobel_kernel=3, thresh=(0, np.pi/3)):
	"""
	Return the direction of the gradient
	for a given sobel kernel size and threshold values
	"""
	# Convert to grayscale
	gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
	# Calculate the x and y gradients
	sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
	sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
	# Take the absolute value of the gradient direction,
	# apply a threshold, and create a binary image result
	absgraddir = np.arctan2(np.absolute(sobely), np.absolute(sobelx))
	binary_output =  np.zeros_like(absgraddir)
	binary_output[(absgraddir >= thresh[0]) & (absgraddir <= thresh[1])] = 1

	# Return the binary image
	return binary_output

def H(image):
    thresh2 = (120, 220)
    B = image[:,:,0]
    '''
    plt.subplot(2, 3, 1)
    plt.imshow( image[:,:,0], vmin=0, vmax=255)
    plt.subplot(2, 3, 2)
    plt.imshow( image[:,:,1], vmin=0, vmax=255)
    plt.subplot(2, 3, 3)
    plt.imshow( image[:,:,2], vmin=0, vmax=255)
    '''

    image = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
    '''
    plt.subplot(2, 3, 4)
    plt.imshow(image[:,:,0], cmap='gray', vmin=0, vmax=100)
    plt.subplot(2, 3, 5)
    plt.imshow(image[:,:,1], cmap='gray', vmin=0, vmax=100)
    plt.subplot(2, 3, 6)
    plt.imshow(image[:,:,2], cmap='gray', vmin=0, vmax=100)
    plt.show()
    '''
    H = image[:,:,0]
    L = image[:,:,1]
    binaryG = np.zeros_like(B)
    binaryG[((H > thresh2[0]) & (H < thresh2[1])) | ((L > thresh2[0]) &( L < thresh2[1]))] = 1
    #binaryG[((B >  thresh2[0]) & (B <  thresh2[1])) | ((L > thresh2[0]) & (L <= thresh2[1]))] = 1
    return binaryG

def hls_thresh(img, thresh=(100, 255)):
	"""
	Convert RGB to HLS and threshold to binary image using S channel
	"""
	hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
	s_channel = hls[:,:,2]
	binary_output = np.zeros_like(s_channel)
	binary_output[(s_channel > thresh[0]) & (s_channel <= thresh[1])] = 1
	return binary_output


def combined_thresh(img):
	abs_bin = abs_sobel_thresh(img, orient='x', thresh_min=50, thresh_max=255)
	mag_bin = mag_thresh(img, sobel_kernel=3, mag_thresh=(20, 100))
	dir_bin = dir_threshold(img, sobel_kernel=15, thresh=(0.6, 1.4))
	hls_bin = H(img)
	combined = np.zeros_like(dir_bin)
	combined[(hls_bin == 0) | (((mag_bin == 1) & (abs_bin == 1))) ] = 1
    #combined[(((mag_bin == 1) & (dir_bin == 1)) | (hls_bin == 1))] = 1
	return combined, abs_bin, mag_bin, dir_bin, hls_bin  # DEBUG


def image_blur(image, kelnel_size = (5, 5)):
    return cv2.GaussianBlur(image, kelnel_size, cv2.BORDER_DEFAULT)
def region(w,h): 
    left_top = [w/2-60,h/2]
    right_top = [w/2+60,h/2]
    left_bottom = [0,h-80]
    right_bottom = [w,h-80]

    fit_left = np.polyfit((left_bottom[0], left_top[0]), (left_bottom[1], left_top[1]), 1)
    fit_right = np.polyfit((right_bottom[0], right_top[0]), (right_bottom[1], right_top[1]), 1)
    fit_up = np.polyfit((left_top[0], right_top[0]), (left_top[1], right_top[1]), 1)
    fit_bottom = np.polyfit((left_bottom[0], right_bottom[0]), (left_bottom[1], right_bottom[1]), 1)
     
    XX, YY = np.meshgrid(np.arange(0, w), np.arange(0, h))
    region_thresholds = (YY > (XX*fit_left[0] + fit_left[1])) & \
                        (YY > (XX*fit_right[0] + fit_right[1])) & \
                        (YY < (XX*fit_bottom[0] + fit_bottom[1])) & \
                        (YY > (XX*fit_up[0] + fit_up[1]))
    return region_thresholds

def adaptive(image):
    th3 = cv2.adaptiveThreshold(image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
            cv2.THRESH_BINARY,15,3)
    return th3

def click_event(image):
    global a_ch
    if a_ch:
        while 1:
            try:
                cv2.imshow('image', image)
                cv2.waitKey(0)
                global xpos,ypos, clicked
                if (clicked):
                    a.append([xpos,ypos])
                    print(a)
                    if len(a) == 4:
                        break
                clicked = False
            except:
                pass
        a_ch = False
        cv2.destroyAllWindows()
    return a
def bird_eye_view(a,image, plot = False):
    corner_points_array = np.float32(a)
    #corner_points_array = np.float32([tl,tr,br,bl])
    # original image dimensions
    
    # Create an array with the parameters (the dimensions) required to build the matrix
    imgTl = [0,0]
    imgTr = [w,0]
    imgBr = [w,h]
    imgBl = [0,h]
    img_params = np.float32([imgTl,imgTr,imgBr,imgBl])
    
    # Compute and return the transformation matrix , flags=(cv2.INTER_LINEAR)
    matrix = cv2.getPerspectiveTransform(corner_points_array,img_params)
    inv_transformation_matrix = cv2.getPerspectiveTransform(img_params,corner_points_array)
    img_transformed = cv2.warpPerspective(image,matrix,(w,h))
    if plot:
        plt.imshow(cv2.cvtColor(img_transformed, cv2.COLOR_BGR2RGB)) # Show results
        plt.show()
    return img_transformed,inv_transformation_matrix
def pos_function(event, x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        print('clicked')
        global xpos,ypos, clicked
        clicked = True
        xpos = x
        ypos = y
def get_lane_line_previous_window(image, left_fit, right_fit):
    """
    Use the lane line from the previous sliding window to get the parameters
    for the polynomial line for filling in the lane line
    :param: left_fit Polynomial function of the left lane line
    :param: right_fit Polynomial function of the right lane line
    :param: plot To display an image or not
    """

 
    # Find the x and y coordinates of all the nonzero 
    # (i.e. white) pixels in the frame.         
    nonzero = image.nonzero()  
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
         
    # Store left and right lane pixel indices
    left_lane_inds = ((nonzerox > (left_fit[0]*(
      nonzeroy**2) + left_fit[1]*nonzeroy + left_fit[2] - margin)) & (
      nonzerox < (left_fit[0]*(
      nonzeroy**2) + left_fit[1]*nonzeroy + left_fit[2] + margin))) 
          
    right_lane_inds = ((nonzerox > (right_fit[0]*(
      nonzeroy**2) + right_fit[1]*nonzeroy + right_fit[2] - margin)) & (
      nonzerox < (right_fit[0]*(
      nonzeroy**2) + right_fit[1]*nonzeroy + right_fit[2] + margin)))           

 
    # Get the left and right lane line pixel locations  
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds] 
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]  
   
     
    # Fit a second order polynomial curve to each lane line
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)
         
    # Create the x and y values to plot on the image
    ploty = np.linspace(
      0, image.shape[0]-1, image.shape[0]) 
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    return ploty,left_fitx,right_fitx
def overlay_lane_lines(image,ori,ploty,left_fitx,right_fitx,inv_transformation_matrix,plot = False):
    """
    Overlay lane lines on the original frame
    :param: Plot the lane lines if True
    :return: Lane with overlay
    """
    # Generate an image to draw the lane lines on 
    warp_zero = np.zeros_like(image).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))       
         
    # Recast the x and y points into usable format for cv2.fillPoly()
    pts_left = np.array([np.transpose(np.vstack([
                         left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([
                          right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))
         
    # Draw lane on the warped blank image
    cv2.fillPoly(color_warp, np.int_([pts]), (0,255, 0))
 
    # Warp the blank back to original image space using inverse perspective 
    # matrix (Minv)
    newwarp = cv2.warpPerspective(color_warp, inv_transformation_matrix,(w, h))
     
    # Combine the result with the original image
    result = cv2.addWeighted(ori, 1, newwarp, 0.3, 0)
    if plot:
        # Plot the figures 
        figure, (ax1, ax2) = plt.subplots(2,1) # 2 rows, 1 column
        figure.set_size_inches(10, 10)
        figure.tight_layout(pad=3.0)
        ax1.imshow(ori)
        ax2.imshow(result)
        ax1.set_title("Original Frame")  
        ax2.set_title("Original Frame With Lane Overlay")
        plt.show()   
 
    return result    
def histogram_peak(image,plot = False):
    """
    Get the left and right peak of the histogram
 
    Return the x coordinate of the left histogram peak and the right histogram
    peak.
    """
    image = np.array(image)
    #image =np.divide(image, 255)
    histogram = np.sum(image, axis=0)
    
    
    midpoint = np.int(histogram.shape[0]/2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint
    
    if plot:
        plt.plot(histogram)
        plt.show()
    # (x coordinate of left peak, x coordinate of right peak)
    return leftx_base, rightx_base
cv2.namedWindow('image')
cv2.setMouseCallback('image',pos_function)
def main(image,plots = False):
    global h,w,margin
    #image = cv2.imread(dirt)
    h,w,_ = image.shape
    #image = image[int(h/2):]
    
    #h,w,_ = image.shape
    margin = int(w/20)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    a = click_event(image)
    image = image_blur(image)
    combined, abs_bin, mag_bin, dir_bin, hls_bin = combined_thresh(image.copy())
    if plots:
        plt.subplot(2, 3, 1)
        plt.imshow(abs_bin, cmap='gray', vmin=0, vmax=1)
        plt.subplot(2, 3, 2)
        plt.imshow(mag_bin, cmap='gray', vmin=0, vmax=1)
        plt.subplot(2, 3, 3)
        plt.imshow(dir_bin, cmap='gray', vmin=0, vmax=1)
        plt.subplot(2, 3, 4)
        plt.imshow(hls_bin, cmap='gray', vmin=0, vmax=1)
        plt.subplot(2, 3, 5)
        plt.imshow(image)
        plt.subplot(2, 3, 6)
        plt.imshow(combined, cmap='gray', vmin=0, vmax=1)
        plt.show()
    combined[combined == 1] = 255
    #combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel)
    #combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)
    combined = np.array(combined, dtype='uint8')
    bird,inv_transformation_matrix = bird_eye_view(a,combined,plot = plots)
    left_fit,right_fit = get_lane_line_indices_sliding_windows(bird,plot = plots)
    #ploty,left_fitx,right_fitx = get_lane_line_previous_window(bird, left_fit, right_fit)
    #result = overlay_lane_lines(combined,img_ori,ploty,left_fitx,right_fitx,inv_transformation_matrix,plot = plots)
    #print(left_fit,right_fit)
    #calculate_car_position(left_fit,right_fit, print_to_terminal=True )
    #return result

def calculate_car_position(left_fit,right_fit, XM_PER_PIX = 3.7 / 781, print_to_terminal=False ):
    """
    Calculate the position of the car relative to the center
         
    :param: print_to_terminal Display data to console if True       
    :return: Offset from the center of the lane
    """
    # Assume the camera is centered in the image.
    # Get position of car in centimeters
    car_location = w / 2
 
    # Fine the x coordinate of the lane line bottom
    bottom_left = left_fit[0]*h**2 + left_fit[
      1]*h + left_fit[2]
    bottom_right = right_fit[0]*h**2 + right_fit[
      1]*h + right_fit[2]
 
    center_lane = (bottom_right + bottom_left)/2
    center_offset = (np.abs(car_location) - np.abs(
      center_lane)) * XM_PER_PIX * 100
 
    if print_to_terminal == True:
      print(str(center_offset) + 'cm')

    return center_offset
'''
cap = cv2.VideoCapture(r'D:\4.14M\project\image\project_video.mp4')
w = int(cap.get(3))
h = int(cap.get(4))

#image = cv2.imread(r'D:\CV2\test6.png', 0)
fourcc = cv2.VideoWriter_fourcc(*"MJPG")
#out = cv2.VideoWriter(r'D:\4.14M\project\image\result2.avi', fourcc, 10.0, (1080, 720))
i=0
l = []
while cap.isOpened():#0.04 s/t
        success, image = cap.read()

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break
        if cv2.waitKey(5) & 0xFF == ord('w'):
            cv2.imwrite(r'D:\4.14M\project\image\rees'+str(i)+'.jpg',image)
            l.append(i)
        try:
            img = main(image)
        except:
            continue
        cv2.imshow('frame', img)        
        #out.write(img)
        
        i+=1
        
cv2.destroyAllWindows()
cap.release()
#out.release()'''

if __name__ == "__main__":
    image = cv2.imread(dirt)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    img = main(image,plots = False)