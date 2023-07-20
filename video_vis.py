import cv2
import os

path = "/media/parvez_alam/Expansion/Kitti/Odometry/data_odometry_gray/dataset/sequences/00/image_0"

img_files = sorted(os.listdir(path))

for img in img_files:
    img_path = os.path.join(path, img)
    im = cv2.imread(img_path)

    cv2.imshow("Image", im)
    
    # Set a small delay between frames (time in milliseconds)
    cv2.waitKey(100)  # Adjust the delay time as needed, e.g., 100 ms for 10 frames per second
    
    # Close the image window if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

