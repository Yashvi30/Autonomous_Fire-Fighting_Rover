import cv2
import numpy as np
import tensorflow as tf
from tensorflow.keras.preprocessing.image import smart_resize
import serial

# Load the pre-trained fire detection model
model = tf.keras.models.load_model(
    'Final_Fire_Rover\fire_detection_model.h5')

# ESP32-CAM live stream URL (replace with the correct IP address)
esp32_stream_url = 'http://192.168.247.253/stream'

# Initialize video capture from the ESP32-CAM stream
cap = cv2.VideoCapture(esp32_stream_url)

# Set up serial communication with the ESP32 (adjust the port as needed)
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

# Fire detection loop
while cap.isOpened():
    ret, frame = cap.read()  # Capture frame from the live stream
    if ret:
        # Resize the frame to (120, 120) as required by the model
        resized_frame = smart_resize(frame, (120, 120))
        reshaped_frame = np.expand_dims(resized_frame, axis=0)

        # Normalize the image
        normalized_frame = tf.cast(reshaped_frame / 255.0, tf.float32)

        # Make a prediction using the model
        prediction = model.predict(normalized_frame)
        prediction = int(np.round(prediction.flatten()[0]))

        # Display the frame
        cv2.imshow("ESP32-CAM Live Stream", frame)

        # Output prediction (0 = Fire, 1 = No fire)
        if prediction == 0:
            print("Fire Detected!")
            ser.write(b'FIRE\n')  # Send fire signal to ESP32
        else:
            print("No Fire Detected")
            ser.write(b'NO_FIRE\n')  # Send no-fire signal to ESP32

    if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to exit
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
ser.close()
