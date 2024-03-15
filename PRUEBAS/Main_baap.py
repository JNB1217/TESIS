import cv2
import mediapipe as mp
import numpy as np
import serial as ser
import math as math
from math import *
import serial 
import time

ser = serial.Serial('COM18', baudrate=9600)
# Inicializa un contador de tiempo
last_send_time = time.time()

# Inicializa las bibliotecas de MediaPipe para la mano y la pose del cuerpo
mp_hands = mp.solutions.hands
mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# Inicializa la captura de video desde la cámara
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

# Pulgar
thumb_points = [1, 2, 4]

# Índice, medio, anular y meñique
palm_points = [0, 1, 2, 5, 9, 13, 17]
fingertips_points = [8, 12, 16, 20]
finger_base_points = [6, 10, 14, 18]

# Colores
GREEN = (48, 255, 48)
BLUE = (192, 101, 21)
YELLOW = (0, 204, 255)
PURPLE = (128, 64, 128)
PEACH = (180, 229, 255)
RED = (0, 0, 255)

# Inicializa el modelo de detección de pose de cuerpo completo y de manos
with mp_pose.Pose(
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5) as pose, mp_hands.Hands(
        model_complexity=1,
        max_num_hands=2,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5) as hands:

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        height, width, _ = frame.shape
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Realiza la detección de pose en el cuadro actual
        pose_results = pose.process(frame_rgb)
        fingers_counter = "_"
        thickness = [2, 2, 2, 2, 2]

        if pose_results.pose_landmarks:
            # Extrae el landmark de la cabeza (nariz)
            head = pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.NOSE]
            head_x = int(head.x * width)
            head_y = int(head.y * height)
            
            # Extrae los landmarks del pulgar derecho y la muñeca izquierda
            right_wrist = pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST]
            right_elbow = pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW]
            right_shoulder = pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER]

            # Convierte las coordenadas normalizadas en coordenadas de píxeles
            right_wrist_x = int(right_wrist.x * width)
            right_wrist_y = int(right_wrist.y * height)
            right_elbow_x = int(right_elbow.x * width)
            right_elbow_y = int(right_elbow.y * height)
            right_shoulder_x = int(right_shoulder.x * width)
            right_shoulder_y = int(right_shoulder.y * height)


        # Realiza la detección de la mano en el cuadro actual
        
        hand_results = hands.process(frame_rgb)
        if hand_results.multi_hand_landmarks:
            # Filtra la mano derecha
            left_hand_landmarks = None
            for hand_landmarks in hand_results.multi_hand_landmarks:
                if hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].x < 0.5:
                    left_hand_landmarks = hand_landmarks

            if left_hand_landmarks:
                
                # Obtén la primera mano detectada
                hand_landmarks = hand_results.multi_hand_landmarks[0]
            
                coordinates_thumb = []
                coordinates_palm = []
                coordinates_ft = []
                coordinates_fb = []

                for index in thumb_points:
                    x = int(hand_landmarks.landmark[index].x * width)
                    y = int(hand_landmarks.landmark[index].y * height)
                    coordinates_thumb.append([x, y])

                for index in palm_points:
                    x = int(hand_landmarks.landmark[index].x * width)
                    y = int(hand_landmarks.landmark[index].y * height)
                    coordinates_palm.append([x, y])

                for index in fingertips_points:
                    x = int(hand_landmarks.landmark[index].x * width)
                    y = int(hand_landmarks.landmark[index].y * height)
                    coordinates_ft.append([x, y])

                for index in finger_base_points:
                    x = int(hand_landmarks.landmark[index].x * width)
                    y = int(hand_landmarks.landmark[index].y * height)
                    coordinates_fb.append([x, y])

                # Pulgar
                p1 = np.array(coordinates_thumb[0])
                p2 = np.array(coordinates_thumb[1])
                p3 = np.array(coordinates_thumb[2])

                l1 = np.linalg.norm(p2 - p3)
                l2 = np.linalg.norm(p1 - p3)
                l3 = np.linalg.norm(p1 - p2)

                # Calcular el ángulo
                if -1 <= (l1 ** 2 + l3 ** 2 - l2 ** 2) / (2 * l1 * l3) <= 1:
                    angle = degrees(acos((l1 ** 2 + l3 ** 2 - l2 ** 2) / (2 * l1 * l3)))
                else:
                    angle = 0  # O establece un valor predeterminado cuando los valores están fuera del rango válido
                thumb_finger = np.array(False)
                if angle > 150:
                    thumb_finger = np.array(True)

                # Distancias
                coordinates_head = np.array([head_x, head_y])
                coordinates_ft = np.array(coordinates_ft)
                coordinates_fb = np.array(coordinates_fb)

                # Distancias desde la cabeza (nariz) a los puntos de los dedos
                d_head_ft = np.linalg.norm(coordinates_head - coordinates_ft, axis=1)
                d_head_fb = np.linalg.norm(coordinates_head - coordinates_fb, axis=1)
                dif = d_head_ft - d_head_fb
                fingers = dif > 0
                fingers = np.append(thumb_finger, fingers)
                fingers_counter = str(np.count_nonzero(fingers == True))
                for (i, finger) in enumerate(fingers):
                    if finger == True:
                        thickness[i] = -1
                        
                fingers_positions = []  # Crear una nueva lista para almacenar los valores

                for i, finger in enumerate(fingers):
                    fingers_positions.append(finger) # Agregar el valor original de finger a la nueva lista
                    
                

                # Calcula la distancia entre las nuevas líneas y la cabeza (nariz)
                distance_head_thumb = np.linalg.norm(
                    [head_x, head_y] - np.array(coordinates_thumb[1]))
                distance_head_pink_tip = np.linalg.norm(
                    [head_x, head_y] - np.array(coordinates_palm[6]))
                
                initial_distance = 140.0
                # Calcula la distancia entre los puntos de interés
                current_distance = np.linalg.norm(
                    np.array([coordinates_thumb[1][0], coordinates_thumb[1][1]]) -
                    np.array([coordinates_palm[6][0], coordinates_palm[6][1]])
                )
                
                turning_hand = ()
                OC_hand = ()
                arm = ()
                
                if not distance_head_thumb > distance_head_pink_tip:
                    turning_hand = "1"
                    if fingers_positions == [1,1,1,1,1]:
                        cv2.putText(frame, "OPENING HAND", (220, 80), 1, 1, (255, 255, 255), 2)
                        OC_hand = "1"
                    
                    if fingers_positions != [1,1,1,1,1]:
                        cv2.putText(frame, "CLOSING HAND", (220, 80), 1, 1, (255, 255, 255), 2)
                        OC_hand = "0"
                if distance_head_thumb > distance_head_pink_tip:
                    cv2.putText(frame, "TURNING HAND", (420, 80), 1, 1, (255, 255, 255), 2)
                    turning_hand = "0"
                    if fingers_positions == [1,1,1,1,1]:
                        cv2.putText(frame, "OPENING HAND", (220, 80), 1, 1, (255, 255, 255), 2)
                        OC_hand = "1"
                    
                    if fingers_positions != [1,1,1,1,1]:
                        cv2.putText(frame, "CLOSING HAND", (220, 80), 1, 1, (255, 255, 255), 2)
                        OC_hand = "0"
                pose_results = pose.process(frame_rgb)
                
                # Extrae los landmarks del hombro y la muñeca izquierda
                right_wrist = pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST]
                right_shoulder = pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER]

                # Convierte las coordenadas normalizadas en coordenadas de píxeles
                right_wrist_x = int(right_wrist.x * width)
                right_wrist_y = int(right_wrist.y * height)
                right_shoulder_x = int(right_shoulder.x * width)
                right_shoulder_y = int(right_shoulder.y * height)

                # Comprueba si se levanta o baja el brazo derecho en función de la posición del hombro y la muñeca izquierda
                if right_shoulder_y > right_wrist_y:
                    cv2.putText(frame, "RAISED ARM", (220, 120), 1, 1, (255, 255, 255), 2)
                    arm = "0"
                else:
                    cv2.putText(frame, "ARM DOWN", (220, 120), 1, 1, (255, 255, 255), 2)
                    arm = "1"
                
                print((OC_hand + turning_hand ).encode('utf-8'))
               # ser.write(OC_hand)
               # Escribe los datos en el puerto serie
               # Envía datos al Arduino cada 10 segundos
        current_time = time.time()
        if current_time - last_send_time >= 2:  # Envia datos cada 0.1 segundos
         ser.write((OC_hand + turning_hand).encode('utf-8'))
         last_send_time = current_time

                #print((OC_hand) + (turning_hand) + (arm))    
              # Cierra el puerto serie
        # Muestra el cuadro de video
        cv2.imshow("BAAP", frame)
        # Detiene el bucle cuando se presiona la tecla 'q'
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
ser.close()
# Libera la captura de video y cierra la ventana
#ser.write(bytes(str(OC_hand) + str(turning_hand) + str(arm), 'utf-8'))
cap.release()
cv2.destroyAllWindows()