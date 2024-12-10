import cv2
import numpy as np


class ArUcoDistanceEstimator:
    def __init__(self, known_distance=20.0, marker_size=6.5):
        # Constantes
        self.KNOWN_DISTANCE = known_distance  # cm
        self.MARKER_SIZE = marker_size  # cm

        # Inicializa el diccionario de ArUco
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        aruco_parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_parameters)

        # Calibración de la cámara (valores de ejemplo; reemplaza con los tuyos)
        self.camera_matrix = np.array([[1080, 0, 540],
                                       [0, 1080, 360],
                                       [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros(
            (4, 1), dtype=np.float32)  # Ajusta según tus datos
        self.cap = None

    def camera_init(self, camera_index=0):
        # Configuración de captura de video
        self.cap = cv2.VideoCapture(camera_index)

    @staticmethod
    def my_estimate_pose_single_markers(corners, marker_size, mtx, distortion):
        marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                  [marker_size / 2, marker_size / 2, 0],
                                  [marker_size / 2, -marker_size / 2, 0],
                                  [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
        rvecs, tvecs = [], []
        for c in corners:
            _, rvec, tvec = cv2.solvePnP(
                marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
            rvecs.append(rvec)
            tvecs.append(tvec)
        return rvecs, tvecs

    @staticmethod
    def estimate_distance(tvec):
        return np.linalg.norm(tvec)

    def process_frame(self, frame):
        # Convertir a escala de grises
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detectar marcadores ArUco
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None:
            # Dibujar los bordes de los marcadores detectados
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Estimar la pose del marcador
            rvecs, tvecs = self.my_estimate_pose_single_markers(
                corners, self.MARKER_SIZE, self.camera_matrix, self.dist_coeffs)

            for rvec, tvec in zip(rvecs, tvecs):
                # Dibujar el eje del marcador
                cv2.drawFrameAxes(frame, self.camera_matrix,
                                  self.dist_coeffs, rvec, tvec, 2)

                # Calcular la distancia
                distance = self.estimate_distance(tvec)

                # Aplicar corrección lineal
                distance = 0.685 * distance - 2.136

                # Mostrar la distancia en la imagen
                cv2.putText(frame, f"Distance: {distance:.2f} cm", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # Verificar si está cerca de 20 cm
                if abs(distance - self.KNOWN_DISTANCE) <= 2.0:
                    print("El marcador está a aproximadamente 20 cm.")

        return frame

    def process_frame_distance_ids(self, frame, base_action):
        # Convertir a escala de grises
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detectar marcadores ArUco
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None:
            # Estimar la pose del marcador
            rvecs, tvecs = self.my_estimate_pose_single_markers(
                corners, self.MARKER_SIZE, self.camera_matrix, self.dist_coeffs)

            for rvec, tvec in zip(rvecs, tvecs):
                # Calcular la distancia
                distance = self.estimate_distance(tvec)

                # Aplicar corrección lineal
                distance = 0.685 * distance - 2.136

                # Verificar si está cerca de 20 cm
                if distance <= self.KNOWN_DISTANCE + 1:
                    print(
                        f"El marcador {ids[0][0]} está a aproximadamente 20 cm.")
                    return ids[0][0]

        return base_action

    def run(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            # Procesar el frame
            frame = self.process_frame(frame)

            # Mostrar el video
            cv2.imshow("ArUco Distance Estimation", frame)

            # Salir con 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()


# Ejemplo de uso
# if __name__ == "__main__":
# estimator = ArUcoDistanceEstimator(camera_index=2)
#    while True:
#        ret, frame = estimator.cap.read()
#
#        if not ret:
#            break
#
#       frame = estimator.process_frame(frame)
#       cv2.imshow("ArUco Distance Estimation", frame)
#
#        if cv2.waitKey(1) & 0xFF == ord('q'):
#            break
