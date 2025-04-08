import cv2
import numpy as np
import os

def generate_aruco_markers(marker_ids, dictionary=cv2.aruco.DICT_4X4_50, marker_size=200, output_folder="markers"):
    """
    Genereert en slaat ArUco-markers op basis van opgegeven ID's.

    :param marker_ids: Lijst van marker ID's die gegenereerd moeten worden.
    :param dictionary: Het type ArUco-marker (standaard 4x4_50).
    :param marker_size: Grootte van de marker in pixels.
    :param output_folder: Map waarin de markers worden opgeslagen.
    """
    # Zorg ervoor dat de outputmap bestaat
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    
    # Laad de ArUco-dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary)
    
    for marker_id in marker_ids:
        marker_img = np.zeros((marker_size, marker_size), dtype=np.uint8)
        cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size, marker_img, 1)
        
        # Opslaan van de marker
        filename = os.path.join(output_folder, f"aruco_{marker_id}.png")
        cv2.imwrite(filename, marker_img)
        print(f"Marker {marker_id} opgeslagen als {filename}")

if __name__ == "__main__":
    # Pas deze lijst aan om andere ID's te genereren
    marker_ids_to_generate = [1, 2, 3, 4]
    generate_aruco_markers(marker_ids_to_generate)
