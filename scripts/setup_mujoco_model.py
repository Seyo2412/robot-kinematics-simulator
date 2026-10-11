import os
import urllib.request
import zipfile

def download_ur5e_model():
    models_dir = "models"
    os.makedirs(models_dir, exist_ok=True)
    
    url = "https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/main/universal_robots_ur5e/ur5e.xml"
    target_path = os.path.join(models_dir, "ur5e.xml")
    
    print("Lade offizielles MuJoCo UR5e Modell herunter...")
    urllib.request.urlretrieve(url, target_path)
    print(f"Modell gespeichert unter: {target_path}")

if __name__ == "__main__":
    download_ur5e_model()