from keras.models import load_model
import os


# The path of the model
file_path = os.path.join(os.path.dirname(__file__), '../../data collection_2 (Z)/Test/Z_best_model.h5')

# Check if the file exists
if os.path.exists(file_path):
    model = load_model(file_path)
else:
    print("File not found.")