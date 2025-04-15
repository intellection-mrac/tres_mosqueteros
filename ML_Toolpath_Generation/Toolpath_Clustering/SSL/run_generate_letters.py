from letter_dataset_utils import generate_letter_dataset

# Choose your letters and image size
letters = ["A", "B", "H", "K", "R"]
generate_letter_dataset(letters, output_folder="letter_dataset", image_size=100)
