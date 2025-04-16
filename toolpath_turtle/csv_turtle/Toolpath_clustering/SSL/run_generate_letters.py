from letter_dataset_utils import generate_letter_dataset
import os

letters = ["A", "H", "K"]
here = os.path.dirname(__file__)
output_folder = os.path.join(here, "letter_dataset")

font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"  # adjust as needed

generate_letter_dataset(
    letters,
    output_folder=output_folder,
    image_size=300,
    font_size=260,
    font_path=font_path
)
