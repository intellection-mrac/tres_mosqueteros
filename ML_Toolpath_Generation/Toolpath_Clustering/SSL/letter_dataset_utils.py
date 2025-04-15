from PIL import Image, ImageDraw, ImageFont
import numpy as np
import os

def letter_to_mask(letter, image_size=200, font_size=160, font_path="arial.ttf"):
    """
    Render a letter into a binary mask (1 = part of the shape, 0 = background).
    """
    img = Image.new("L", (image_size, image_size), color=0)
    draw = ImageDraw.Draw(img)

    try:
        font = ImageFont.truetype(font_path, font_size)
    except OSError:
        print(f"[!] Font not found: {font_path} — using default font.")
        font = ImageFont.load_default()

    bbox = draw.textbbox((0, 0), letter, font=font)
    w = bbox[2] - bbox[0]
    h = bbox[3] - bbox[1]

    draw.text(((image_size - w) / 2, (image_size - h) / 2), letter, fill=255, font=font)
    mask = np.array(img) > 128
    return mask.astype(np.uint8)

def mask_to_valid_points(mask):
    """
    Converts a binary mask to a list of (x, y) coordinates for valid (1) points.
    """
    return np.argwhere(mask == 1)

def save_mask_as_image(mask, output_path):
    """
    Save the binary mask as an image file for visualization.
    """
    img = Image.fromarray((mask * 255).astype(np.uint8))
    img.save(output_path)

def generate_letter_dataset(letters, output_folder="letter_dataset", image_size=200, font_size=160, font_path="arial.ttf"):
    """
    Generates masks and point sets for a list of letters.
    Saves binary mask images and npy arrays of valid points.
    """
    os.makedirs(output_folder, exist_ok=True)
    for letter in letters:
        mask = letter_to_mask(letter, image_size=image_size, font_size=font_size, font_path=font_path)
        points = mask_to_valid_points(mask)

        save_mask_as_image(mask, os.path.join(output_folder, f"{letter}_mask.png"))
        np.save(os.path.join(output_folder, f"{letter}_points.npy"), points)

        print(f"Saved '{letter}' mask and points with {len(points)} valid points.")
