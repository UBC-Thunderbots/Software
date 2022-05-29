import adafruit_rgb_display.st7735 as st7735
import board
import busio
import digitalio
from PIL import Image, ImageDraw, ImageOps

"""
Information for the display that we are using can be found here:
https://www.adafruit.com/product/358#description

Pin Connections:
    Jetson Nano              LCD Display
        Pin 6  (GND)            Pin 1  (GND)
        Pin 1  (3.3V)           Pin 2  (VCC)
        Pin 18                  Pin 3  (Reset)
        Pin 40                  Pin 4  (D/C) 
        N/C                     Pin 5  (CARD_CS)
        Pin 24 (CS)             Pin 6  (TFT_CS)
        Pin 19 (MOSI)           Pin 7  (MOSI)
        Pin 23 (SCK)            Pin 8  (SCK)
        N/C                     Pin 9  (MISO)
        Pin 1  (3.3V)           Pin 10 (LITE)
"""

# Configuration for CS and DC pins (these are PiTFT defaults):
CS_PIN = digitalio.DigitalInOut(board.D22)
DC_PIN = digitalio.DigitalInOut(board.D21)
RESET_PIN = digitalio.DigitalInOut(board.D24)

# Config for display baudrate to 20 MHz:
BAUDRATE = 2000000

# Config the proper screen rotation
ROTATION = 90

# Setup SPI bus using hardware SPI:
SPI = busio.SPI(board.SCK_1, MOSI=board.MOSI_1, MISO=board.MISO_1)

# We will use RGB colour model
COLOUR_MODEL = "RGB"


class LcdDisplay:
    def __init__(self):
        """ Create a lcd_display object """
        # Create the display for 1.8" ST7735R:
        self.disp = st7735.ST7735R(
            SPI,
            rotation=ROTATION,
            cs=CS_PIN,
            dc=DC_PIN,
            rst=RESET_PIN,
            baudrate=BAUDRATE,
        )

        # Set display dimensions.
        # We swap height/width to rotate display to landscape
        self.width = self.disp.height
        self.height = self.disp.width

        # Create blank image for drawing
        self.image = Image.new(COLOUR_MODEL, (self.width, self.height))
        # Get drawing object to draw on image
        self.draw = ImageDraw.Draw(self.image)

        # Initialize to an empty black screen
        self.clear_screen()

    def clear_screen(self):
        """ Clear this LCD display, make it black """
        # Draw a black filled box to clear the image.
        self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=(0, 0, 0))
        self.disp.image(self.image)

    def draw_image(self, path_to_image):
        """ Draw image on this LCD display """
        self.clear_screen()

        image = Image.open(path_to_image)
        image = ImageOps.invert(image)  # Image has inverted colour

        # Scale the image to the smaller screen dimension
        image_ratio = image.width / image.height
        screen_ratio = self.width / self.height
        if screen_ratio < image_ratio:
            scaled_width = image.width * self.height // image.height
            scaled_height = self.height
        else:
            scaled_width = self.width
            scaled_height = image.height * self.width // image.width
        image = image.resize((scaled_width, scaled_height), Image.BICUBIC)

        # Crop and center the image
        x = scaled_width // 2 - self.width // 2
        y = scaled_height // 2 - self.height // 2
        image = image.crop((x, y, x + self.width, y + self.height))

        # Display image
        self.disp.image(image)

    def prepare(self):
        """ Create a blank rectangle for drawing """
        self.draw.rectangle((0, 20, self.width, self.height), outline=0, fill=0)

    def show(self):
        """ Display the image """
        self.disp.image(self.image)


if __name__ == "__main__":
    path_to_logo = "./imgs/tbots.jpg"

    display = LcdDisplay()
    display.draw_image(path_to_logo)
