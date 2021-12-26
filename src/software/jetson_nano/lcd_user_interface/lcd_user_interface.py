import digitalio
import board
from PIL import Image, ImageDraw, ImageOps
import adafruit_rgb_display.st7735 as st7735

# Configuration for CS and DC pins (these are PiTFT defaults):
CS_PIN = digitalio.DigitalInOut(board.CE0)      # Pin 24
DC_PIN = digitalio.DigitalInOut(board.D25)      # Pin 22
RESET_PIN = digitalio.DigitalInOut(board.D24)   # Pin 18

# Config for display baudrate (default max is 24mhz):
BAUDRATE = 24000000
# Config the proper screen rotation
ROTATION = 90

# Setup SPI bus using hardware SPI:
SPI = board.SPI()

# We will use RGB colour model
COLOUR_MODEL = 'RGB'

class lcd_display:
    def __init__(self):
        """ Create a lcd_dislpay object """
        # Create the display for 1.8" ST7735R:
        self.disp = st7735.ST7735R(SPI, rotation=ROTATION, cs=CS_PIN, dc=DC_PIN, rst=RESET_PIN, baudrate=BAUDRATE)
        
        # We swap height/width to rotate it to landscape
        self.width = self.disp.height
        self.height = self.disp.width

        # Initalize to an empty black screen
        self.clear_screen()

    def clear_screen(self):
        """ Clear this LCD dislpay, make it black """
        # Create blank image for drawing.
        image = Image.new(COLOUR_MODEL, (self.width, self.height))

        # Get drawing object to draw on image.
        draw = ImageDraw.Draw(image)

        # Draw a black filled box to clear the image.
        draw.rectangle((0, 0, self.width, self.height), outline=0, fill=(0, 0, 0))
        self.disp.image(image)

    def draw_logo(self):
        """ Draw Thunderbots Logo on this LCD display """
        self.clear_screen()

        image = Image.open("./imgs/tbots.jpg")
        image = ImageOps.invert(image)  # Image has inverted color

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
