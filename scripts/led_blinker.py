import neopixel
import board
import time

pixels = neopixel.NeoPixel(
	board.D21,
	50,
	brightness = 0.3,
	auto_write = False,
	pixel_order = 'GRB',
)

pixels.fill((0, 0, 0))
pixels.show()

iterations = 0
while True:
	pixels.fill((0, 0, 0))
	for i in range(10):
		pixels[i*5 + iterations%5] = ( 255, 0, 255 )

	time.sleep(0.1)
	pixels.show()
	iterations += 1
