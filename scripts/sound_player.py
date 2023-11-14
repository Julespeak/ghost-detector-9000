import pygame

pygame.mixer.init()
sound = pygame.mixer.Sound('audio/ghost_capture.wav')
playing = sound.play()
while playing.get_busy():
    pygame.time.delay(100)
