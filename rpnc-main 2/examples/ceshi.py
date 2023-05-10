import numpy as np
frame = np.random.randint(low=0, high=2, size=64)
print(frame)
noise_power = 0.1
noise = np.sqrt(noise_power) * np.random.randn(*frame.shape)
print(noise)
frame = frame + noise
print(frame)