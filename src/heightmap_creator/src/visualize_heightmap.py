#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import sys
fname = sys.argv[1]
image = Image.open(fname).convert("L")
arr = np.asarray(image)
print arr[100]
plt.imshow(arr,cmap='gray')
plt.show()
