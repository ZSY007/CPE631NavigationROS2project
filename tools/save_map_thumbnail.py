from pathlib import Path
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

MAP_PGM = Path(r"d:\code\ROS\F2\CPE631-merged\maps\cafe.pgm")
OUT_DIR = Path(r"d:\code\ROS\F2\CPE631-merged\ppt_figures")
OUT_DIR.mkdir(parents=True, exist_ok=True)

img = mpimg.imread(str(MAP_PGM))
# Normalize if needed
try:
    import numpy as np
    if np.issubdtype(img.dtype, np.integer) or img.max() > 1.0:
        img = img.astype(float)
        img = (img - img.min()) / (img.max() - img.min())
except Exception:
    pass

out_path = OUT_DIR / "cafe_map.png"
plt.imsave(str(out_path), img, cmap='gray')
print(f"Saved map thumbnail to: {out_path}")
