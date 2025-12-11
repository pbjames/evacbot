print("fire_animator running")
from controller import Supervisor

sup = Supervisor()
timestep = int(sup.getBasicTimeStep())

texA = sup.getFromDef("FIRE_TEX_A")
texB = sup.getFromDef("FIRE_TEX_B")
urlA = texA.getField("url")  # MFString
urlB = texB.getField("url")

N = 20            # 你有多少帧就写多少
i = 0
frame_every = 2   # 每2个step换一帧（想更快就设1）

count = 0
while sup.step(timestep) != -1:
    count += 1
    if count % frame_every != 0:
        continue

    path = f"../textures/fire/fire_{i:03d}.png"
    urlA.setMFString(0, path)
    urlB.setMFString(0, path)
    i = (i + 1) % N
print("texA:", texA, "texB:", texB)